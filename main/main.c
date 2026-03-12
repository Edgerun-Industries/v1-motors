/**
 * Minimal single-motor CubeMars MIT test (ESP32-S3 + CAN).
 * Goal: from terminal, send "spin_once cw|ccw" and get one controlled revolution.
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "driver/twai.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CAN_TX_GPIO GPIO_NUM_4
#define CAN_RX_GPIO GPIO_NUM_5
#define MOTOR_ID 1

#define CONTROL_PERIOD_MS 5
#define SPIN_ONE_TURN_RAD 6.2831853f
#define FB_TIMEOUT_US 200000LL

#define P_MIN (-12.5f)
#define P_MAX (12.5f)
#define V_MIN (-37.5f)
#define V_MAX (37.5f)
#define T_MIN (-32.0f)
#define T_MAX (32.0f)
#define KP_MIN (0.0f)
#define KP_MAX (500.0f)
#define KD_MIN (0.0f)
#define KD_MAX (5.0f)

static const char *TAG = "cubemars";
static const uint8_t CMD_ENTER_MOTOR_MODE[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
static const uint8_t CMD_EXIT_MOTOR_MODE[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

typedef struct
{
    bool enabled;
    bool fault;
    bool spin_active;
    float target_p;
    int64_t spin_start_us;
    int64_t last_fb_time_us;

    float p_cmd;
    float v_cmd;
    float kp_cmd;
    float kd_cmd;
    float t_cmd;

    bool fb_seen;
    float p_fb;
    float v_fb;
    float t_fb;
    int temp_fb;
    int err_fb;

    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t rx_any_count;
    uint32_t last_rx_can_id;
    uint8_t last_rx_dlc;
    bool last_rx_extd;
} motor_state_t;

static motor_state_t g_state = {
    .enabled = false,
    .fault = false,
    .spin_active = false,
    .p_cmd = 0.0f,
    .v_cmd = 0.0f,
    .kp_cmd = 0.0f,
    .kd_cmd = 1.0f,
    .t_cmd = 0.0f,
    .last_fb_time_us = 0,
};
static portMUX_TYPE g_lock = portMUX_INITIALIZER_UNLOCKED;
static int g_motor_id = MOTOR_ID;
static bool g_log_raw = false;

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    return ((float)x_int) * (x_max - x_min) / ((float)((1 << bits) - 1)) + x_min;
}

static int float_to_uint(float x, float x_min, float x_max, int bits)
{
    if (x < x_min)
        x = x_min;
    if (x > x_max)
        x = x_max;
    return (int)((x - x_min) * ((float)((1 << bits) - 1)) / (x_max - x_min));
}

static esp_err_t send_can_frame(const uint8_t data[8])
{
    int motor_id;
    portENTER_CRITICAL(&g_lock);
    motor_id = g_motor_id;
    portEXIT_CRITICAL(&g_lock);

    twai_message_t msg = {
        .identifier = (uint32_t)motor_id,
        .data_length_code = 8,
        .flags = 0,
    };
    memcpy(msg.data, data, 8);
    return twai_transmit(&msg, pdMS_TO_TICKS(100));
}

static esp_err_t send_can_frame_ext(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    twai_message_t msg = {
        .identifier = can_id,
        .data_length_code = dlc,
        .flags = TWAI_MSG_FLAG_EXTD,
    };
    memset(msg.data, 0, sizeof(msg.data));
    if (data != NULL && dlc > 0)
    {
        memcpy(msg.data, data, dlc);
    }
    return twai_transmit(&msg, pdMS_TO_TICKS(100));
}

static void pack_mit_command(float p, float v, float kp, float kd, float t_ff, uint8_t out[8])
{
    const int p_int = float_to_uint(p, P_MIN, P_MAX, 16);
    const int v_int = float_to_uint(v, V_MIN, V_MAX, 12);
    const int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    const int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    const int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

    out[0] = (uint8_t)(p_int >> 8);
    out[1] = (uint8_t)(p_int & 0xFF);
    out[2] = (uint8_t)(v_int >> 4);
    out[3] = (uint8_t)(((v_int & 0x0F) << 4) | (kp_int >> 8));
    out[4] = (uint8_t)(kp_int & 0xFF);
    out[5] = (uint8_t)(kd_int >> 4);
    out[6] = (uint8_t)(((kd_int & 0x0F) << 4) | (t_int >> 8));
    out[7] = (uint8_t)(t_int & 0xFF);
}

static void print_help(void)
{
    ESP_LOGI(TAG, "Commands:");
    ESP_LOGI(TAG, "  ping");
    ESP_LOGI(TAG, "  status");
    ESP_LOGI(TAG, "  id <1..127>");
    ESP_LOGI(TAG, "  scan_ids");
    ESP_LOGI(TAG, "  raw on|off");
    ESP_LOGI(TAG, "  enable");
    ESP_LOGI(TAG, "  disable");
    ESP_LOGI(TAG, "  stop");
    ESP_LOGI(TAG, "  spin_once cw");
    ESP_LOGI(TAG, "  spin_once ccw");
    ESP_LOGI(TAG, "Note: if extd=1 in status, spin_once uses Servo mode position command.");
    ESP_LOGI(TAG, "Note: FAULT (fb_timeout >200ms) disables motor. Send 'disable' to clear.");
}

static void send_enter_for_id(int id)
{
    twai_message_t msg = {
        .identifier = (uint32_t)id,
        .data_length_code = 8,
        .flags = 0,
    };
    memcpy(msg.data, CMD_ENTER_MOTOR_MODE, 8);
    (void)twai_transmit(&msg, pdMS_TO_TICKS(50));
}

static void print_status(void)
{
    twai_status_info_t tsi = {0};
    twai_get_status_info(&tsi);

    motor_state_t s;
    int motor_id;
    portENTER_CRITICAL(&g_lock);
    s = g_state;
    motor_id = g_motor_id;
    portEXIT_CRITICAL(&g_lock);

    ESP_LOGI(TAG,
             "STATUS id=%d enabled=%d fault=%d tx=%lu rx=%lu rx_any=%lu seen=%d pos=%.3f vel=%.2f tq=%.2f temp=%d err=%d last_can=0x%03lX dlc=%u extd=%d",
             motor_id, s.enabled, s.fault, (unsigned long)s.tx_count, (unsigned long)s.rx_count,
             (unsigned long)s.rx_any_count, s.fb_seen, s.p_fb, s.v_fb, s.t_fb, s.temp_fb, s.err_fb,
             (unsigned long)s.last_rx_can_id, (unsigned)s.last_rx_dlc, (int)s.last_rx_extd);
    ESP_LOGI(TAG,
             "CAN state=%d txerr=%lu rxerr=%lu bus_err=%lu tx_failed=%lu arb_lost=%lu",
             (int)tsi.state, (unsigned long)tsi.tx_error_counter, (unsigned long)tsi.rx_error_counter,
             (unsigned long)tsi.bus_error_count, (unsigned long)tsi.tx_failed_count,
             (unsigned long)tsi.arb_lost_count);
}

static void can_receive_task(void *arg)
{
    (void)arg;
    twai_message_t rx_msg;
    while (1)
    {
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(100)) != ESP_OK)
        {
            continue;
        }

        portENTER_CRITICAL(&g_lock);
        g_state.rx_any_count++;
        g_state.last_rx_can_id = rx_msg.identifier;
        g_state.last_rx_dlc = rx_msg.data_length_code;
        g_state.last_rx_extd = ((rx_msg.flags & TWAI_MSG_FLAG_EXTD) != 0);
        portEXIT_CRITICAL(&g_lock);

        if (g_log_raw)
        {
            ESP_LOGI(TAG, "RX_RAW can_id=0x%03lX extd=%d dlc=%d data=%02X %02X %02X %02X %02X %02X %02X %02X",
                     (unsigned long)rx_msg.identifier, (int)((rx_msg.flags & TWAI_MSG_FLAG_EXTD) != 0), rx_msg.data_length_code,
                     rx_msg.data[0], rx_msg.data[1], rx_msg.data[2], rx_msg.data[3],
                     rx_msg.data[4], rx_msg.data[5], rx_msg.data[6], rx_msg.data[7]);
        }

        if (rx_msg.flags & TWAI_MSG_FLAG_EXTD)
        {
            // Servo-mode feedback frame format
            // CAN ID low byte is controller ID, payload has pos/spd/cur/temp/err.
            int32_t id8 = (int32_t)(rx_msg.identifier & 0xFF);
            int16_t pos_i = (int16_t)((rx_msg.data[0] << 8) | rx_msg.data[1]);
            int16_t spd_i = (int16_t)((rx_msg.data[2] << 8) | rx_msg.data[3]);
            int16_t cur_i = (int16_t)((rx_msg.data[4] << 8) | rx_msg.data[5]);
            float pos_deg = (float)pos_i * 0.1f;
            float spd_erpm = (float)spd_i * 10.0f;
            float cur_a = (float)cur_i * 0.01f;

            portENTER_CRITICAL(&g_lock);
            if (g_motor_id == MOTOR_ID)
            {
                // Auto-adopt first seen servo ID when default id is still set.
                g_motor_id = id8;
            }
            if (id8 == g_motor_id)
            {
                g_state.fb_seen = true;
                g_state.p_fb = pos_deg;
                g_state.v_fb = spd_erpm;
                g_state.t_fb = cur_a;
                g_state.temp_fb = rx_msg.data[6];
                g_state.err_fb = rx_msg.data[7];
                g_state.rx_count++;
                g_state.last_fb_time_us = esp_timer_get_time();
            }
            portEXIT_CRITICAL(&g_lock);
            continue;
        }

        if (rx_msg.data_length_code < 6)
        {
            continue;
        }
        int motor_id;
        portENTER_CRITICAL(&g_lock);
        motor_id = g_motor_id;
        portEXIT_CRITICAL(&g_lock);

        if ((int)rx_msg.identifier != motor_id && rx_msg.data[0] != motor_id)
        {
            continue;
        }

        const int p_int = (rx_msg.data[1] << 8) | rx_msg.data[2];
        const int v_int = (rx_msg.data[3] << 4) | (rx_msg.data[4] >> 4);
        const int i_int = ((rx_msg.data[4] & 0x0F) << 8) | rx_msg.data[5];

        const float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
        const float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
        const float t = uint_to_float(i_int, T_MIN, T_MAX, 12);
        const int temp = (rx_msg.data_length_code > 6) ? ((int)rx_msg.data[6] - 40) : 0;
        const int err = (rx_msg.data_length_code > 7) ? rx_msg.data[7] : 0;

        portENTER_CRITICAL(&g_lock);
        g_state.fb_seen = true;
        g_state.p_fb = p;
        g_state.v_fb = v;
        g_state.t_fb = t;
        g_state.temp_fb = temp;
        g_state.err_fb = err;
        g_state.rx_count++;
        g_state.last_fb_time_us = esp_timer_get_time();
        portEXIT_CRITICAL(&g_lock);
    }
}

static void control_task(void *arg)
{
    (void)arg;
    uint8_t tx[8];
    while (1)
    {
        motor_state_t s;
        portENTER_CRITICAL(&g_lock);
        s = g_state;
        portEXIT_CRITICAL(&g_lock);

        if (s.enabled && !s.fault && s.fb_seen)
        {
            if ((esp_timer_get_time() - s.last_fb_time_us) > FB_TIMEOUT_US)
            {
                portENTER_CRITICAL(&g_lock);
                g_state.fault = true;
                g_state.enabled = false;
                g_state.spin_active = false;
                portEXIT_CRITICAL(&g_lock);
                send_can_frame(CMD_EXIT_MOTOR_MODE);
                ESP_LOGW(TAG, "FAULT reason=fb_timeout last_pos=%.3f", s.p_fb);
                s.enabled = false;
            }
        }

        if (s.enabled && !s.last_rx_extd)
        {
            if (s.spin_active && s.fb_seen)
            {
                const float pos_error = s.target_p - s.p_fb;
                if (fabsf(pos_error) < 0.08f)
                {
                    portENTER_CRITICAL(&g_lock);
                    g_state.spin_active = false;
                    g_state.p_cmd = s.target_p;
                    g_state.v_cmd = 0.0f;
                    g_state.kp_cmd = 20.0f;
                    g_state.kd_cmd = 1.0f;
                    g_state.t_cmd = 0.0f;
                    portEXIT_CRITICAL(&g_lock);
                    ESP_LOGI(TAG, "SPIN_DONE pos=%.3f", s.p_fb);
                }
                else if ((esp_timer_get_time() - s.spin_start_us) > 8000000)
                {
                    portENTER_CRITICAL(&g_lock);
                    g_state.spin_active = false;
                    g_state.enabled = false;
                    portEXIT_CRITICAL(&g_lock);
                    send_can_frame(CMD_EXIT_MOTOR_MODE);
                    ESP_LOGW(TAG, "SPIN_TIMEOUT pos=%.3f target=%.3f", s.p_fb, s.target_p);
                    s.enabled = false;
                }
            }

            pack_mit_command(s.p_cmd, s.v_cmd, s.kp_cmd, s.kd_cmd, s.t_cmd, tx);
            if (send_can_frame(tx) == ESP_OK)
            {
                portENTER_CRITICAL(&g_lock);
                g_state.tx_count++;
                portEXIT_CRITICAL(&g_lock);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

static void handle_command(char *line)
{
    if (line[0] == '\0')
        return;

    if (strcmp(line, "ping") == 0)
    {
        ESP_LOGI(TAG, "PONG");
        return;
    }
    if (strcmp(line, "status") == 0)
    {
        print_status();
        return;
    }
    if (strcmp(line, "help") == 0)
    {
        print_help();
        return;
    }
    if (strcmp(line, "raw on") == 0)
    {
        g_log_raw = true;
        ESP_LOGI(TAG, "ACK raw on");
        return;
    }
    if (strcmp(line, "raw off") == 0)
    {
        g_log_raw = false;
        ESP_LOGI(TAG, "ACK raw off");
        return;
    }
    int new_id = 0;
    if (sscanf(line, "id %d", &new_id) == 1)
    {
        if (new_id < 1 || new_id > 127)
        {
            ESP_LOGW(TAG, "NACK id must be 1..127");
            return;
        }
        portENTER_CRITICAL(&g_lock);
        g_motor_id = new_id;
        g_state.fb_seen = false;
        g_state.rx_count = 0;
        g_state.spin_active = false;
        portEXIT_CRITICAL(&g_lock);
        ESP_LOGI(TAG, "ACK id=%d", new_id);
        return;
    }
    if (strcmp(line, "scan_ids") == 0)
    {
        ESP_LOGI(TAG, "SCAN start ids=1..127");
        portENTER_CRITICAL(&g_lock);
        g_state.rx_any_count = 0;
        g_state.rx_count = 0;
        g_state.fb_seen = false;
        g_state.last_rx_can_id = 0;
        g_state.last_rx_dlc = 0;
        portEXIT_CRITICAL(&g_lock);

        for (int id = 1; id <= 127; id++)
        {
            send_enter_for_id(id);
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        vTaskDelay(pdMS_TO_TICKS(150));
        print_status();
        ESP_LOGI(TAG, "SCAN done");
        return;
    }
    if (strcmp(line, "enable") == 0)
    {
        motor_state_t s_en;
        portENTER_CRITICAL(&g_lock);
        s_en = g_state;
        portEXIT_CRITICAL(&g_lock);
        if (s_en.fault)
        {
            ESP_LOGW(TAG, "NACK enable reason=fault_active send_disable_first");
            return;
        }
        send_can_frame(CMD_ENTER_MOTOR_MODE);
        portENTER_CRITICAL(&g_lock);
        g_state.enabled = true;
        g_state.fb_seen = false;
        portEXIT_CRITICAL(&g_lock);
        ESP_LOGI(TAG, "ACK enable");
        return;
    }
    if (strcmp(line, "disable") == 0)
    {
        portENTER_CRITICAL(&g_lock);
        g_state.enabled = false;
        g_state.fault = false;
        g_state.spin_active = false;
        portEXIT_CRITICAL(&g_lock);
        send_can_frame(CMD_EXIT_MOTOR_MODE);
        ESP_LOGI(TAG, "ACK disable");
        return;
    }
    if (strcmp(line, "stop") == 0)
    {
        portENTER_CRITICAL(&g_lock);
        g_state.spin_active = false;
        g_state.enabled = false;
        g_state.v_cmd = 0.0f;
        g_state.kp_cmd = 0.0f;
        g_state.kd_cmd = 0.0f;
        g_state.t_cmd = 0.0f;
        portEXIT_CRITICAL(&g_lock);
        send_can_frame(CMD_EXIT_MOTOR_MODE);
        ESP_LOGI(TAG, "ACK stop");
        return;
    }
    if (strcmp(line, "spin_once cw") == 0 || strcmp(line, "spin_once ccw") == 0)
    {
        motor_state_t s;
        portENTER_CRITICAL(&g_lock);
        s = g_state;
        portEXIT_CRITICAL(&g_lock);

        if (!s.fb_seen)
        {
            ESP_LOGW(TAG, "NACK spin_once reason=no_feedback");
            return;
        }

        const float sign = (strcmp(line, "spin_once cw") == 0) ? 1.0f : -1.0f;
        // If we are receiving extended frames, use Servo mode set-position command
        // with degrees; otherwise use MIT mode position target (radians).
        if (s.last_rx_extd)
        {
            int motor_id;
            portENTER_CRITICAL(&g_lock);
            motor_id = g_motor_id;
            portEXIT_CRITICAL(&g_lock);

            float target_deg = s.p_fb + (sign * 360.0f);
            int32_t pos_cmd = (int32_t)lroundf(target_deg * 1000000.0f);
            uint8_t payload[4] = {
                (uint8_t)((pos_cmd >> 24) & 0xFF),
                (uint8_t)((pos_cmd >> 16) & 0xFF),
                (uint8_t)((pos_cmd >> 8) & 0xFF),
                (uint8_t)(pos_cmd & 0xFF),
            };
            uint32_t can_id = ((uint32_t)motor_id & 0xFFu) | (4u << 8); // CAN_PACKET_SET_POS
            esp_err_t err = send_can_frame_ext(can_id, payload, 4);
            if (err == ESP_OK)
            {
                ESP_LOGI(TAG, "ACK spin_once servo id=%d target_deg=%.2f can=0x%03lX",
                         motor_id, target_deg, (unsigned long)can_id);
            }
            else
            {
                ESP_LOGW(TAG, "NACK spin_once servo tx_err=%s", esp_err_to_name(err));
            }
            return;
        }

        const float target = s.p_fb + (sign * SPIN_ONE_TURN_RAD);
        portENTER_CRITICAL(&g_lock);
        g_state.enabled = true;
        g_state.spin_active = true;
        g_state.target_p = target;
        g_state.spin_start_us = esp_timer_get_time();
        g_state.p_cmd = target;
        g_state.v_cmd = 0.0f;
        g_state.kp_cmd = 20.0f;
        g_state.kd_cmd = 1.0f;
        g_state.t_cmd = 0.0f;
        portEXIT_CRITICAL(&g_lock);
        send_can_frame(CMD_ENTER_MOTOR_MODE);
        ESP_LOGI(TAG, "ACK spin_once target=%.3f", target);
        return;
    }

    ESP_LOGW(TAG, "NACK unknown_cmd=%s", line);
}

static void serial_task(void *arg)
{
    (void)arg;
    char line[128];
    size_t idx = 0;
    memset(line, 0, sizeof(line));

    print_help();
    ESP_LOGI(TAG, "READY");

    while (1)
    {
        uint8_t ch = 0;
        int n = usb_serial_jtag_read_bytes(&ch, 1, 20 / portTICK_PERIOD_MS);
        if (n <= 0)
            continue;

        if (ch == '\r')
            continue;
        if (ch == '\n')
        {
            line[idx] = '\0';
            handle_command(line);
            idx = 0;
            continue;
        }

        if (idx < (sizeof(line) - 1))
        {
            line[idx++] = (char)ch;
        }
        else
        {
            idx = 0;
            ESP_LOGW(TAG, "NACK line_too_long");
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== Single Motor Spin Test ===");

    usb_serial_jtag_driver_config_t usb_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    esp_err_t usb_err = usb_serial_jtag_driver_install(&usb_cfg);
    if (usb_err != ESP_OK && usb_err != ESP_ERR_INVALID_STATE)
    {
        ESP_ERROR_CHECK(usb_err);
    }

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 128;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI started @ 1Mbps, MOTOR_ID=%d", MOTOR_ID);

    xTaskCreatePinnedToCore(can_receive_task, "can_rx", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(control_task, "ctrl", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(serial_task, "serial", 4096, NULL, 3, NULL, 0);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}