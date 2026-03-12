/**
 * Dual-motor assist controller:
 * - Motor IDs fixed to 1 and 2.
 * - Same smooth assist logic per motor.
 * - Same +/-90 deg clamp per motor.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "driver/twai.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CAN_TX_GPIO GPIO_NUM_4
#define CAN_RX_GPIO GPIO_NUM_5

#define MOTOR_COUNT 2
static const uint8_t MOTOR_IDS[MOTOR_COUNT] = {1, 2};

#define CONTROL_PERIOD_MS 10
#define STATUS_PERIOD_MS 400
#define BUS_STATUS_PERIOD_MS 5000

#define CAN_PACKET_SET_CURRENT 1

#define VEL_LP_ALPHA 0.35f
#define INTENT_START_VEL_ERPM 18.0f
#define INTENT_STOP_VEL_ERPM 8.0f
#define ASSIST_DEADBAND_ERPM 14.0f
#define ASSIST_K_A_PER_ERPM 0.0018f
#define MIN_ASSIST_CURRENT_A 0.08f
#define MAX_ASSIST_CURRENT_A 0.60f

#define CMD_SLEW_UP_A_PER_STEP 0.016f
#define CMD_SLEW_DOWN_A_PER_STEP 0.018f
#define CMD_ZERO_CROSS_STEP 0.022f

#define POS_LIMIT_DEG 90.0f
#define POS_SOFT_LIMIT_DEG 85.0f

typedef struct {
    bool enabled;
    bool fb_seen;
    float pos_deg;
    float vel_erpm;
    float cur_a;
    int temp_c;
    int err_code;

    float vel_filt_erpm;
    float cmd_current_a;
    bool intent_active;
    int human_intent_dir;

    bool zero_ref_set;
    float zero_ref_deg;

    uint32_t tx_count;
    uint32_t rx_count;
} motor_state_t;

typedef struct {
    motor_state_t m[MOTOR_COUNT];
    uint32_t rx_any_count;
} app_state_t;

static app_state_t g = {0};
static const char *TAG = "exo_ctrl";
static portMUX_TYPE g_lock = portMUX_INITIALIZER_UNLOCKED;

static int motor_index_from_id(uint8_t id)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (MOTOR_IDS[i] == id) return i;
    }
    return -1;
}

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float signf0(float x)
{
    if (x > 0.0f) return 1.0f;
    if (x < 0.0f) return -1.0f;
    return 0.0f;
}

static float slew_toward(float current, float target, float max_step)
{
    float delta = target - current;
    if (delta > max_step) return current + max_step;
    if (delta < -max_step) return current - max_step;
    return target;
}

static float smooth_command_update(float current, float target)
{
    if ((current > 0.0f && target < 0.0f) || (current < 0.0f && target > 0.0f)) {
        if (fabsf(current) > CMD_ZERO_CROSS_STEP) {
            return slew_toward(current, 0.0f, CMD_ZERO_CROSS_STEP);
        }
        return slew_toward(0.0f, target, CMD_SLEW_UP_A_PER_STEP);
    }
    if (fabsf(target) > fabsf(current)) {
        return slew_toward(current, target, CMD_SLEW_UP_A_PER_STEP);
    }
    return slew_toward(current, target, CMD_SLEW_DOWN_A_PER_STEP);
}

static esp_err_t send_servo_current(uint8_t id, float current_a)
{
    int32_t cur_raw = (int32_t)(current_a * 1000.0f);
    uint8_t payload[4] = {
        (uint8_t)((cur_raw >> 24) & 0xFF),
        (uint8_t)((cur_raw >> 16) & 0xFF),
        (uint8_t)((cur_raw >> 8) & 0xFF),
        (uint8_t)(cur_raw & 0xFF),
    };

    twai_message_t msg = {
        .identifier = ((uint32_t)CAN_PACKET_SET_CURRENT << 8) | id,
        .data_length_code = 4,
        .flags = TWAI_MSG_FLAG_EXTD,
    };
    memcpy(msg.data, payload, 4);
    return twai_transmit(&msg, pdMS_TO_TICKS(100));
}

static float compute_assist_cmd(motor_state_t *s, float vfilt)
{
    float abs_v = fabsf(vfilt);
    if (!s->intent_active) {
        if (abs_v >= INTENT_START_VEL_ERPM) s->intent_active = true;
    } else if (abs_v <= INTENT_STOP_VEL_ERPM) {
        s->intent_active = false;
    }

    if (vfilt >= INTENT_START_VEL_ERPM) s->human_intent_dir = 1;
    else if (vfilt <= -INTENT_START_VEL_ERPM) s->human_intent_dir = -1;
    else if (abs_v <= INTENT_STOP_VEL_ERPM) s->human_intent_dir = 0;

    if (!s->intent_active) return 0.0f;

    float amp = ASSIST_K_A_PER_ERPM * (abs_v - ASSIST_DEADBAND_ERPM);
    amp = clampf(amp, MIN_ASSIST_CURRENT_A, MAX_ASSIST_CURRENT_A);
    float dir = (float)s->human_intent_dir;
    if (dir == 0.0f) dir = signf0(vfilt);
    return dir * amp;
}

static float apply_position_limit(float cmd_a, const motor_state_t *s)
{
    float rel_deg = s->pos_deg - s->zero_ref_deg;
    if (rel_deg >= POS_LIMIT_DEG || rel_deg <= -POS_LIMIT_DEG) return 0.0f;

    if (rel_deg > POS_SOFT_LIMIT_DEG && cmd_a > 0.0f) {
        float scale = (POS_LIMIT_DEG - rel_deg) / (POS_LIMIT_DEG - POS_SOFT_LIMIT_DEG);
        cmd_a *= clampf(scale, 0.0f, 1.0f);
    } else if (rel_deg < -POS_SOFT_LIMIT_DEG && cmd_a < 0.0f) {
        float scale = (POS_LIMIT_DEG + rel_deg) / (POS_LIMIT_DEG - POS_SOFT_LIMIT_DEG);
        cmd_a *= clampf(scale, 0.0f, 1.0f);
    }
    return cmd_a;
}

static void print_status_lines(void)
{
    app_state_t s;
    portENTER_CRITICAL(&g_lock);
    s = g;
    portEXIT_CRITICAL(&g_lock);

    for (int i = 0; i < MOTOR_COUNT; i++) {
        const motor_state_t *m = &s.m[i];
        float rel_raw = m->pos_deg - m->zero_ref_deg;
        int in_range = (rel_raw >= -POS_LIMIT_DEG && rel_raw <= POS_LIMIT_DEG) ? 1 : 0;
        const char *machine_dir = "none";
        if (m->cmd_current_a > 0.01f) machine_dir = "forward";
        else if (m->cmd_current_a < -0.01f) machine_dir = "backward";

        ESP_LOGI(TAG,
                 "STAT id=%d on=%d in_range=%d intent=%d intent_dir=%d rel=%.1fdeg machine=%.2fA machine_dir=%s mot=%.2fA err=%d",
                 MOTOR_IDS[i], m->enabled, in_range, m->intent_active, m->human_intent_dir, rel_raw,
                 m->cmd_current_a, machine_dir, m->cur_a, m->err_code);
    }
}

static void print_bus_line(void)
{
    twai_status_info_t tsi = {0};
    twai_get_status_info(&tsi);
    app_state_t s;
    portENTER_CRITICAL(&g_lock);
    s = g;
    portEXIT_CRITICAL(&g_lock);

    ESP_LOGI(TAG, "BUS rx_any=%lu can_state=%d txerr=%lu rxerr=%lu",
             (unsigned long)s.rx_any_count, (int)tsi.state,
             (unsigned long)tsi.tx_error_counter, (unsigned long)tsi.rx_error_counter);
}

static void can_rx_task(void *arg)
{
    (void)arg;
    twai_message_t rx;
    while (1) {
        if (twai_receive(&rx, pdMS_TO_TICKS(100)) != ESP_OK) continue;

        portENTER_CRITICAL(&g_lock);
        g.rx_any_count++;
        portEXIT_CRITICAL(&g_lock);

        if (!(rx.flags & TWAI_MSG_FLAG_EXTD) || rx.data_length_code < 8) continue;
        int idx = motor_index_from_id((uint8_t)(rx.identifier & 0xFFu));
        if (idx < 0) continue;

        int16_t p_i = (int16_t)((rx.data[0] << 8) | rx.data[1]);
        int16_t v_i = (int16_t)((rx.data[2] << 8) | rx.data[3]);
        int16_t c_i = (int16_t)((rx.data[4] << 8) | rx.data[5]);

        portENTER_CRITICAL(&g_lock);
        motor_state_t *m = &g.m[idx];
        m->fb_seen = true;
        m->pos_deg = (float)p_i * 0.1f;
        m->vel_erpm = (float)v_i * 10.0f;
        m->cur_a = (float)c_i * 0.01f;
        m->temp_c = rx.data[6];
        m->err_code = rx.data[7];
        m->rx_count++;
        portEXIT_CRITICAL(&g_lock);
    }
}

static void control_task(void *arg)
{
    (void)arg;
    TickType_t last_stat_tick = xTaskGetTickCount();
    TickType_t last_bus_tick = xTaskGetTickCount();

    while (1) {
        app_state_t s;
        portENTER_CRITICAL(&g_lock);
        s = g;
        portEXIT_CRITICAL(&g_lock);

        for (int i = 0; i < MOTOR_COUNT; i++) {
            motor_state_t m = s.m[i];
            if (!m.fb_seen) continue;

            if (!m.zero_ref_set) {
                m.zero_ref_set = true;
                m.zero_ref_deg = m.pos_deg;
                ESP_LOGI(TAG, "EVT id=%d zero_ref_set=%.2fdeg", MOTOR_IDS[i], m.zero_ref_deg);
            }

            float vfilt = m.vel_filt_erpm + VEL_LP_ALPHA * (m.vel_erpm - m.vel_filt_erpm);
            m.vel_filt_erpm = vfilt;

            float cmd_target = 0.0f;
            if (!m.enabled) {
                m.intent_active = false;
                m.human_intent_dir = 0;
            } else {
                cmd_target = compute_assist_cmd(&m, vfilt);
            }
            cmd_target = apply_position_limit(cmd_target, &m);

            float cmd_out = smooth_command_update(m.cmd_current_a, cmd_target);
            if (send_servo_current(MOTOR_IDS[i], cmd_out) == ESP_OK) {
                portENTER_CRITICAL(&g_lock);
                g.m[i].vel_filt_erpm = m.vel_filt_erpm;
                g.m[i].intent_active = m.intent_active;
                g.m[i].human_intent_dir = m.human_intent_dir;
                g.m[i].zero_ref_deg = m.zero_ref_deg;
                g.m[i].zero_ref_set = m.zero_ref_set;
                g.m[i].cmd_current_a = cmd_out;
                g.m[i].tx_count++;
                portEXIT_CRITICAL(&g_lock);
            }
        }

        TickType_t now = xTaskGetTickCount();
        if ((now - last_stat_tick) >= pdMS_TO_TICKS(STATUS_PERIOD_MS)) {
            last_stat_tick = now;
            print_status_lines();
        }
        if ((now - last_bus_tick) >= pdMS_TO_TICKS(BUS_STATUS_PERIOD_MS)) {
            last_bus_tick = now;
            print_bus_line();
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

static void handle_command(char *line)
{
    if (line[0] == '\0') return;
    if (strcmp(line, "help") == 0) {
        ESP_LOGI(TAG, "EVT commands=status|ping");
        return;
    }
    if (strcmp(line, "ping") == 0) {
        ESP_LOGI(TAG, "EVT pong");
        return;
    }
    if (strcmp(line, "status") == 0) {
        print_status_lines();
        print_bus_line();
        return;
    }
    ESP_LOGW(TAG, "WARN unknown_cmd=%s", line);
}

static void serial_task(void *arg)
{
    (void)arg;
    char line[96];
    size_t idx = 0;
    memset(line, 0, sizeof(line));
    ESP_LOGI(TAG, "EVT ready dual ids=%d,%d", MOTOR_IDS[0], MOTOR_IDS[1]);

    while (1) {
        uint8_t ch = 0;
        int n = usb_serial_jtag_read_bytes(&ch, 1, 20 / portTICK_PERIOD_MS);
        if (n <= 0) continue;
        if (ch == '\r') continue;

        if (ch == '\n') {
            line[idx] = '\0';
            handle_command(line);
            idx = 0;
            continue;
        }
        if (idx < (sizeof(line) - 1)) {
            line[idx++] = (char)ch;
        } else {
            idx = 0;
            ESP_LOGW(TAG, "WARN cmd_too_long");
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "EVT boot ids=%d,%d", MOTOR_IDS[0], MOTOR_IDS[1]);
    ESP_LOGI(TAG, "EVT assist_auto_on");
    for (int i = 0; i < MOTOR_COUNT; i++) {
        g.m[i].enabled = true;
    }

    usb_serial_jtag_driver_config_t usb_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    esp_err_t usb_err = usb_serial_jtag_driver_install(&usb_cfg);
    if (usb_err != ESP_OK && usb_err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(usb_err);
    }

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 128;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "EVT twai_started_1mbps");

    xTaskCreatePinnedToCore(can_rx_task, "can_rx", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(control_task, "ctrl", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(serial_task, "serial", 4096, NULL, 3, NULL, 0);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}