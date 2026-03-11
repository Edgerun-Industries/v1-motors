/**
 * Single-motor exo controller with runtime mode switching.
 *
 * Modes:
 * - transparent: very light friction/gravity compensation.
 * - assist: human-torque observer with smooth current assist.
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
#define MOTOR_ID 104

#define CONTROL_PERIOD_MS 10
#define STATUS_PERIOD_MS 500
#define BUS_STATUS_PERIOD_MS 5000

#define CAN_PACKET_SET_CURRENT 1
#define ERPM_TO_RAD_S 0.104719755f
#define DEG_TO_RAD 0.01745329252f

// Observer / assist tuning.
#define VEL_LP_ALPHA 0.35f
#define ACC_LP_ALPHA 0.40f
#define J_EQUIV 0.0035f
#define B_EQUIV 0.0300f
#define TAU_COULOMB 0.0700f
#define K_TAU_PER_AMP 0.1000f
#define HUMAN_TAU_DEADBAND 0.0350f
#define ASSIST_GAIN 0.90f

// Intent gating / drift rejection.
#define INTENT_START_VEL_ERPM 30.0f
#define INTENT_STOP_VEL_ERPM 12.0f
#define INTENT_START_ACC_RAD_S2 9.0f
#define BIAS_VEL_ERPM 8.0f
#define BIAS_ACC_RAD_S2 2.0f
#define BIAS_ALPHA 0.02f

// Transparent mode compensation.
#define TR_K_VEL 0.015f
#define TR_K_COULOMB 0.040f
#define TR_K_GRAV 0.030f
#define TR_MAX_CURRENT_A 0.20f

// Command shaping / limits.
#define MAX_ASSIST_CURRENT_A 0.55f
#define CMD_SLEW_UP_A_PER_STEP 0.020f
#define CMD_SLEW_DOWN_A_PER_STEP 0.014f
#define CMD_ZERO_CROSS_STEP 0.030f
#define POS_LIMIT_DEG 90.0f

typedef enum {
    MODE_TRANSPARENT = 0,
    MODE_ASSIST = 1,
} control_mode_t;

typedef struct {
    bool enabled;
    control_mode_t mode;

    bool fb_seen;
    float pos_deg;
    float vel_erpm;
    float cur_a;
    int temp_c;
    int err_code;

    float vel_filt_erpm;
    float acc_filt_rad_s2;
    float human_tau_est_nm;
    float tau_bias_nm;
    float cmd_current_a;
    bool intent_active;

    bool zero_ref_set;
    float zero_ref_deg;

    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t rx_any_count;
} ctrl_state_t;

static ctrl_state_t g = {
    .enabled = false,
    .mode = MODE_ASSIST,
};
static const char *TAG = "exo_ctrl";
static portMUX_TYPE g_lock = portMUX_INITIALIZER_UNLOCKED;

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

static float compute_human_tau(ctrl_state_t *s, float *vfilt_out, float *afilt_out, float *tau_bias_out)
{
    float vfilt = s->vel_filt_erpm + VEL_LP_ALPHA * (s->vel_erpm - s->vel_filt_erpm);
    float vel_rad_s = vfilt * ERPM_TO_RAD_S;
    float prev_vel_rad_s = s->vel_filt_erpm * ERPM_TO_RAD_S;
    float acc_raw = (vel_rad_s - prev_vel_rad_s) / (CONTROL_PERIOD_MS * 0.001f);
    float afilt = s->acc_filt_rad_s2 + ACC_LP_ALPHA * (acc_raw - s->acc_filt_rad_s2);
    float tau_motor = K_TAU_PER_AMP * s->cur_a;
    float tau_raw = (J_EQUIV * afilt) + (B_EQUIV * vel_rad_s) + (TAU_COULOMB * signf0(vel_rad_s)) - tau_motor;
    float tau_bias = s->tau_bias_nm;

    if ((fabsf(vfilt) <= BIAS_VEL_ERPM) && (fabsf(afilt) <= BIAS_ACC_RAD_S2) && (fabsf(s->cmd_current_a) <= 0.03f)) {
        tau_bias += BIAS_ALPHA * (tau_raw - tau_bias);
    }

    *vfilt_out = vfilt;
    *afilt_out = afilt;
    *tau_bias_out = tau_bias;
    return tau_raw - tau_bias;
}

static float compute_assist_cmd(ctrl_state_t *s, float human_tau, float vfilt, float afilt)
{
    float abs_v = fabsf(vfilt);
    float abs_a = fabsf(afilt);
    float abs_tau = fabsf(human_tau);

    if (!s->intent_active) {
        if ((abs_v >= INTENT_START_VEL_ERPM) || (abs_a >= INTENT_START_ACC_RAD_S2)) {
            s->intent_active = true;
        }
    } else if ((abs_v <= INTENT_STOP_VEL_ERPM) && (abs_tau <= HUMAN_TAU_DEADBAND)) {
        s->intent_active = false;
    }

    if (!s->intent_active || abs_tau <= HUMAN_TAU_DEADBAND) {
        return 0.0f;
    }

    float tau_assist = ASSIST_GAIN * human_tau;
    float raw_cmd = tau_assist / K_TAU_PER_AMP;
    return clampf(raw_cmd, -MAX_ASSIST_CURRENT_A, MAX_ASSIST_CURRENT_A);
}

static float compute_transparent_cmd(const ctrl_state_t *s)
{
    float rel_deg = s->pos_deg - s->zero_ref_deg;
    float rel_rad = rel_deg * DEG_TO_RAD;
    float vel_rad_s = s->vel_filt_erpm * ERPM_TO_RAD_S;

    float c_visc = TR_K_VEL * vel_rad_s;
    float c_coul = TR_K_COULOMB * signf0(vel_rad_s);
    float c_grav = TR_K_GRAV * sinf(rel_rad);
    float tau_comp = c_visc + c_coul + c_grav;
    float cmd = tau_comp / K_TAU_PER_AMP;
    return clampf(cmd, -TR_MAX_CURRENT_A, TR_MAX_CURRENT_A);
}

static float apply_position_limit(float cmd_a, const ctrl_state_t *s)
{
    float rel_deg = s->pos_deg - s->zero_ref_deg;
    if (rel_deg >= POS_LIMIT_DEG && cmd_a > 0.0f) return 0.0f;
    if (rel_deg <= -POS_LIMIT_DEG && cmd_a < 0.0f) return 0.0f;
    return cmd_a;
}

static void print_status_line(void)
{
    ctrl_state_t s;
    portENTER_CRITICAL(&g_lock);
    s = g;
    portEXIT_CRITICAL(&g_lock);

    const char *mode_str = (s.mode == MODE_ASSIST) ? "assist" : "transparent";
    ESP_LOGI(TAG,
             "STAT on=%d mode=%s intent=%d rel=%.1fdeg vel=%.0ferpm tau=%.3fNm cmd=%.2fA mot=%.2fA err=%d",
             s.enabled, mode_str, s.intent_active, (s.pos_deg - s.zero_ref_deg), s.vel_filt_erpm,
             s.human_tau_est_nm, s.cmd_current_a, s.cur_a, s.err_code);
}

static void print_bus_line(void)
{
    twai_status_info_t tsi = {0};
    twai_get_status_info(&tsi);

    ctrl_state_t s;
    portENTER_CRITICAL(&g_lock);
    s = g;
    portEXIT_CRITICAL(&g_lock);

    ESP_LOGI(TAG, "BUS tx=%lu rx=%lu rx_any=%lu can_state=%d txerr=%lu rxerr=%lu",
             (unsigned long)s.tx_count, (unsigned long)s.rx_count, (unsigned long)s.rx_any_count,
             (int)tsi.state, (unsigned long)tsi.tx_error_counter, (unsigned long)tsi.rx_error_counter);
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
        if ((int)(rx.identifier & 0xFF) != MOTOR_ID) continue;

        int16_t p_i = (int16_t)((rx.data[0] << 8) | rx.data[1]);
        int16_t v_i = (int16_t)((rx.data[2] << 8) | rx.data[3]);
        int16_t c_i = (int16_t)((rx.data[4] << 8) | rx.data[5]);

        portENTER_CRITICAL(&g_lock);
        g.fb_seen = true;
        g.pos_deg = (float)p_i * 0.1f;
        g.vel_erpm = (float)v_i * 10.0f;
        g.cur_a = (float)c_i * 0.01f;
        g.temp_c = rx.data[6];
        g.err_code = rx.data[7];
        g.rx_count++;
        portEXIT_CRITICAL(&g_lock);
    }
}

static void control_task(void *arg)
{
    (void)arg;
    TickType_t last_stat_tick = xTaskGetTickCount();
    TickType_t last_bus_tick = xTaskGetTickCount();

    while (1) {
        ctrl_state_t s;
        portENTER_CRITICAL(&g_lock);
        s = g;
        portEXIT_CRITICAL(&g_lock);

        float cmd_target = 0.0f;
        float vfilt = s.vel_filt_erpm;
        float afilt = s.acc_filt_rad_s2;
        float tau_bias = s.tau_bias_nm;
        float human_tau = s.human_tau_est_nm;

        if (s.fb_seen) {
            if (!s.zero_ref_set) {
                s.zero_ref_set = true;
                s.zero_ref_deg = s.pos_deg;
                ESP_LOGI(TAG, "EVT zero_ref_set=%.2fdeg", s.zero_ref_deg);
            }

            human_tau = compute_human_tau(&s, &vfilt, &afilt, &tau_bias);
            s.vel_filt_erpm = vfilt;
            s.acc_filt_rad_s2 = afilt;
            s.tau_bias_nm = tau_bias;
            s.human_tau_est_nm = human_tau;

            if (!s.enabled) {
                s.intent_active = false;
                cmd_target = 0.0f;
            } else if (s.mode == MODE_ASSIST) {
                cmd_target = compute_assist_cmd(&s, human_tau, vfilt, afilt);
            } else {
                s.intent_active = false;
                cmd_target = compute_transparent_cmd(&s);
            }

            cmd_target = apply_position_limit(cmd_target, &s);
        }

        float cmd_out = smooth_command_update(s.cmd_current_a, cmd_target);
        if (send_servo_current(MOTOR_ID, cmd_out) == ESP_OK) {
            portENTER_CRITICAL(&g_lock);
            g.tx_count++;
            g.vel_filt_erpm = s.vel_filt_erpm;
            g.acc_filt_rad_s2 = s.acc_filt_rad_s2;
            g.human_tau_est_nm = s.human_tau_est_nm;
            g.tau_bias_nm = s.tau_bias_nm;
            g.intent_active = s.intent_active;
            g.zero_ref_deg = s.zero_ref_deg;
            g.zero_ref_set = s.zero_ref_set;
            g.cmd_current_a = cmd_out;
            portEXIT_CRITICAL(&g_lock);
        }

        TickType_t now = xTaskGetTickCount();
        if ((now - last_stat_tick) >= pdMS_TO_TICKS(STATUS_PERIOD_MS)) {
            last_stat_tick = now;
            print_status_line();
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
        ESP_LOGI(TAG, "EVT commands=on|off|mode assist|mode transparent|zero|status|ping");
        return;
    }
    if (strcmp(line, "ping") == 0) {
        ESP_LOGI(TAG, "EVT pong");
        return;
    }
    if (strcmp(line, "on") == 0) {
        portENTER_CRITICAL(&g_lock);
        g.enabled = true;
        portEXIT_CRITICAL(&g_lock);
        ESP_LOGI(TAG, "EVT on");
        return;
    }
    if (strcmp(line, "off") == 0) {
        portENTER_CRITICAL(&g_lock);
        g.enabled = false;
        g.intent_active = false;
        portEXIT_CRITICAL(&g_lock);
        ESP_LOGI(TAG, "EVT off");
        return;
    }
    if (strcmp(line, "mode assist") == 0) {
        portENTER_CRITICAL(&g_lock);
        g.mode = MODE_ASSIST;
        portEXIT_CRITICAL(&g_lock);
        ESP_LOGI(TAG, "EVT mode=assist");
        return;
    }
    if (strcmp(line, "mode transparent") == 0) {
        portENTER_CRITICAL(&g_lock);
        g.mode = MODE_TRANSPARENT;
        g.intent_active = false;
        portEXIT_CRITICAL(&g_lock);
        ESP_LOGI(TAG, "EVT mode=transparent");
        return;
    }
    if (strcmp(line, "zero") == 0) {
        portENTER_CRITICAL(&g_lock);
        g.zero_ref_deg = g.pos_deg;
        g.zero_ref_set = true;
        portEXIT_CRITICAL(&g_lock);
        ESP_LOGI(TAG, "EVT zero=%.2fdeg", g.zero_ref_deg);
        return;
    }
    if (strcmp(line, "status") == 0) {
        print_status_line();
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

    ESP_LOGI(TAG, "EVT ready");
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
    ESP_LOGI(TAG, "EVT boot id=%d", MOTOR_ID);

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