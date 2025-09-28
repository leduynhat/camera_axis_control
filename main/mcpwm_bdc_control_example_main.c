#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/mcpwm_prelude.h"
#include "driver/uart.h"
#include "esp_log.h"

#define UART_PORT       UART_NUM_0
#define UART_RX_BUF     256
#define SERVO_MIN_US    500
#define SERVO_MAX_US    2500
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

static const char *TAG = "PT_CTRL";

static float pan_angle = 90.0f;   // current servo states
static float tilt_angle = 90.0f;
static float target_pan = 90.0f;  // target states
static float target_tilt = 90.0f;

static mcpwm_cmpr_handle_t pan_cmp;
static mcpwm_cmpr_handle_t tilt_cmp;

// PID gains (start simple with P control)
static float Kp = 0.1f;   // degrees per pixel
static float Ki = 0.0f;
static float Kd = 0.0f;

static float err_sum_pan = 0, err_sum_tilt = 0;
static float last_err_pan = 0, last_err_tilt = 0;

// ---- helper: convert angle to PWM pulse ----
static inline uint32_t angle_to_us(float angle) {
    float ratio = (angle - SERVO_MIN_ANGLE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);
    return (uint32_t)(SERVO_MIN_US + ratio * (SERVO_MAX_US - SERVO_MIN_US));
}

// ---- update PWM ----
static void servo_update(mcpwm_cmpr_handle_t cmp, float angle) {
    uint32_t pulse = angle_to_us(angle);
    mcpwm_comparator_set_compare_value(cmp, pulse);
}

// ---- RX Task ----
static void uart_rx_task(void *arg) {
    uint8_t data[UART_RX_BUF];
    while (1) {
        int len = uart_read_bytes(UART_PORT, data, sizeof(data)-1, pdMS_TO_TICKS(100));
        if (len > 0) {
            data[len] = 0;
            char *msg = (char*)data;
            int dx, dy;
            if (sscanf(msg, "%d,%d", &dx, &dy) == 2) {
                // ---- PID control ----
                float error_pan  = -dx; // dx > 0 → need to turn left (negative)
                float error_tilt = -dy; // dy > 0 → need to tilt up (negative)

                err_sum_pan  += error_pan;
                err_sum_tilt += error_tilt;

                float d_pan  = error_pan  - last_err_pan;
                float d_tilt = error_tilt - last_err_tilt;

                float control_pan  = Kp * error_pan  + Ki * err_sum_pan  + Kd * d_pan;
                float control_tilt = Kp * error_tilt + Ki * err_sum_tilt + Kd * d_tilt;

                last_err_pan  = error_pan;
                last_err_tilt = error_tilt;

                // map control output to servo target
                target_pan  = pan_angle + control_pan;
                target_tilt = tilt_angle + control_tilt;

                // ---- clamp ----
                if (target_pan < 0) target_pan = 0;
                if (target_pan > 180) target_pan = 180;
                if (target_tilt < 45) target_tilt = 45;
                if (target_tilt > 135) target_tilt = 135;

                ESP_LOGI(TAG, "ERR dx=%d dy=%d -> target_pan=%.1f target_tilt=%.1f",
                         dx, dy, target_pan, target_tilt);
            }
        }
    }
}

// ---- Control Task (servo smoothing) ----
static void control_task(void *arg) {
    float v = 15.0f;  // deg/sec (slew rate limit)
    float dt = 0.05f; // 50 ms
    while (1) {
        float dx = target_pan - pan_angle;
        float dy = target_tilt - tilt_angle;
        float dist = sqrtf(dx*dx + dy*dy);

		if (dist < 5.0f)
		{
			v = 5;
		}
        if (dist > 2.0f) {
            float step = v * dt;
            if (step > dist) step = dist;
            pan_angle  += step * dx / dist;
            tilt_angle += step * dy / dist;
            servo_update(pan_cmp, pan_angle);
            servo_update(tilt_cmp, tilt_angle);
            ESP_LOGI(TAG, "Move -> pan=%.1f tilt=%.1f", pan_angle, tilt_angle);
        }
        vTaskDelay(pdMS_TO_TICKS((int)(dt*1000)));
    }
}

// ---- Setup ----
void app_main(void) {
    // UART setup
    const uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_cfg);

    // MCPWM setup
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t tcfg = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000, // 1MHz, 1us
        .period_ticks = 20000,    // 20ms
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    mcpwm_new_timer(&tcfg, &timer);

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t ocfg = {.group_id = 0};
    mcpwm_new_operator(&ocfg, &oper);
    mcpwm_operator_connect_timer(oper, timer);

    mcpwm_comparator_config_t ccfg = {.flags.update_cmp_on_tez = true};
    mcpwm_generator_config_t gcfg = {.gen_gpio_num = 18}; // pan on GPIO18
    mcpwm_new_comparator(oper, &ccfg, &pan_cmp);
    mcpwm_gen_handle_t pan_gen;
    mcpwm_new_generator(oper, &gcfg, &pan_gen);
    mcpwm_generator_set_action_on_timer_event(pan_gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(pan_gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, pan_cmp, MCPWM_GEN_ACTION_LOW));

    // second comparator for tilt
    mcpwm_comparator_config_t ccfg2 = {.flags.update_cmp_on_tez = true};
    mcpwm_generator_config_t gcfg2 = {.gen_gpio_num = 19}; // tilt on GPIO19
    mcpwm_new_comparator(oper, &ccfg2, &tilt_cmp);
    mcpwm_gen_handle_t tilt_gen;
    mcpwm_new_generator(oper, &gcfg2, &tilt_gen);
    mcpwm_generator_set_action_on_timer_event(tilt_gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(tilt_gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, tilt_cmp, MCPWM_GEN_ACTION_LOW));

    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

    // set initial positions
    servo_update(pan_cmp, pan_angle);
    servo_update(tilt_cmp, tilt_angle);

    // start tasks
    xTaskCreate(uart_rx_task, "uart_rx", 4096, NULL, 4, NULL);
    xTaskCreate(control_task, "ctrl", 4096, NULL, 5, NULL);
}
