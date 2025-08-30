/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "driver/uart.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include <string.h>
#include "freertos/timers.h"
#include "driver/uart.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"

// Enable this config,  we will print debug formated string, which in return can be captured and parsed by Serial-Studio
#define SERIAL_STUDIO_DEBUG           CONFIG_SERIAL_STUDIO_DEBUG

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC_MCPWM_GPIO_A              2
#define BDC_MCPWM_GPIO_B              4

#define BDC_ENCODER_GPIO_A            5
#define BDC_ENCODER_GPIO_B            18
#define BDC_ENCODER_PCNT_HIGH_LIMIT   140
#define BDC_ENCODER_PCNT_LOW_LIMIT    -140

#define BDC_PID_LOOP_PERIOD_MS        10   // calculate the motor speed every 10ms
#define BDC_PID_EXPECT_SPEED          20  // expected motor speed, in the pulses counted by the rotary encoder
#define UART_PORT 					  UART_NUM_0
#define RX_BUF_SIZE 					128
#define QUEUE_SIZE 						256
#define MSG_HEADER_1					 0xAA
#define MSG_HEADER_2 				0x55
#define MSG_PREFIX_PID 				0x01
#define MAX_MSG_LENGTH 				32

#define UART_NUM            UART_NUM_0
#define BUF_SIZE            1024
#define RD_BUF_SIZE         BUF_SIZE
#define UART_QUEUE_SIZE     20

#define HEADER_BYTE1        0xAA
#define HEADER_BYTE2        0x55
#define MAX_DATA_LEN        64
#define TIMEOUT_MS          500

static const char *TAG_UART = "uart_pid";
static const char *TAG = "MTC";

static QueueHandle_t uart0_queue = NULL;
static TimerHandle_t uart_timeout_timer = NULL;

// PID values
float pid_kp = 0.6, pid_ki = 0.1, pid_kd = 0.2;

typedef enum {
    WAIT_HEADER_1,
    WAIT_HEADER_2,
    WAIT_OPCODE,
    WAIT_LENGTH,
    WAIT_DATA,
    WAIT_CHECKSUM_1,
    WAIT_CHECKSUM_2
} uart_parse_state_t;


typedef struct {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int report_pulses;
} motor_control_context_t;

static uart_parse_state_t state = WAIT_HEADER_1;

// Parser state machine variables
static uint8_t opcode = 0;
static uint8_t length = 0;
static uint8_t data_buf[MAX_DATA_LEN];
static uint8_t data_index = 0;
static uint16_t checksum_calc = 0;
static uint8_t checksum_lo = 0;
static uint8_t checksum_hi = 0;


// Checksum: simple 16-bit additive
uint16_t calc_checksum(const uint8_t *data, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum += data[i];
    }
    return sum;
}

// PID update logic
void update_pid(const uint8_t *data) {
    memcpy(&pid_kp, data, 4);
    memcpy(&pid_ki, data + 4, 4);
    memcpy(&pid_kd, data + 8, 4);
    ESP_LOGI(TAG_UART, "PID Updated: Kp=%.3f, Ki=%.3f, Kd=%.3f", pid_kp, pid_ki, pid_kd);
}

// Timer callback: called when timeout (no data for TIMEOUT_MS)
void uart_timeout_callback(TimerHandle_t xTimer) {
    ESP_LOGW(TAG_UART, "UART timeout - resetting state machine");
    state = WAIT_HEADER_1;
}

// Reset timeout timer
void uart_reset_timeout() {
    xTimerStop(uart_timeout_timer, 0);
    xTimerChangePeriod(uart_timeout_timer, pdMS_TO_TICKS(TIMEOUT_MS), 0);
    xTimerStart(uart_timeout_timer, 0);
}

// Build feedback packet (opcode 0x02)
static void send_speed_feedback(float speed) {
    uint8_t msg[64];
    int idx = 0;
    msg[idx++] = 0xAA;
    msg[idx++] = 0x55;
    msg[idx++] = 0x02;   // opcode
    msg[idx++] = 4;      // length
    memcpy(&msg[idx], &speed, 4);
    idx += 4;

    uint16_t checksum = 0;
    for (int i = 0; i < idx; i++) checksum += msg[i];
    msg[idx++] = checksum & 0xFF;
    msg[idx++] = (checksum >> 8) & 0xFF;

    uart_write_bytes(UART_NUM, (const char *)msg, idx);
}

// UART byte-by-byte parser
void uart_parse_byte(uint8_t byte) {
    uart_reset_timeout();

    switch (state) {
        case WAIT_HEADER_1:
            if (byte == HEADER_BYTE1) {
                state = WAIT_HEADER_2;
            }
            break;

        case WAIT_HEADER_2:
            if (byte == HEADER_BYTE2) {
                state = WAIT_OPCODE;
                checksum_calc = 0;
            } else {
                state = WAIT_HEADER_1;
            }
            break;

        case WAIT_OPCODE:
            opcode = byte;
            checksum_calc += byte;
            state = WAIT_LENGTH;
            break;

        case WAIT_LENGTH:
            length = byte;
            checksum_calc += byte;
            if (length > MAX_DATA_LEN) {
                ESP_LOGW(TAG_UART, "Invalid payload length: %d", length);
                state = WAIT_HEADER_1;
            } else {
                data_index = 0;
                state = WAIT_DATA;
            }
            break;

        case WAIT_DATA:
            data_buf[data_index++] = byte;
            checksum_calc += byte;
            if (data_index >= length) {
                state = WAIT_CHECKSUM_1;
            }
            break;

        case WAIT_CHECKSUM_1:
            checksum_lo = byte;
            state = WAIT_CHECKSUM_2;
            break;

        case WAIT_CHECKSUM_2:
            checksum_hi = byte;
            {
                uint16_t received_checksum = ((uint16_t)checksum_hi << 8) | checksum_lo;
                if (received_checksum == checksum_calc) {
                    if (opcode == 0x01 && length == 12) {
                        update_pid(data_buf);
                    } else {
                        ESP_LOGW(TAG_UART, "Unknown opcode or invalid length");
                    }
                } else {
                    ESP_LOGW(TAG_UART, "Checksum mismatch! Expected: 0x%04X, Got: 0x%04X", checksum_calc, received_checksum);
                }
            }
            state = WAIT_HEADER_1;
            break;
    }
}

// UART event task
static void uart_event_task(void *pvParameters) {
    uart_event_t event;
    uint8_t *dtmp = (uint8_t *)malloc(RD_BUF_SIZE);
    if (!dtmp) {
        ESP_LOGE(TAG_UART, "UART buffer allocation failed");
        vTaskDelete(NULL);
        return; 
    }

    for (;;) {
        if (xQueueReceive(uart0_queue, (void *)&event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    uart_read_bytes(UART_NUM, dtmp, event.size, portMAX_DELAY);
                    for (int i = 0; i < event.size; ++i) {
                        uart_parse_byte(dtmp[i]);
                    }
                    break;

                case UART_FIFO_OVF:
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG_UART, "UART overflow or full");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart0_queue);
                    break;

                case UART_PARITY_ERR:
                case UART_FRAME_ERR:
                    ESP_LOGW(TAG_UART, "UART error");
                    break;

                default:
                    ESP_LOGI(TAG_UART, "Unhandled UART event type: %d", event.type);
                    break;
            }
        }
    }

    free(dtmp);
    vTaskDelete(NULL);
}

static void pid_loop_cb(void *args)
{
    static int last_pulse_count = 0;
    motor_control_context_t *ctx = (motor_control_context_t *)args;
    pcnt_unit_handle_t pcnt_unit = ctx->pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl = ctx->pid_ctrl;
    bdc_motor_handle_t motor = ctx->motor;

    // get the result from rotary encoder
    int cur_pulse_count = 0;
    pcnt_unit_get_count(pcnt_unit, &cur_pulse_count);
    int real_pulses = cur_pulse_count - last_pulse_count;
    last_pulse_count = cur_pulse_count;
    ctx->report_pulses = real_pulses;

    // calculate the speed error
    float error = BDC_PID_EXPECT_SPEED - real_pulses;
    float new_speed = 0;

    // set the new speed
    pid_compute(pid_ctrl, error, &new_speed);
    bdc_motor_set_speed(motor, (uint32_t)new_speed);
}

void app_main(void)
{
    esp_log_level_set(TAG_UART, ESP_LOG_INFO);

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Create FreeRTOS timer
    uart_timeout_timer = xTimerCreate(
        "UART Timeout",
        pdMS_TO_TICKS(TIMEOUT_MS),
        pdFALSE,
        NULL,
        uart_timeout_callback
    );

    if (uart_timeout_timer == NULL) {
        ESP_LOGE(TAG_UART, "Failed to create FreeRTOS timer");
        return;
    }

    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, UART_QUEUE_SIZE, &uart0_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    
    static motor_control_context_t motor_ctrl_ctx = {
        .pcnt_encoder = NULL,
    };

    ESP_LOGI(TAG, "Create DC motor");
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_A,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_B,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    motor_ctrl_ctx.motor = motor;

    ESP_LOGI(TAG, "Init pcnt driver to decode rotary signal");
    pcnt_unit_config_t unit_config = {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true, // enable counter accumulation
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = BDC_ENCODER_GPIO_A,
        .level_gpio_num = BDC_ENCODER_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = BDC_ENCODER_GPIO_B,
        .level_gpio_num = BDC_ENCODER_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
    motor_ctrl_ctx.pcnt_encoder = pcnt_unit;

    ESP_LOGI(TAG, "Create PID control block");
    pid_ctrl_parameter_t pid_runtime_param = {
        .kp = pid_kp,
        .ki = pid_ki,
        .kd = pid_kd,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = BDC_MCPWM_DUTY_TICK_MAX - 1,
        .min_output   = 0,
        .max_integral = 1000,
        .min_integral = -1000,
    };
    pid_ctrl_block_handle_t pid_ctrl = NULL;
    pid_ctrl_config_t pid_config = {
        .init_param = pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl));
    motor_ctrl_ctx.pid_ctrl = pid_ctrl;

    ESP_LOGI(TAG, "Create a timer to do PID calculation periodically");
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = pid_loop_cb,
        .arg = &motor_ctrl_ctx,
        .name = "pid_loop"
    };
    esp_timer_handle_t pid_loop_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &pid_loop_timer));

    ESP_LOGI(TAG, "Enable motor");
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    ESP_LOGI(TAG, "Forward motor");
    ESP_ERROR_CHECK(bdc_motor_forward(motor));

    ESP_LOGI(TAG, "Start motor speed loop");
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer, BDC_PID_LOOP_PERIOD_MS * 1000));

	// Start UART event task
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
        // the following logging format is according to the requirement of serial-studio frame format
        // also see the dashboard config file `serial-studio-dashboard.json` for more information
		/* Continiously update the PID values */

		pid_runtime_param.kp = pid_kp;
		pid_runtime_param.ki = pid_ki;
		pid_runtime_param.kd = pid_kd;

		pid_update_parameters(motor_ctrl_ctx.pid_ctrl, &pid_runtime_param);
       	// /* printf("/*%d*/\r\n", motor_ctrl_ctx.report_pulses); */
		send_speed_feedback(motor_ctrl_ctx.report_pulses);
    }
}
