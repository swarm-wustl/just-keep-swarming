#include "motor_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "util.h"

static QueueHandle_t xQueue = NULL;
static TickType_t delay = MOTOR_QUEUE_DELAY;
static struct queue_data data;

static void setup_pwm() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel_left_motor = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,  // LEDC low-speed mode
        .channel        = LEFT_MOTOR_CHANNEL,   // Set motor channel
        .timer_sel      = LEDC_TIMER_0,         // Use timer 0
        .intr_type      = LEDC_INTR_DISABLE,    // Disable interrupts
        .gpio_num       = ENA,                  // Assign GPIO pin
        .duty           = 0,                    // Initial duty cycle (0%)
        .hpoint         = 0                     // Default hpoint value
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_left_motor));

    ledc_channel_config_t ledc_channel_right_motor = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,  // LEDC low-speed mode
        .channel        = RIGHT_MOTOR_CHANNEL,  // Set motor channel
        .timer_sel      = LEDC_TIMER_0,         // Use timer 0
        .intr_type      = LEDC_INTR_DISABLE,    // Disable interrupts
        .gpio_num       = ENB,                  // Assign GPIO pin
        .duty           = 0,                    // Initial duty cycle (0%)
        .hpoint         = 0                     // Default hpoint value
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_right_motor));
}

static double compute_duty_cycle(float pwm_ratio) {
    // [0, 1] --> [0, (2 ^ resolution) - 1]
    return pwm_ratio * ((1 << LEDC_DUTY_RES) - 1);
}

void push_to_motor_queue(struct queue_data d) {
    // TODO: error handle
    if (xQueue != NULL) {
        xQueueSendToBack(xQueue, (void *)&d, delay);
    }
}

void motor_task(void *param) {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_BITMASK;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    setup_pwm();

    xQueue = xQueueCreate(MOTOR_QUEUE_SIZE, sizeof(struct queue_data));

    while (1) {
        if (xQueue != NULL) {
            if (xQueueReceive(xQueue, &data, delay) == pdPASS) {
                printf("received %d!\n", data.left.dir);

                // Control motors based on table:
                // https://lastminuteengineers.com/l293d-dc-motor-arduino-tutorial/

                // Control left motor
                enum direction left_dir = data.left.dir;
                if (left_dir == DIR_STOP) {
                    gpio_set_level(IN1, LOW);
                    gpio_set_level(IN2, LOW);
                } else if (left_dir == FORWARD) {
                    gpio_set_level(IN1, HIGH);
                    gpio_set_level(IN2, LOW);
                } else if (left_dir == BACKWARD) {
                    gpio_set_level(IN1, LOW);
                    gpio_set_level(IN2, HIGH);
                }

                // Control right motor
                enum direction right_dir = data.right.dir;
                if (right_dir == DIR_STOP) {
                    gpio_set_level(IN3, LOW);
                    gpio_set_level(IN4, LOW);
                } else if (left_dir == FORWARD) {
                    gpio_set_level(IN3, HIGH);
                    gpio_set_level(IN4, LOW);
                } else if (left_dir == BACKWARD) {
                    gpio_set_level(IN3, LOW);
                    gpio_set_level(IN4, HIGH);
                }

                // Update left motor PWM
                double left_duty_cycle = compute_duty_cycle(data.left.pwm_ratio);
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEFT_MOTOR_CHANNEL, left_duty_cycle));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEFT_MOTOR_CHANNEL));

                // Update right motor PWM
                double right_duty_cycle = compute_duty_cycle(data.right.pwm_ratio);
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, RIGHT_MOTOR_CHANNEL, right_duty_cycle));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, RIGHT_MOTOR_CHANNEL));

                // printf("pwm left, right: %f, %f", data.left.pwm_ratio, data.right.pwm_ratio);
                // printf("duty left, right: %f, %f", left_duty_cycle, right_duty_cycle);
            }
        }
    }
}
