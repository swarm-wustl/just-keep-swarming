#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include "differential_drive.h"
#include "util.h"

#define WHEELBASE 0.5   // dist between wheels (m) (we're gonna fake it)

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY 1000 // Hz

#define LEFT_MOTOR_CHANNEL LEDC_CHANNEL_0
#define RIGHT_MOTOR_CHANNEL LEDC_CHANNEL_1

#define ENA GPIO_NUM_27
#define IN1 GPIO_NUM_14
#define IN2 GPIO_NUM_32
#define ENB GPIO_NUM_26
#define IN3 GPIO_NUM_25
#define IN4 GPIO_NUM_33
#define GPIO_BITMASK (1ULL << ENA) | (1ULL << IN1) | (1ULL << IN2) | (1ULL << ENB) | (1ULL << IN3) | (1ULL << IN4)

#define FLOAT_TOLERANCE 0.01

command_parser_ret_t twist_to_differential_drive(const geometry_msgs__msg__Twist *msgin, differential_drive_motor_command_t *msgout) {
    if (msgin == NULL || msgout == NULL) {
        return COMMAND_PARSER_ERROR_GENERIC;
    }

    // Extract linear and angular velocities
    double linear_velocity = msgin->linear.x;
    double angular_velocity = msgin->angular.z;

    // Compute individual wheel velocities
    double left_velocity = linear_velocity - (angular_velocity * WHEELBASE / 2);
    double right_velocity = linear_velocity + (angular_velocity * WHEELBASE / 2);

    // Find the maximum velocity for normalization
    double max_velocity = fmax(fabs(left_velocity), fabs(right_velocity));
    if (max_velocity > 1.0) {
        left_velocity /= max_velocity;
        right_velocity /= max_velocity;
    }

    // Determine motor directions and set PWM ratios
    if (fabs(left_velocity) <= FLOAT_TOLERANCE) {
        msgout->left_dir = MOTOR_STOP;
    } else {
        msgout->left_dir = (left_velocity >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
    }
    
    if (fabs(right_velocity) <= FLOAT_TOLERANCE) {
        msgout->right_dir = MOTOR_STOP;
    } else {
        msgout->right_dir = (right_velocity >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
    }

    msgout->left_pwm_ratio = fabs(left_velocity);
    msgout->right_pwm_ratio = fabs(right_velocity);

    return COMMAND_PARSER_SUCCESS;
}

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

motor_driver_ret_t esp32_l293d_differential_drive_init() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_BITMASK;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    setup_pwm();

    return MOTOR_DRIVER_SUCCESS;
}

motor_driver_ret_t esp32_l293d_differential_drive_handler(differential_drive_motor_command_t *msgin) {
    if (msgin == NULL) {
        return MOTOR_DRIVER_ERROR_GENERIC;
    }

    printf("hola: %d %d %f %f\n", msgin->left_dir, msgin->right_dir, msgin->left_pwm_ratio, msgin->right_pwm_ratio);

    // Control motors based on table:
    // https://lastminuteengineers.com/l293d-dc-motor-arduino-tutorial/

    // Control left motor
    if (msgin->left_dir == MOTOR_STOP) {
        gpio_set_level(IN1, LOW);
        gpio_set_level(IN2, LOW);
    } else if (msgin->left_dir == MOTOR_FORWARD) {
        gpio_set_level(IN1, HIGH);
        gpio_set_level(IN2, LOW);
    } else if (msgin->left_dir == MOTOR_BACKWARD) {
        gpio_set_level(IN1, LOW);
        gpio_set_level(IN2, HIGH);
    }

    // Control right motor
    if (msgin->right_dir == MOTOR_STOP) {
        gpio_set_level(IN3, LOW);
        gpio_set_level(IN4, LOW);
    } else if (msgin->right_dir == MOTOR_FORWARD) {
        gpio_set_level(IN3, HIGH);
        gpio_set_level(IN4, LOW);
    } else if (msgin->right_dir == MOTOR_BACKWARD) {
        gpio_set_level(IN3, LOW);
        gpio_set_level(IN4, HIGH);
    }

    // Update left motor PWM
    double left_duty_cycle = compute_duty_cycle(msgin->left_pwm_ratio);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEFT_MOTOR_CHANNEL, left_duty_cycle));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEFT_MOTOR_CHANNEL));

    // Update right motor PWM
    double right_duty_cycle = compute_duty_cycle(msgin->right_pwm_ratio);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, RIGHT_MOTOR_CHANNEL, right_duty_cycle));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, RIGHT_MOTOR_CHANNEL));

    printf("ran motors successfully!\n");

    return MOTOR_DRIVER_SUCCESS;
}
