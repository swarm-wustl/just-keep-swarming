#pragma once

#include "esp_err.h"
#include "motor_driver.h"
#include "command_parser.h"

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY 1000 // Hz

#define LEFT_MOTOR_CHANNEL LEDC_CHANNEL_0
#define RIGHT_MOTOR_CHANNEL LEDC_CHANNEL_1

#define ENA GPIO_NUM_13
#define IN1 GPIO_NUM_12
#define IN2 GPIO_NUM_14
#define ENB GPIO_NUM_14   // TODO
#define IN3 GPIO_NUM_27
#define IN4 GPIO_NUM_26
#define GPIO_BITMASK (1ULL << ENA) | (1ULL << IN1) | (1ULL << IN2) | (1ULL << ENB) | (1ULL << IN3) | (1ULL << IN4)

#define PWM_STOP 0.0

typedef struct {
    motor_direction_t left_dir;
    motor_direction_t right_dir;
    double left_pwm_ratio;
    double right_pwm_ratio;
} differential_drive_motor_command_t;

command_parser_ret_t twist_to_differential_drive(const geometry_msgs__msg__Twist *msgin, differential_drive_motor_command_t *msgout);
motor_driver_ret_t esp32_l293d_differential_drive_init();
motor_driver_ret_t esp32_l293d_differential_drive_handler(differential_drive_motor_command_t *msgin);