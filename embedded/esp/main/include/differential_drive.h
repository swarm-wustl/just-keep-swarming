#pragma once

#include "esp_err.h"
#include "motor_driver.h"
#include "command_parser.h"

typedef struct {
    motor_direction_t left_dir;
    motor_direction_t right_dir;
    double left_pwm_ratio;
    double right_pwm_ratio;
} differential_drive_motor_command_t;

command_parser_ret_t twist_to_differential_drive(const geometry_msgs__msg__Twist *msgin, differential_drive_motor_command_t *msgout);
motor_driver_ret_t esp32_l293d_differential_drive_init();
motor_driver_ret_t esp32_l293d_differential_drive_handler(differential_drive_motor_command_t *msgin);