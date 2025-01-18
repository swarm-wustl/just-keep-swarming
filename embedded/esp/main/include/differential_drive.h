#pragma once

#include "motor_converter.h"
#include "motor_driver.h"

typedef struct {
    motor_direction_t left_dir;
    motor_direction_t right_dir;
    double left_pwm_ratio;
    double right_pwm_ratio;
} differential_drive_motor_command_t;

// of type command_parser_func_t
command_parser_ret_t twist_to_differential_drive(geometry_msgs__msg__Twist *msgin, differential_drive_motor_command_t *msgout);