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

#define WHEELBASE 0.5   // dist between wheels (m)

command_parser_ret_t twist_to_differential_drive(geometry_msgs__msg__Twist *msgin, differential_drive_motor_command_t *msgout) {
    if (msgin == NULL || msgout == NULL) {
        return MOTOR_CONVERTER_ERROR_GENERIC;
    }

    // Extract linear and angular velocities
    double linear_velocity = msgin->linear.x;
    double angular_velocity = msgin->angular.z;

    // Compute individual wheel velocities
    double left_velocity = linear_velocity - (angular_velocity * wheelbase / 2);
    double right_velocity = linear_velocity + (angular_velocity * wheelbase / 2);

    // Find the maximum velocity for normalization
    double max_velocity = fmax(fabs(left_velocity), fabs(right_velocity));
    if (max_velocity > 1.0) {
        left_velocity /= max_velocity;
        right_velocity /= max_velocity;
    }

    // Determine motor directions and set PWM ratios
    msgout->left_dir = (left_velocity >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
    msgout->right_dir = (right_velocity >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;

    msgout->left_pwm_ratio = fabs(left_velocity);
    msgout->right_pwm_ratio = fabs(right_velocity);

    // TODO: push_to_motor_driver_queue(msgout);

    return MOTOR_CONVERTER_SUCCESS;
}
