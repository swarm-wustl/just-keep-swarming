#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include "motor_driver.h"
#include "differential_drive.h"
#include "command_parser.h"
#include "util.h"

// TODO: refactor this to not use any of the malloc bs, just use a simple header file API

static command_parser_t parser;
static motor_driver_t motor_driver;

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    parser.node_name = "diffdrive_node";
    parser.topic_name = "diffdrive_twist_2";
    parser.message_type = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);
    parser.handler = (command_parser_handler_t)twist_to_differential_drive;

    parser.msgin = malloc(sizeof(geometry_msgs__msg__Twist));
    parser.msgout = malloc(sizeof(differential_drive_motor_command_t));

    motor_driver.init = (motor_driver_ret_t (*)())esp32_l293d_differential_drive_init;
    motor_driver.handler = (motor_driver_ret_t (*)(void *))esp32_l293d_differential_drive_handler;

    motor_driver.msgin = malloc(sizeof(differential_drive_motor_command_t));

    xTaskCreate((TaskFunction_t)command_parser_task, "command_parser_task", 4*1024, &parser, 1, NULL);
    xTaskCreate((TaskFunction_t)motor_driver_task, "motor_driver_task", 4*1024, &motor_driver, 1, NULL);
}

// ros2 topic pub /diffdrive_twist geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}'
// source /opt/ros/${ROS_DISTRO}/setup.bash
