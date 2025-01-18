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

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    command_parser_t parser = {
        .node_name = "diffdrive_node",
        .subscriber_name = "diffdrive_subscriber",
        .message_type = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        .handler = (command_parser_handler_t)twist_to_differential_drive
    };

    motor_driver_t motor_driver = {
        .init = (motor_driver_ret_t (*)())esp32_l293d_differential_drive_init,
        .handler = (motor_driver_ret_t (*)(void *))esp32_l293d_differential_drive_handler
    };

    xTaskCreate((TaskFunction_t)command_parser_task, "command_parser_task", 4*1024, &parser, 1, NULL);
    xTaskCreate((TaskFunction_t)motor_driver_task, "motor_driver_task", 4*1024, &motor_driver, 1, NULL);
}