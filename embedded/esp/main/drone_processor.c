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

#include <drone_data/drone_data/msg/robot_position.h>

#include "util.h"
#include "motor_driver.h"

static void drone_callback(const void *msgin) {
    const drone_data__msg__RobotPosition *msg = (const drone_data__msg__RobotPosition *)msgin;
    printf("Received: %d\n",  (int)msg->current_pos.position.x);

    // TODO: data processing

    push_to_queue((struct queue_data){
        .left = (struct motor_command){
            .dir = FORWARD,
            .pwm_ratio = 0.75
        },
        .right = (struct motor_command){
            .dir = FORWARD,
            .pwm_ratio = 0.75
        }
    });
}

void drone_task(void *param) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "esp32_example_node", "", &support));

    // create subscriber
    rcl_subscription_t subscriber;
    RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(drone_data, msg, RobotPosition),
		"esp32_example_subscriber"
    ));

    // create message
    drone_data__msg__RobotPosition msg;

    // create executor with a single handle
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    // add subscriber to executor
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &subscriber, 
        &msg,
		&drone_callback, 
        ON_NEW_DATA
    ));

    // run the executor forever (continuously receive messages)
    rclc_executor_spin(&executor);
}