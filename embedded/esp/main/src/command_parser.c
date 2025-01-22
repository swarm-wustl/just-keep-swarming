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

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include <drone_data/drone_data/msg/robot_position.h>

#include "command_parser.h"
#include "motor_driver.h"
#include "util.h"

static command_parser_handler_t handler;

static void callback(const void *msgin) {
    if (handler == NULL) {
        return;
    }

    void *msgout = malloc(256);
    command_parser_ret_t res = handler(msgin, msgout);

    printf("received msg!\n");

    if (res != COMMAND_PARSER_SUCCESS) {
        // TODO: error handler
        return;
    }

    printf("parsed msg!\n");

    push_to_motor_driver_queue(msgout);
}

void command_parser_task(command_parser_t *parser) {
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
	RCCHECK(rclc_node_init_default(&node, parser->node_name, "", &support));

    // create subscriber
    rcl_subscription_t subscriber;
    RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		parser->message_type,
		parser->topic_name
    ));

    // store the parser handler so that it can be used in the callback
    handler = parser->handler;

    // create message
    void *msg = malloc(256);

    // create executor with a single handle
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    // add subscriber to executor
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &subscriber, 
        msg,
		&callback, 
        ON_NEW_DATA
    ));

    // run the executor forever (continuously receive messages)
    rclc_executor_spin(&executor);

    // clean up memory (if task ever reaches this point)
    free(parser);
    free(msg);
    vTaskDelete(NULL);
}