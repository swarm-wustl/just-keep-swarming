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

#include "receiver.h"

#include <my_custom_message/my_custom_message/msg/my_custom_message.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}

void subscription_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    printf("Received: %d\n",  (int)msg->data);
}

static void basic_task(void *param) {
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
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"esp32_example_subscriber"
    ));

    // create message
    // TODO: this type is deprecated
    std_msgs__msg__Int32 msg;

    // create executor with a single handle
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    // add subscriber to executor
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &subscriber, 
        &msg,
		&subscription_callback, 
        ON_NEW_DATA
    ));

    // run the executor forever (continuously receive messages)
    rclc_executor_spin(&executor);

    /*struct queue_data sending_data;

    sending_data.id = 1;
    sending_data.x = 25;
    sending_data.y = -30;

    while (1) {
        printf("sending task data in 3 seconds...\n");
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        push_to_queue(sending_data);
    }*/
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    xTaskCreate(basic_task, "basic_task", 4*1024, NULL, 1, NULL);
    // xTaskCreate(receiving_task, "receiving_task", 4*1024, NULL, 1, NULL);
}
