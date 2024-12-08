#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include <drone_data/drone_data/msg/robot_position.h>

#include "util.h"
#include "motor_driver.h"
#include "pid.h"

// NOTE: The robot is assumed to only rotate about the z-axis
// Therefore, all quaternion input data should have x=0, y=0, w and z are nonzero
static double quaternion_to_yaw(double w, double x, double y, double z) {
    return atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

#define WHEEL_BASE 0.5

/*
    Assume a stream of continual packets
    Computation flow:
    * Check if distance is outside of tolerance
    * If it is, pass it through a PID controller
    * Push the motor command to the queue
*/
static void drone_callback(const void *msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    printf("Received: %d\n",  (int)3);

    // Extract linear and angular velocities
    double linear_x = msg->linear.x;
    double angular_z = msg->angular.z;

    // Compute left and right motor speeds
    double left_motor_speed = linear_x - angular_z * (WHEEL_BASE / 2.0);
    double right_motor_speed = linear_x + angular_z * (WHEEL_BASE / 2.0);

    // Normalize motor speeds if necessary
    double max_speed = fmax(fabs(left_motor_speed), fabs(right_motor_speed));
    if (max_speed > 1.0)
    {
        left_motor_speed /= max_speed;
        right_motor_speed /= max_speed;
    }

    // Get current and target positions

    struct motor_command left_motor;
    struct motor_command right_motor;

    if (left_motor_speed > 0) {
        left_motor.dir = FORWARD;
    } else if (left_motor_speed < 0) {
        left_motor.dir = BACKWARD;
    } else {
        left_motor.dir = DIR_STOP;
    }

    left_motor.pwm_ratio = left_motor_speed;

    if (right_motor_speed > 0) {
        right_motor.dir = FORWARD;
    } else if (right_motor_speed < 0) {
        right_motor.dir = BACKWARD;
    } else {
        right_motor.dir = DIR_STOP;
    }

    right_motor.pwm_ratio = right_motor_speed;
    
    push_to_motor_queue((struct queue_data){
        .left = left_motor,
        .right = right_motor
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
	RCCHECK(rclc_node_init_default(&node, "esp32_subscriber_node", "", &support));

    // create subscriber
    rcl_subscription_t subscriber;
    RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"diff_cont/cmd_vel_unstamped"
    ));

    // create message
    geometry_msgs__msg__Twist msg;

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