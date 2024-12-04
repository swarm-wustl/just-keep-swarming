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

#include "util.h"
#include "motor_driver.h"

// NOTE: The robot is assumed to only rotate about the z-axis
// Therefore, all quaternion input data should have x=0, y=0, w and z are nonzero
static double quaternion_to_yaw(double w, double x, double y, double z) {
    return atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

/*
    Assume a stream of continual packets
    Computation flow:
    * Check if distance is outside of tolerance
    * If it is, pass it through a PID controller
    * Push the motor command to the queue
*/
static void drone_callback(const void *msgin) {
    const drone_data__msg__RobotPosition *msg = (const drone_data__msg__RobotPosition *)msgin;
    printf("Received: %d\n",  (int)msg->current_pos.position.x);

    // Get current and target positions

    struct motor_command left_motor;
    struct motor_command right_motor;

    double x_curr = msg->current_pos.position.x;
    double y_curr = msg->current_pos.position.y;
    double theta_curr = quaternion_to_yaw(
        msg->current_pos.orientation.w, 
        msg->current_pos.orientation.x, 
        msg->current_pos.orientation.y, 
        msg->current_pos.orientation.z
    );

    double x_target = msg->target_pos.position.x;
    double y_target = msg->target_pos.position.y;

    double theta_target = atan2(y_target - y_curr, x_target - x_curr);
    double theta_error = theta_target - theta_curr;

    // printf("theta_target, theta_curr: %f, %f", theta_target, theta_curr);

    // Normalize angle to [-pi, pi]

    if (theta_error > M_PI) {
        theta_error -= 2 * M_PI;
    }

    if (theta_error < -1 * M_PI) {
        theta_error += 2 * M_PI;
    }

    // Turn robot to face target point

    if (fabs(theta_error) > ANGLE_TOLERANCE) {
        // TODO: add Kd, Ki
        double angular_velocity = Kp_angular * theta_error;

        double pwm = fmin(fabs(angular_velocity), 1.0);

        // TODO: make sure motors are running in proper directions
        if (angular_velocity >= 0) {
            left_motor = (struct motor_command){
                .dir = BACKWARD,
                .pwm_ratio = pwm
            };

            right_motor = (struct motor_command){
                .dir = FORWARD,
                .pwm_ratio = pwm
            };
        } else {
            left_motor = (struct motor_command){
                .dir = FORWARD,
                .pwm_ratio = pwm
            };

            right_motor = (struct motor_command){
                .dir = BACKWARD,
                .pwm_ratio = pwm
            };
        }

        push_to_motor_queue((struct queue_data){
            .left = left_motor,
            .right = right_motor
        });

        printf("Rotating to target: Theta Error = %.2f\n", theta_error);

        // Return early so we don't try moving linearly if we still need to fix our angle
        return;
    }

    // Move robot to reach target distance

    double distance_error = sqrt(pow(x_target - x_curr, 2) + pow(y_target - y_curr, 2));

    if (distance_error > DISTANCE_TOLERANCE) {
        // TODO: add Kd, Ki
        double linear_velocity = Kp_linear * distance_error;

        // Set motor commands for forward motion
        left_motor = (struct motor_command){
            .dir = FORWARD,
            .pwm_ratio = fmin(linear_velocity, 1.0)
        };

        right_motor = (struct motor_command){
            .dir = FORWARD,
            .pwm_ratio = fmin(linear_velocity, 1.0)
        };

        push_to_motor_queue((struct queue_data){
            .left = left_motor,
            .right = right_motor
        });

        printf("Moving to target: Distance Error = %.2f\n", distance_error);

        // Return early if still trying to reach target position
        return;
    }

    printf("Target reached!\n");

    left_motor = (struct motor_command){
        .dir = STOP,
        .pwm_ratio = 0.0
    };

    right_motor = (struct motor_command){
        .dir = STOP,
        .pwm_ratio = 0.0
    };
    
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