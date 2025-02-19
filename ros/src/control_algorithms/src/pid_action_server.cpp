#include "control_algorithms/pid_action_server.hpp"

// TODO: test Kd
// TODO: add Ki
// TODO: add reset functionality when going to a very new point (need to reset prev time, prev error, etc.)

#define DISTANCE_TOLERANCE 0.05 // meters
#define ANGLE_TOLERANCE 0.05    // rad

// Angular constsants
#define Kp_angular 2.0
#define Kd_angular 0.5

// Linear constants
#define Kp_linear 1.0
#define Kd_linear 0.3

using builtin_interfaces::msg::Time;

static Time angular_time_prev;
static double angular_error_prev = 0.0;

static Time linear_time_prev;
static double linear_error_prev = 0.0;

#define SEC_TO_NSEC 1000000000.0 

static double delta_time(Time final, Time initial) {
    int32_t sec = final.sec - initial.sec;
    int32_t nsec = final.nanosec - initial.nanosec;

    if (nsec < 0) {
        sec -= 1;
        nsec += SEC_TO_NSEC;
    }

    return sec + (nsec / SEC_TO_NSEC);
}

// TODO: think about the design of passing in current time but using global prev time... kinda weird

static double angular_error_to_velocity(double error, Time time_curr) { 
    double derivative;
    double time_diff = delta_time(time_curr, angular_time_prev);

    // If this is the first packet, don't include a derivative term
    // Otherwise, calculate the derivative based on current and previous values
    if (angular_time_prev.sec == 0 || time_curr == angular_time_prev) {
        derivative = 0;
    } else {
        derivative = (error - angular_error_prev) / time_diff;
    }

    angular_time_prev = time_curr;
    angular_error_prev = error;

    return (Kp_angular * error) + (Kd_angular * derivative);
}

static double linear_error_to_velocity(double error, Time time_curr) {
    double derivative;
    double time_diff = delta_time(time_curr, linear_time_prev);

    // If this is the first packet, don't include a derivative term
    // Otherwise, calculate the derivative based on current and previous values
    if (linear_time_prev.sec == 0 || time_curr == linear_time_prev) {
        derivative = 0;
    } else {
        derivative = (error - linear_error_prev) / time_diff;
    }

    linear_time_prev = time_curr;
    linear_error_prev = error;

    return (Kp_linear * error) + (Kd_linear * derivative);
}

// NOTE: The robot is assumed to only rotate about the z-axis
// Therefore, all quaternion input data should have x=0, y=0, w and z are nonzero
static double quaternion_to_yaw(geometry_msgs::msg::Quaternion pos) {
    return atan2(2.0 * (pos.w * pos.z + pos.x * pos.y), 1.0 - 2.0 * (pos.y * pos.y + pos.z * pos.z));
}

namespace control_algorithms {

PIDActionServer::PIDActionServer(const rclcpp::NodeOptions &options)
        : Node("pid_action_server", options) {
    
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<PID>(
      this, "pid",
      std::bind(&PIDActionServer::handle_goal, this, _1, _2),
      std::bind(&PIDActionServer::handle_cancel, this, _1),
      std::bind(&PIDActionServer::handle_accepted, this, _1)
    );

    angular_time_prev.sec = -1;
    linear_time_prev.sec = -1;

    RCLCPP_INFO(this->get_logger(), "PID action server initialized");
}

rclcpp_action::GoalResponse PIDActionServer::handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const PID::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PIDActionServer::handle_cancel(
        const std::shared_ptr<GoalHandlePID> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PIDActionServer::handle_accepted(
        const std::shared_ptr<GoalHandlePID> goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&PIDActionServer::execute, this, _1), goal_handle}.detach();
}

void PIDActionServer::execute(const std::shared_ptr<GoalHandlePID> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<PID::Feedback>();
    auto result = std::make_shared<PID::Result>();

    geometry_msgs::msg::Pose current_pose;  // TODO
    const geometry_msgs::msg::Pose &target_pose = goal->target_pose;

    Time current_time = goal->header.stamp;

    double current_x = current_pose.position.x;
    double current_y = current_pose.position.y;
    double current_theta = quaternion_to_yaw(current_pose.orientation);

    double target_x = target_pose.position.x;
    double target_y = target_pose.position.y;
    double target_theta = atan2(target_y - current_y, target_x - current_x);

    double theta_error = target_theta - current_theta;

    // Normalize angle to [-pi, pi]

    if (theta_error > M_PI) {
        theta_error -= 2 * M_PI;
    }

    if (theta_error < -1 * M_PI) {
        theta_error += 2 * M_PI;
    }

    // Turn robot to face target point

    if (fabs(theta_error) > ANGLE_TOLERANCE) {
        double angular_velocity = angular_error_to_velocity(theta_error, current_time);

        // TODO: do something
        printf("Rotating to target: Theta Error = %.2f\n", theta_error);

        // Return early so we don't try moving linearly if we still need to fix our angle
        return;
    }

    // If angle is within tolerance, move robot to reach target distance

    double distance_error = sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));

    if (distance_error > DISTANCE_TOLERANCE) {
        double linear_velocity = linear_error_to_velocity(distance_error, current_time);

        // TODO: do something
        printf("Moving to target: Distance Error = %.2f\n", distance_error);

        // Return early if still trying to reach target position
        return;
    }

    printf("Target reached!\n");
}

}
