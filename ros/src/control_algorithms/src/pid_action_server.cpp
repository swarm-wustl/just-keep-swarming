#include "control_algorithms/pid_action_server.hpp"

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

}