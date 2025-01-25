// Copyright 2024 Sebastian Theiler
#include "control_algorithms/demo_assembler_client.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace control_algorithms {

AssemblerActionClient::AssemblerActionClient(const rclcpp::NodeOptions &options)
    : Node("mrpp_action_client", options) {
  this->client_ptr_ = rclcpp_action::create_client<Assemble>(this, "assembler");

  this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&AssemblerActionClient::send_goal, this));

  RCLCPP_INFO(this->get_logger(), "Initialized client");
}

void AssemblerActionClient::send_goal() {
  using std::placeholders::_1, std::placeholders::_2;

  this->timer_->cancel();

  RCLCPP_INFO(this->get_logger(), "Connecting to action server");
  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available");
    rclcpp::shutdown();
  }

  auto goal_msg = Assemble::Goal();
  // TODO(sebtheiler): populate goal msg

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<Assemble>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&AssemblerActionClient::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&AssemblerActionClient::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&AssemblerActionClient::result_callback, this, _1);

  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void AssemblerActionClient::goal_response_callback(
    const GoalHandleAssemble::SharedPtr &goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal rejected");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted");
  }
}

void AssemblerActionClient::feedback_callback(
    GoalHandleAssemble::SharedPtr,
    const std::shared_ptr<const Assemble::Feedback> feedback) {
  RCLCPP_INFO(this->get_logger(), "Received feedback");
}

void AssemblerActionClient::result_callback(
    const GoalHandleAssemble::WrappedResult &result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }

  RCLCPP_INFO(this->get_logger(), "Printing result");
  rclcpp::shutdown();
}

}  // namespace control_algorithms

RCLCPP_COMPONENTS_REGISTER_NODE(control_algorithms::AssemblerActionClient)
