// Copyright 2024 Sebastian Theiler
#pragma once

#include <memory>

#include "control_algorithms/action/assemble.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace control_algorithms {

class AssemblerActionClient : public rclcpp::Node {
 public:
  using Assemble = control_algorithms::action::Assemble;
  using GoalHandleAssemble = rclcpp_action::ClientGoalHandle<Assemble>;

  explicit AssemblerActionClient(const rclcpp::NodeOptions &options);

  void send_goal();

 private:
  rclcpp_action::Client<Assemble>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandleAssemble::SharedPtr &goal_handle);
  void feedback_callback(
      GoalHandleAssemble::SharedPtr,
      const std::shared_ptr<const Assemble::Feedback> feedback);
  void result_callback(const GoalHandleAssemble::WrappedResult &result);
};

}  // namespace control_algorithms
