// Copyright 2024 Sebastian Theiler
#pragma once

#include <memory>

#include "control_algorithms/action/multi_robot_path_plan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace control_algorithms {

class MRPPActionClient : public rclcpp::Node {
 public:
  using MultiRobotPathPlan = control_algorithms::action::MultiRobotPathPlan;
  using GoalHandleMultiRobotPathPlan =
      rclcpp_action::ClientGoalHandle<MultiRobotPathPlan>;

  explicit MRPPActionClient(const rclcpp::NodeOptions &options);

  void send_goal();

 private:
  rclcpp_action::Client<MultiRobotPathPlan>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(
      const GoalHandleMultiRobotPathPlan::SharedPtr &goal_handle);
  void feedback_callback(
      GoalHandleMultiRobotPathPlan::SharedPtr,
      const std::shared_ptr<const MultiRobotPathPlan::Feedback> feedback);
  void result_callback(
      const GoalHandleMultiRobotPathPlan::WrappedResult &result);
};

}  // namespace control_algorithms
