// Copyright 2024 Sebastian Theiler
#pragma once

#include <memory>
#include <vector>

#include "control_algorithms/action/multi_robot_path_plan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace control_algorithms {

enum MultiRobotPathPlannerActionServerErrorCode {
  SUCCESS = 0,
  NUM_GOAL_NUM_ROBOTS_MISMATCH = 1,
  FAILED_TO_PLAN = 2,
  CANCELED = 3
};

class MultiRobotPathPlannerActionServer : public rclcpp::Node {
 public:
  using MultiRobotPathPlan = control_algorithms::action::MultiRobotPathPlan;
  using GoalHandleMultiRobotPathPlan =
      rclcpp_action::ServerGoalHandle<MultiRobotPathPlan>;

  explicit MultiRobotPathPlannerActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  rclcpp_action::Server<MultiRobotPathPlan>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const MultiRobotPathPlan::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleMultiRobotPathPlan> goal_handle);

  void handle_accepted(
      const std::shared_ptr<GoalHandleMultiRobotPathPlan> goal_handle);

  void execute(const std::shared_ptr<GoalHandleMultiRobotPathPlan> goal_handle);

  std::vector<int> map;
};

}  // namespace control_algorithms
