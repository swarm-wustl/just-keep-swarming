// Copyright 2024 Sebastian Theiler
#pragma once

#include <memory>

#include "control_algorithms/action/assemble.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace control_algorithms {

class AssemblerActionServer : public rclcpp::Node {
 public:
  using Assemble = control_algorithms::action::Assemble;
  using GoalHandleAssemble = rclcpp_action::ServerGoalHandle<Assemble>;

  explicit AssemblerActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  rclcpp_action::Server<Assemble>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const Assemble::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleAssemble> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleAssemble> goal_handle);
    
  void execute(const std::shared_ptr<GoalHandleAssemble> goal_handle);

  void id_request(const std::shared_ptr<shared_types::srv::IdRequest::Request> request,
          std::shared_ptr<shared_types::srv::IdRequest::Response> response);

  int counter = 0;

};

}  // namespace control_algorithms
