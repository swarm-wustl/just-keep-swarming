// Copyright 2024 Alston Liu, Jaxon Poenti
#pragma once

#include <unordered_map>

#include "control_algorithms/action/pid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "shared_types/msg/pid_position.hpp"
#include "shared_types/srv/position_list.hpp"

namespace control_algorithms {
class TestPID_AC : public rclcpp::Node {
 public:
  using PID = control_algorithms::action::PID;
  using GoalHandlePID = rclcpp_action::ClientGoalHandle<PID>;
  explicit TestPID_AC(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  std::unordered_map<uint16_t, geometry_msgs::msg::Pose> test_robots;
  std::unordered_map<uint16_t, geometry_msgs::msg::Pose> test_robots_targets;

  void goal_response_callback(const GoalHandlePID::SharedPtr &goal_handle);

  void result_callback(const GoalHandlePID::WrappedResult &result);
};
}  // namespace control_algorithms
