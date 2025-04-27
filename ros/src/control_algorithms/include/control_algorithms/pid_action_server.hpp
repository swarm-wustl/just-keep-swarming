// Copyright 2024 Prestin Meek
#pragma once

#include <memory>
#include <set>
#include <vector>

#include "control_algorithms/action/pid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "shared_types/msg/pid_position.hpp"

namespace control_algorithms {

enum result_code { SUCCEED, CANCELED, TIMED_OUT, ERROR };

class PIDActionServer : public rclcpp::Node {
 public:
  using PID = control_algorithms::action::PID;
  using GoalHandlePID = rclcpp_action::ServerGoalHandle<PID>;

  explicit PIDActionServer(const rclcpp::NodeOptions &options);

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const PID::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandlePID> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandlePID> goal_handle);

  void execute(const std::shared_ptr<GoalHandlePID> goal_handle);

 private:
  rclcpp_action::Server<PID>::SharedPtr action_server_;
  bool is_sim;
  int debug_lvl;

  std::mutex subscribers_mutex_;
  std::vector<
      std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>>>
      active_subscribers_;

  // Add this method to safely remove subscribers:
  void remove_subscriber(
      std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>>
          sub) {
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    auto it =
        std::find(active_subscribers_.begin(), active_subscribers_.end(), sub);
    if (it != active_subscribers_.end()) {
      active_subscribers_.erase(it);
    }
  }
};

}  // namespace control_algorithms
