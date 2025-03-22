// Copyright 2024 Prestin Meek
#pragma once
// // Copyright 2024 Prestin Meek
// #pragma once

#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

#include "control_algorithms/action/pid.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "shared_types/msg/pid_position.hpp"
/*
 * camera node initializes all ESP32s
 * camera node tells PID controller: hey, this robot exists, here's my ID,
 make
 * a topic for me PID controller responds: ok, here's a topic for the robot
 * camera node sends ID back to ESP32
 * ESP32 subscribes to PID topic
 */

/*
* subscribe to camera topic
* callback on new data
* for each robot, run pid
* pid fucking somehow has the target position
* pid sends data to each robot

* TODO: make flowchart
*/

/*

    One thing to note, check if execute runs multipel times

*/

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

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr robot_pub_;

  std::unordered_map<uint32_t, geometry_msgs::msg::Pose> robot_map_;

  std::mutex position_mutex_;

  // could just make this an anon function
  void position_callback_(const shared_types::msg::PidPosition pos_msg_);
};

}  // namespace control_algorithms
