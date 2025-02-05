// Copyright 2024 Sebastian Theiler
#pragma once

#include <memory>
#include <set>
#include <vector>

#include "control_algorithms/action/multi_robot_path_plan.hpp"
#include "drone_msg/msg/robot_position.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
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

  void robot_feedback(const geometry_msgs::msg::Pose current_pose,
                      const geometry_msgs::msg::Pose target_pos, const int id);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr og_map_sub;

  void update_map(const nav_msgs::msg::OccupancyGrid &map_msg);

  nav_msgs::msg::OccupancyGrid current_map;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr
      robot_poses_sub;

  geometry_msgs::msg::PoseArray robot_poses;
  // changed this function to not be const to allow member variables to be
  // updated, revert if wrong (Jaxon)
  void update_poses(const geometry_msgs::msg::PoseArray &msg);

  rclcpp::Publisher<drone_msg::msg::RobotPosition>::SharedPtr robot_full_pub;

  const double cut_off_dist = 0.05;
};

}  // namespace control_algorithms
