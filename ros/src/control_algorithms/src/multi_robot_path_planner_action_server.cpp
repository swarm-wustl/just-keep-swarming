// Copyright 2024 Sebastian Theiler
#include "control_algorithms/multi_robot_path_planner_action_server.hpp"

#include "control_algorithms/algorithms/astar.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace control_algorithms {

MultiRobotPathPlannerActionServer::MultiRobotPathPlannerActionServer(
    const rclcpp::NodeOptions &options)
    : Node("multi_robot_path_planner_action_server", options) {
  using std::placeholders::_1, std::placeholders::_2;

  this->action_server_ = rclcpp_action::create_server<MultiRobotPathPlan>(
      this, "multi_robot_path_planner",
      std::bind(&MultiRobotPathPlannerActionServer::handle_goal, this, _1, _2),
      std::bind(&MultiRobotPathPlannerActionServer::handle_cancel, this, _1),
      std::bind(&MultiRobotPathPlannerActionServer::handle_accepted, this, _1));

  RCLCPP_INFO(this->get_logger(),
              "Multi robot path planner action server initialized");
}

rclcpp_action::GoalResponse MultiRobotPathPlannerActionServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const MultiRobotPathPlan::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MultiRobotPathPlannerActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleMultiRobotPathPlan> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MultiRobotPathPlannerActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleMultiRobotPathPlan> goal_handle) {
  using std::placeholders::_1;
  std::thread{std::bind(&MultiRobotPathPlannerActionServer::execute, this, _1),
              goal_handle}
      .detach();
}

void MultiRobotPathPlannerActionServer::execute(
    const std::shared_ptr<GoalHandleMultiRobotPathPlan> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<MultiRobotPathPlan::Feedback>();
  auto result = std::make_shared<MultiRobotPathPlan::Result>();

  Cell startCell = {0, 0};
  Cell goalCell = {95, 95};
  vector<vector<int>> map;
  for (int i = 0; i < 100; i++) {
    vector<int> row;
    for (int j = 0; j < 100; j++) {
      /*if (i > 1 && i < 8 && j > 1 && j < 8) {*/
      if (((i << j) * 1103515245 + 12345) % RAND_MAX + 1 > RAND_MAX / 1.3) {
        /*if (j == 100 - i && i > 2 && j && 2) {*/
        row.push_back(1);
      } else {
        row.push_back(0);
      }
    }
    map.push_back(row);
  }
  vector<control_algorithms::Cell> path =
      control_algorithms::astar(startCell, goalCell, map);
  if (path.size() == 0) RCLCPP_INFO(this->get_logger(), "No path found");

  int i = 0;
  while (i < 100) {
    if (goal_handle->is_canceling()) {
      // result-> ...
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    // do thing

    RCLCPP_INFO(this->get_logger(), "Publish feedback");
    goal_handle->publish_feedback(feedback);
    loop_rate.sleep();
    ++i;
  }

  if (rclcpp::ok()) {
    // result->...
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}

}  // namespace control_algorithms

RCLCPP_COMPONENTS_REGISTER_NODE(
    control_algorithms::MultiRobotPathPlannerActionServer)
