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

  // Map map = {
  //     {1, 1, 1, 1, 1, 0, 0, 1, 1, 1},
  //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  //     {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
  // };
  // vector<Cell> robots = {{1, 0}, {1, 7}, {1, 9}};
  // vector<Cell> goals = {{1, 9}, {1, 1}, {1, 0}};
  Map map = {
      {1, 1, 1, 1, 0, 0, 0, 0, 1, 1},  // (stop autowrap)
      {0, 0, 0, 0, 0, 1, 0, 0, 0, 0},  // (stop autowrap)
      {1, 1, 0, 1, 1, 1, 1, 1, 0, 1},  // (stop autowrap)
      {0, 0, 0, 1, 1, 1, 1, 1, 0, 1},  // (stop autowrap)
      {0, 1, 1, 1, 1, 1, 1, 1, 0, 1},  // (stop autowrap)
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // (stop autowrap)
  };
  vector<Cell> robots = {{1, 0}, {0, 4}, {4, 8}};  //, {5, 9}};
  vector<Cell> goals = {{1, 9}, {5, 9}, {1, 0}};   //, {1, 1}};
  // vector<vector<Cell>> net_neighbors = get_net_neighbors(map, robots);
  // Map map = {
  //     {0, 0, 0, 0, 0},  // (stop autowrap)
  //     {0, 0, 0, 0, 0},  // (stop autowrap)
  //     {0, 0, 0, 0, 0},  // (stop autowrap)
  //     {0, 0, 0, 0, 0},  // (stop autowrap)
  //     {0, 0, 0, 0, 0},  // (stop autowrap)
  // };
  // vector<Cell> robots = {{0, 0}, {1, 0}, {2, 0}, {3, 0}, {0, 4}};
  // vector<Cell> goals = {{0, 4}, {1, 4}, {2, 4}, {3, 4}, {0, 0}};
  print_multi_map(map, robots);

  vector<vector<Cell>> plan = sstar(robots, goals, map);
  for (auto &step : plan) {
    for (size_t i = 0; i < step.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "Robot #%d -> (%d, %d)",
                  static_cast<int>(i), step[i].x, step[i].y);
    }
  }

  size_t i = 0;
  while (i < plan.size()) {
    if (goal_handle->is_canceling()) {
      // result-> ...
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    print_multi_map(map, plan[i]);
    RCLCPP_INFO(this->get_logger(), "===");
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
