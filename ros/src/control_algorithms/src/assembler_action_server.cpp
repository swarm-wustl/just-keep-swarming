// Copyright 2024 Sebastian Theiler
#include "control_algorithms/assembler_action_server.hpp"

#include <string>

#include "control_algorithms/algorithms/assemble_graph.hpp"
#include "control_algorithms/algorithms/common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace control_algorithms {

AssemblerActionServer::AssemblerActionServer(const rclcpp::NodeOptions &options)
    : Node("assembler_action_server", options) {
  using std::placeholders::_1, std::placeholders::_2;

  this->action_server_ = rclcpp_action::create_server<Assemble>(
      this, "assembler",
      std::bind(&AssemblerActionServer::handle_goal, this, _1, _2),
      std::bind(&AssemblerActionServer::handle_cancel, this, _1),
      std::bind(&AssemblerActionServer::handle_accepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "Assembler action server initialized");
}

rclcpp_action::GoalResponse AssemblerActionServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Assemble::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  if (goal->goal_topology == "MOBILE_MANIPULATOR" ||
      goal->goal_topology == "WALKER")
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  else
    return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse AssemblerActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleAssemble> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void AssemblerActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleAssemble> goal_handle) {
  using std::placeholders::_1;
  std::thread{std::bind(&AssemblerActionServer::execute, this, _1), goal_handle}
      .detach();
}

void AssemblerActionServer::execute(
    const std::shared_ptr<GoalHandleAssemble> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Assemble::Feedback>();
  auto result = std::make_shared<Assemble::Result>();

  std::string goal_topology = goal->goal_topology;
  int num_robots = 11;  // TODO(sebtheiler): get this from some global state

  StructureType topology;
  if (goal_topology == "MOBILE_MANIPULATOR") {
    topology = MOBILE_MANIPULATOR;
  } else if (goal_topology == "WALKER") {
    topology = WALKER;
  } else {
    result->error_code = 1;
    result->error_msg = "Invalid topology";
    goal_handle->abort(result);
  }

  AdjMatrix adj_matrix = load_structure(topology, num_robots);
  Cell root = {10, 10};
  int root_n = 0;
  int radius = 2;
  vector<Cell> unfolded = unfold_graph(adj_matrix, root, root_n, radius);

  size_t i = 0;
  while (i < 42) {
    if (goal_handle->is_canceling()) {
      // result-> ...
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

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

RCLCPP_COMPONENTS_REGISTER_NODE(control_algorithms::AssemblerActionServer)
