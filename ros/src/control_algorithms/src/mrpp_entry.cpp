// Copyright 2025 Sebastian Theiler
#include "control_algorithms/multi_robot_path_planner_action_server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<control_algorithms::MultiRobotPathPlannerActionServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    4); // e.g., 4 threads
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
