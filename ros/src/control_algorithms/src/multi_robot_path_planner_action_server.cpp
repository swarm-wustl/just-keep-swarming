// Copyright 2024 Sebastian Theiler, Jaxon Poentis
#include "control_algorithms/multi_robot_path_planner_action_server.hpp"

#include "control_algorithms/algorithms/astar.hpp"
#include "control_algorithms/algorithms/common.hpp"
#include "control_algorithms/algorithms/pplan.hpp"
#include "control_algorithms/algorithms/sstar.hpp"
#include "drone_msg/msg/robot_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
namespace control_algorithms {

// place holder for now, convert eugune code to correct code
double cell_to_real(double x) { return x; }

geometry_msgs::msg::Pose create_pose(double x, double y) {
  geometry_msgs::msg::Pose pose = geometry_msgs::msg::Pose();

  geometry_msgs::msg::Point point = geometry_msgs::msg::Point();
  point.x = x;
  point.y = y;

  pose.position = point;
  return pose;
}

MultiRobotPathPlannerActionServer::MultiRobotPathPlannerActionServer(
    const rclcpp::NodeOptions &options)
    : Node("multi_robot_path_planner_action_server", options) {
  using std::placeholders::_1, std::placeholders::_2;

  this->action_server_ = rclcpp_action::create_server<MultiRobotPathPlan>(
      this, "multi_robot_path_planner",
      std::bind(&MultiRobotPathPlannerActionServer::handle_goal, this, _1, _2),
      std::bind(&MultiRobotPathPlannerActionServer::handle_cancel, this, _1),
      std::bind(&MultiRobotPathPlannerActionServer::handle_accepted, this, _1));

  this->map = nav_msgs::msg::OccupancyGrid();

  this->robot_full_pub = this->create_publisher<drone_msg::msg::RobotPosition>(
      "robot_full_pos", 10);

  this->robot_poses_sub =
      this->create_subscription<geometry_msgs::msg::PoseArray>(
          "filtered_robot_array_pos", 10,
          std::bind(&MultiRobotPathPlannerActionServer::update_poses, this,
                    _1));

  RCLCPP_INFO(this->get_logger(),
              "Multi robot path planner action server initialized");
}

rclcpp_action::GoalResponse MultiRobotPathPlannerActionServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const MultiRobotPathPlan::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;
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

void MultiRobotPathPlannerActionServer::robot_feedback(
    const geometry_msgs::msg::Pose current_pose,
    const geometry_msgs::msg::Pose target_pos, const int id) {
  drone_msg::msg::RobotPosition robot_msg = drone_msg::msg::RobotPosition();

  std_msgs::msg::Header header = std_msgs::msg::Header();

  rclcpp::Time now = this->get_clock()->now();
  header.stamp = now;
  robot_msg.header = header;
  robot_msg.id = id;
  robot_msg.current_pose = current_pose;
  robot_msg.target_pose = target_pos;
  this->robot_full_pub->publish(robot_msg);
}

void MultiRobotPathPlannerActionServer::execute(
    const std::shared_ptr<GoalHandleMultiRobotPathPlan> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<MultiRobotPathPlan::Feedback>();
  auto result = std::make_shared<MultiRobotPathPlan::Result>();

  // Fake starting positions in place of state estimation
  vector<Cell> robots = {
      {0, 0}, {0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5},
      /*{5, 0}, {4, 0}, {3, 0}, {2, 0}, {1, 0},*/
  };

  vector<Cell> goals;
  for (const auto &goal_cell : goal->goal_cells) {
    goals.emplace_back(goal_cell.x, goal_cell.y);
  }

  if (robots.size() != goals.size()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Number of goals does not equal number of robots");
    result->error_code = NUM_GOAL_NUM_ROBOTS_MISMATCH;
    result->error_msg = "Number of goals does not equal number of robots";
    goal_handle->abort(result);
    return;
  }

  // Fake map in place of occupancy grid
  Map map = {
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // .
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // .
      {0, 0, 1, 0, 0, 0, 0, 0, 0, 0},  // .
      {0, 0, 0, 0, 0, 0, 0, 1, 0, 0},  // .
      {0, 0, 0, 0, 1, 0, 0, 0, 0, 0},  // .
      {0, 0, 0, 0, 1, 0, 0, 0, 0, 0},  // .
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // .
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // .
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // .
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // .
  };
  bool changed = obstacle_inflate(&map, 1);
  print_multi_map(map, robots);

  auto start_time = this->now();

  /*vector<vector<Cell>> plan = sstar(robots, goals, map);*/
  vector<vector<Cell>> plan = pplan(robots, goals, map);
  if (plan.empty()) {
    result->error_code = FAILED_TO_PLAN;
    result->error_msg = "Failed to generate a plan";
    goal_handle->abort(result);
  }

  size_t i = 0;
  while (i < plan.size()) {
    if (goal_handle->is_canceling()) {
      result->error_code = CANCELED;
      result->error_msg = "Canceled";
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Planning canceled");
      return;
    }

    feedback->navigation_time = this->now() - start_time;
    feedback->current_poses = {};
    for (Cell &c : plan[i]) {
      control_algorithms::msg::GridCell grid_cell_msg;
      grid_cell_msg.x = c.x;
      grid_cell_msg.y = c.y;

      feedback->current_poses.push_back(grid_cell_msg);
    }

    int num_goals_reached = 0;
    for (size_t n = 0; n < goals.size(); n++) {
      if (plan[i][n] == goals[n]) ++num_goals_reached;
    }

    feedback->num_goals_reached = num_goals_reached;
    goal_handle->publish_feedback(feedback);

    // this tracks how much robots made it to each goal for i-th timestep

    std::unordered_set<int> passed_j = {};

    while (passed_j.size() < robots.size()) {
      for (int j = 0; j < plan[i].size(); ++j) {
        // the robot has already got to its position
        if (passed_j.find(j) != passed_j.end()) continue;

        // LOL
        Cell &c = plan[i][j];

        // // Current
        double cx = robot_poses.poses[j].position.x;
        double cy = robot_poses.poses[j].position.y;

        // (Jaxon)
        // im having this just return the pixel coord so it compiles LOL
        // we're going to need to implement the python code for this.
        // Is c.x going to be the grid x or real pos x, we can make the real pos
        // be the point in the cnter of the grid would be pretty simple to calc
        // that point
        double tx = cell_to_real(c.x);
        double ty = cell_to_real(c.y);

        // if the stuff is close enuf, then mark in the set that robot j has
        // reach its target, which will then be skipped above.
        if (abs(cx - tx) <= this->cut_off_dist &&
            abs(cy - ty) <= this->cut_off_dist) {
          passed_j.insert(j);
        }

        // this is a little questionable, we're publishing the points even tho
        // there could be no update, we should pontially move this into
        // update_poses, and then keep track of the current timestamp with a
        // member variable this should work for now tho
        geometry_msgs::msg::Pose pose = create_pose(tx, ty);

        this->robot_feedback(pose, robot_poses.poses[j], j);
      }

      loop_rate.sleep();
    }
    ++i;
  }

  if (rclcpp::ok() && plan.size()) {
    result->error_code = SUCCESS;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Planning succeeded");
  }
}

// Fixed this here as I was getting errors with this function being const
// updating a member variable
void MultiRobotPathPlannerActionServer::update_poses(
    const geometry_msgs::msg::PoseArray &msg) {
  this->robot_poses = msg;

  // going to have this send the pose here
}

}  // namespace control_algorithms

RCLCPP_COMPONENTS_REGISTER_NODE(
    control_algorithms::MultiRobotPathPlannerActionServer)
