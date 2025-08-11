// Copyright 2024 Sebastian Theiler, Jaxon Poentis
#include "control_algorithms/multi_robot_path_planner_action_server.hpp"

#include "control_algorithms/action/pid.hpp"
#include "control_algorithms/algorithms/astar.hpp"
#include "control_algorithms/algorithms/common.hpp"
#include "control_algorithms/algorithms/pplan.hpp"
#include "control_algorithms/algorithms/sstar.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "shared_types/msg/robot_position.hpp"
#include <stdexcept>

namespace control_algorithms {

Cell pose_to_cell(const geometry_msgs::msg::Pose &pose,
                  const nav_msgs::msg::OccupancyGrid &og_map) {
  if (og_map.info.resolution <= 0) {
    throw std::invalid_argument("Map resolution must be positive");
  }

  // Transform pose to map's coordinate frame
  float x = pose.position.x - og_map.info.origin.position.x;
  float y = pose.position.y - og_map.info.origin.position.y;

  std::cout << "x" << x << std::endl;
  std::cout << "y" << y << std::endl;
  std::cout << "res" << og_map.info.resolution << std::endl;

  // Convert to cell indices
  int cell_x = static_cast<int>(std::floor(x / og_map.info.resolution)) +
               og_map.info.width / 2;
  int cell_y = static_cast<int>(std::floor(y / og_map.info.resolution)) +
               og_map.info.height / 2;

  // Check if the cell is within map bounds
  if (cell_x < 0 || cell_y < 0 ||
      cell_x >= static_cast<int>(og_map.info.width) ||
      cell_y >= static_cast<int>(og_map.info.height)) {
    std::cout << "cx" << cell_x << std::endl;
    std::cout << "cy" << cell_y << std::endl;
    throw std::out_of_range("Pose is outside the map bounds");
  }

  return {static_cast<size_t>(cell_x), static_cast<size_t>(cell_y)};
}
/*Cell pose_to_cell(geometry_msgs::msg::Pose pose,*/
/*                  nav_msgs::msg::OccupancyGrid const &og_map) {*/
/*  float x = pose.position.x;*/
/*  float y = pose.position.y;*/
/**/
/*  int cell_x =*/
/*      static_cast<int>(((x + static_cast<float>((og_map.info.width / 2.0)) /*/
/*                                 og_map.info.resolution)));*/
/*  int cell_y =*/
/*      static_cast<int>(((y + static_cast<float>((og_map.info.height / 2.0))
 * /*/
/*                                 og_map.info.resolution)));*/
/**/
/*  return {static_cast<size_t>(cell_x), static_cast<size_t>(cell_y)};*/
/*}*/

Map create_map(nav_msgs::msg::OccupancyGrid const &og_map) {
  Map map = {};

  vector<int> row = {};

  int width = og_map.info.width;
  int height = og_map.info.height;

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; ++j) {
      row.push_back(og_map.data.at(i * width + j));
    }
    map.push_back(row);
    row.clear();
  }
  return map;
}

// place holder for now, convert eugune code to correct code
// accepts one dimension, convert to real dimension
double x_cell_to_real(const double x,
                      nav_msgs::msg::OccupancyGrid const &og_map) {
  // plus resolution makes the goal the center of the square
  return x * og_map.info.resolution - og_map.info.width / 2 +
         og_map.info.resolution / 2;
}
double y_cell_to_real(const double y,
                      nav_msgs::msg::OccupancyGrid const &og_map) {
  return y * og_map.info.resolution - og_map.info.height / 2 +
         og_map.info.resolution / 2;
}

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

  // generating default map
  this->declare_parameter<int>("width", 1000);
  this->declare_parameter<int>("height", 1000);
  this->declare_parameter<double>("res", 0.05);

  // Get parameters
  int map_width = this->get_parameter("width").as_int();
  int map_height = this->get_parameter("height").as_int();
  double res = this->get_parameter("res").as_double();

  const std::vector<int8_t> map_data(map_height * map_width, 0);

  this->current_map.set__data(map_data);
  this->current_map.info.set__height(map_height);
  this->current_map.info.set__width(map_width);
  this->current_map.info.set__resolution(res);

  this->action_server_ = rclcpp_action::create_server<MultiRobotPathPlan>(
      this, "multi_robot_path_planner",
      std::bind(&MultiRobotPathPlannerActionServer::handle_goal, this, _1, _2),
      std::bind(&MultiRobotPathPlannerActionServer::handle_cancel, this, _1),
      std::bind(&MultiRobotPathPlannerActionServer::handle_accepted, this, _1));

  // may need a service call to get an initial map
  this->og_map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "og_map", 10,
      std::bind(&MultiRobotPathPlannerActionServer::update_map, this, _1));

  this->robot_full_pub =
      this->create_publisher<shared_types::msg::RobotPosition>("robot_full_pos",
                                                               10);

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

void MultiRobotPathPlannerActionServer::update_map(
    const nav_msgs::msg::OccupancyGrid &map_msg) {
  this->current_map = map_msg;
}

void MultiRobotPathPlannerActionServer::robot_feedback(
    const geometry_msgs::msg::Pose current_pose,
    const geometry_msgs::msg::Pose target_pos, const int id) {
  shared_types::msg::RobotPosition robot_msg =
      shared_types::msg::RobotPosition();

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

  // stores the most recent map available, avoids the map changing mid run due
  // to sub call_back
  nav_msgs::msg::OccupancyGrid used_map = this->current_map;

  Map map = create_map(used_map);

  // Gets the initial robot positions based on what's current being reported as
  // the positions
  vector<Cell> robots = {};

  // should I have the robots positions always being updated like this?
  // what if we just make a service call, that just gets the position ONLY for
  // path planning

  // fuck it im making a service call

  // std::shared_ptr<rclcpp::Node> node =
  // rclcpp::Node::make_shared("add_two_ints_client");
  //

  rclcpp::Client<shared_types::srv::PositionList>::SharedPtr client =
      this->create_client<shared_types::srv::PositionList>("get_full_robo_pos");

  if (!client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Service 'get_full_robo_pos' not available.");
    result->error_code = FAILED_TO_PLAN; // Use appropriate error codes
    result->error_msg = "Service not available";
    goal_handle->abort(result); // Abort the goal
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Making request");
  auto robo_pos_request =
      std::make_shared<shared_types::srv::PositionList::Request>();
  auto robot_pos_result_future = client->async_send_request(robo_pos_request);

  RCLCPP_INFO(this->get_logger(), "Waiting for service response...");

  // *** CHANGE HERE: Wait directly on the future ***
  // The MultiThreadedExecutor in main will handle the underlying processing
  // needed for the future to complete.
  std::future_status status = robot_pos_result_future.wait_for(
      std::chrono::seconds(5)); // Wait for 5 seconds

  if (status != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(),
                 "Service call to 'get_full_robo_pos' timed out or failed.");
    result->error_code = FAILED_TO_PLAN;
    result->error_msg = "FAILED_SERVICE_CALL_TIMEOUT_OR_ERROR";
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Received service response");

  // Get the result (this might throw if the service call itself failed
  // internally)
  try {
    auto response = robot_pos_result_future.get();

    // Optional: Check the response content if the service definition includes
    // success/failure flags if (!response->success /* or similar check based on
    // your srv definition */) {
    //   RCLCPP_ERROR(this->get_logger(), "Robot position service call indicated
    //   failure."); result->error_code = FAILED_TO_PLAN; result->error_msg =
    //   "FAILED_SERVICE_CALL_LOGIC"; goal_handle->abort(result); return;
    // }

    this->robot_poses.set__poses(response->poses.poses);
    for (const geometry_msgs::msg::Pose robot_pose : this->robot_poses.poses) {
      std::cout << "x" << robot_pose.position.x << std::endl;
      std::cout << "y" << robot_pose.position.y << std::endl;
      auto cell = pose_to_cell(robot_pose, used_map);
      std::cout << "cx" << cell.x << std::endl;
      std::cout << "cy" << cell.y << std::endl;
      robots.push_back(pose_to_cell(robot_pose, used_map));
    }
    RCLCPP_INFO(this->get_logger(), "Updated poses");

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Exception while getting service result: %s", e.what());
    result->error_code = FAILED_TO_PLAN;
    result->error_msg = "FAILED_SERVICE_CALL_EXCEPTION";
    goal_handle->abort(result);
    return;
  }

  /*rclcpp::Client<shared_types::srv::PositionList>::SharedPtr client =*/
  /*    this->create_client<shared_types::srv::PositionList>("get_full_robo_pos");*/
  /**/
  /*// Add checks for service availability*/
  /*if (!client->wait_for_service(std::chrono::seconds(2))) { // Add timeout*/
  /*  RCLCPP_ERROR(this->get_logger(),*/
  /*               "Service 'get_full_robo_pos' not available.");*/
  /*  // ... handle failure (abort goal) ...*/
  /*  return;*/
  /*}*/
  /**/
  /*RCLCPP_INFO(this->get_logger(), "Making request");*/
  /*auto robo_pos_request =*/
  /*    std::make_shared<shared_types::srv::PositionList::Request>();*/
  /*auto robot_pos_result_future =
   * client->async_send_request(robo_pos_request);*/
  /**/
  /*// Wait for the result using the future's wait_for or get method*/
  /*// The MultiThreadedExecutor in main will process the response*/
  /*RCLCPP_INFO(this->get_logger(), "Waiting");*/
  /*auto spin_status = rclcpp::spin_until_future_complete(*/
  /*    this->get_node_base_interface(), robot_pos_result_future,*/
  /*    std::chrono::seconds(5)); // Add a timeout*/
  /**/
  /*RCLCPP_INFO(this->get_logger(), "Received");*/
  /*if (spin_status != rclcpp::FutureReturnCode::SUCCESS) {*/
  /*  RCLCPP_ERROR(this->get_logger(),*/
  /*               "Failed to get robot positions service call (Spin Status:
   * %d)",*/
  /*               static_cast<int>(spin_status));*/
  /*  result->error_code = FAILED_TO_PLAN;*/
  /*  result->error_msg = "FAILED_SERVICE_CALL";*/
  /*  goal_handle->abort(result); // Use abort for internal errors*/
  /*  return;*/
  /*}*/
  /**/
  /*// Check the result status from the service itself if applicable*/
  /*auto response = robot_pos_result_future.get();*/

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
  // Map map = {
  //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // .
  //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // .
  //     {0, 0, 1, 0, 0, 0, 0, 0, 0, 0},  // .
  //     {0, 0, 0, 0, 0, 0, 0, 1, 0, 0},  // .
  //     {0, 0, 0, 0, 1, 0, 0, 0, 0, 0},  // .
  //     {0, 0, 0, 0, 1, 0, 0, 0, 0, 0},  // .
  //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // .
  //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // .
  //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // .
  //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // .
  // };

  // bool changed = obstacle_inflate(&map, 1);
  obstacle_inflate(&map, 1);
  /*print_multi_map(map, robots);*/

  auto start_time = this->now();

  /*vector<vector<Cell>> plan = sstar(robots, goals, map);*/
  vector<vector<Cell>> plan = pplan(robots, goals, map);
  if (plan.empty()) {
    result->error_code = FAILED_TO_PLAN;
    result->error_msg = "Failed to generate a plan";
    goal_handle->abort(result);
  }
  // i here is the current time step in the plan
  size_t i = 0;
  while (i < plan.size()) {
    if (goal_handle->is_canceling()) {
      result->error_code = CANCELED;
      result->error_msg = "Canceled";
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Planning canceled");
      return;
    }

    /*

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
    */
    // this tracks how much robots made it to each goal for i-th timestep

    /*

      Later: possibily create paths to run in parrallel, or just concurrently

    */

    using PIDGoalHandle =
        rclcpp_action::ClientGoalHandle<control_algorithms::action::PID>;

    std::vector<std::shared_future<PIDGoalHandle::SharedPtr>> future_queue_;

    uint16_t robo_id = 0;

    for (Cell target_goal : plan[i]) {
      // create goal
      auto send_goal_options = rclcpp_action::Client<
          control_algorithms::action::PID>::SendGoalOptions();

      auto goal_msg = control_algorithms::action::PID::Goal();

      auto target_pose = geometry_msgs::msg::Pose();

      double tx = x_cell_to_real(target_goal.x, used_map);
      double ty = y_cell_to_real(target_goal.y, used_map);

      target_pose.position.x = tx;
      target_pose.position.y = ty;

      goal_msg.target_pose = target_pose;

      goal_msg.robot_id = robo_id;

      auto client_ptr_ =
          rclcpp_action::create_client<control_algorithms::action::PID>(this,
                                                                        "pid");

      // this is of type std::shared_future<PIDGoalHandle::SharedPtr>, auto
      // used cuz implied lol
      auto future_goal =
          client_ptr_->async_send_goal(goal_msg, send_goal_options);

      future_queue_.push_back(future_goal);
      robo_id++;
    }

    // GOOD LUCK LOL
    rclcpp::Rate rate(10); // 10 Hz loop
    while (!future_queue_.empty()) {
      future_queue_.erase(
          // erease the future from the queue if its ready, meaning its
          // finished as progress -> ready when done
          std::remove_if(
              future_queue_.begin(), future_queue_.end(),

              [this](std::shared_future<PIDGoalHandle::SharedPtr> &fut) {
                if (fut.wait_for(std::chrono::seconds(0)) ==
                    std::future_status::ready) {
                  RCLCPP_INFO(this->get_logger(),
                              "a robot reached a goal, removing future");
                  return true;
                }
                return false;
              }),
          future_queue_.end());

      // idk wtf this is doing
      /*if (rclcpp::ok()) {*/
      /*  rclcpp::spin_some(this->get_node_base_interface());*/
      /*}*/
      rate.sleep();
    }

    // success! move forward

    ++i;
  }

  if (rclcpp::ok() && plan.size()) {
    result->error_code = SUCCESS;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Planning succeeded");
  }
}

// void MultiRobotPathPlannerActionServer::create_new_goal (
//   unsigned robo_id,
//   Cell target,
//   vector<control_algorithms::PIDActionServer> &current_servers ) {
//     rclcpp::NodeOption options(); //default options
//     control_algorithms::PIDActionServer PID_controller(options, robo_id);

//     //RCLCPP_COMPONENTS_REGISTER_NODE()

// }

// Fixed this here as I was getting errors with this function being const
// updating a member variable
void MultiRobotPathPlannerActionServer::update_poses(
    const geometry_msgs::msg::PoseArray &msg) {
  this->robot_poses = msg;

  // going to have this send the pose here
}

} // namespace control_algorithms

RCLCPP_COMPONENTS_REGISTER_NODE(
    control_algorithms::MultiRobotPathPlannerActionServer)
