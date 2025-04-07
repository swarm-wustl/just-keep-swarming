// Copyright 2024 Alston Liu, Jaxon Poentis
#include "control_algorithms/test/test_PID.hpp"

std::vector<double> proximity_to_goal(geometry_msgs::msg::Pose cur,
                                      geometry_msgs::msg::Pose tar) {
  return {cur.position.x - tar.position.x, cur.position.y - tar.position.y,
          cur.position.z - tar.position.z};
}

namespace control_algorithms {
TestPID_AC::TestPID_AC(const rclcpp::NodeOptions &options)
    : Node("PID_AS_TESTER", options) {
  using std::placeholders::_1, std::placeholders::_2;

  // GENERATE TEST BOTS HERE

  this->declare_parameter<double>("x_pos", 2.0);

  double pos_x = this->get_parameter("x_pos").as_double();

  this->declare_parameter<double>("y_pos", 2.0);

  double pos_y = this->get_parameter("y_pos").as_double();

  this->declare_parameter<int>("rob_id", 0);

  int rob_id = this->get_parameter("rob_id").as_int();

  // ROBOT 0 TEST
  geometry_msgs::msg::Pose test_robot_0;

  test_robot_0.position.set__x(0.0);
  test_robot_0.position.set__y(0.0);
  test_robot_0.position.set__z(0.0);

  this->test_robots[rob_id] = test_robot_0;

  geometry_msgs::msg::Pose test_robot_target_0;

  test_robot_target_0.position.set__x(pos_x);
  test_robot_target_0.position.set__y(pos_y);
  test_robot_target_0.position.set__z(0.0);

  this->test_robots_targets[rob_id] = test_robot_target_0;

  this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/model/robot_0/pose", 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        auto rob_pose = this->test_robots_targets[0].position;

        this->test_robots[0].position.set__x(rob_pose.x + msg->twist.linear.x);
        this->test_robots[0].position.set__y(rob_pose.y + msg->twist.linear.y);
        this->test_robots[0].position.set__z(rob_pose.z + msg->twist.linear.z);
      });
  auto goal_msg = control_algorithms::action::PID::Goal();

  auto target_pose = geometry_msgs::msg::Pose();

  auto send_goal_options =
      rclcpp_action::Client<control_algorithms::action::PID>::SendGoalOptions();

  goal_msg.target_pose = test_robot_target_0;

  goal_msg.robot_id = rob_id;
  // should try make this bitch use a loop to generate these tests and queue

  send_goal_options.goal_response_callback =
      std::bind(&TestPID_AC::goal_response_callback, this, _1);
  send_goal_options.result_callback =
      std::bind(&TestPID_AC::result_callback, this, _1);

  auto client_ptr_ =
      rclcpp_action::create_client<control_algorithms::action::PID>(this,
                                                                    "pid");

  // this is of type std::shared_future<PIDGoalHandle::SharedPtr>, auto used
  // cuz implied lol
  auto future_goal = client_ptr_->async_send_goal(goal_msg, send_goal_options);

  RCLCPP_INFO(this->get_logger(), "Goal sent!");
  // must verify ans
}

// ned to do make a feedback tester
void TestPID_AC::goal_response_callback(
    const GoalHandlePID::SharedPtr &goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

void TestPID_AC::result_callback(const GoalHandlePID::WrappedResult &result) {
  RCLCPP_INFO(this->get_logger(), "Goal finsished, printing results:");

  std::vector<double> prox =
      proximity_to_goal(this->test_robots[0], this->test_robots_targets[0]);

  std::stringstream ans;

  ans << "prox: x: " << prox[0] << "y: " << prox[1] << "z: " << prox[2];

  RCLCPP_INFO(this->get_logger(), ans.str().c_str());

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }

  std::stringstream ss;
  ss << "Result received: " << result.result->error_code;

  RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  rclcpp::shutdown();
}

}  // namespace control_algorithms

RCLCPP_COMPONENTS_REGISTER_NODE(control_algorithms::TestPID_AC)
