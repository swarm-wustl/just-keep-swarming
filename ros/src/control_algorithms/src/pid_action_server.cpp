// Copyright 2025 Preston Meek, Jaxon Poentis, Sebastian Theiler
#include "control_algorithms/pid_action_server.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#define DISTANCE_TOLERANCE 0.10  // meters
#define ANGLE_TOLERANCE 0.2      // rad
#define Kp_angular 0.8
#define Kd_angular 0.0
#define Kp_linear 0.5
#define Kd_linear 0.0
#define GOAL_TIMEOUT 360.0

namespace control_algorithms {

struct PIDState {
  int angular_time_prev = -1;
  double angular_error_prev = 0.0;
  int linear_time_prev = -1;
  double linear_error_prev = 0.0;
};

static geometry_msgs::msg::TwistStamped create_twist(
    double lin_x, double rot_z, builtin_interfaces::msg::Time time_) {
  geometry_msgs::msg::Twist twist_;
  twist_.linear.x = lin_x;
  twist_.angular.z = rot_z;
  geometry_msgs::msg::TwistStamped twist_stamped_;
  twist_stamped_.header.stamp = time_;
  twist_stamped_.twist = twist_;
  return twist_stamped_;
}

/*static double delta_time(builtin_interfaces::msg::Time final,*/
/*                         builtin_interfaces::msg::Time initial) {*/
/*  const double NSEC_TO_SEC = 1000000000.0;*/
/*  int sec = final.sec - initial.sec;*/
/*  int nsec = final.nanosec - initial.nanosec;*/
/*  if (nsec < 0) {*/
/*    sec -= 1;*/
/*    nsec += NSEC_TO_SEC;*/
/*  }*/
/*  return sec + (nsec / NSEC_TO_SEC);*/
/*}*/

static double angular_error_to_velocity(double error,
                                        builtin_interfaces::msg::Time time,
                                        PIDState *state) {
  int time_curr = time.sec;
  double derivative;
  if (state->angular_time_prev == -1 || time_curr == state->angular_time_prev) {
    derivative = 0;
  } else {
    derivative = (error - state->angular_error_prev) /
                 (time_curr - state->angular_time_prev);
  }

  state->angular_time_prev = time_curr;
  state->angular_error_prev = error;

  return (Kp_angular * error) + (Kd_angular * derivative) * 0.75;
}

static double linear_error_to_velocity(double error,
                                       builtin_interfaces::msg::Time time,
                                       PIDState *state) {
  int time_curr = time.sec;
  double derivative;
  if (state->linear_time_prev == -1 || time_curr == state->linear_time_prev) {
    derivative = 0;
  } else {
    derivative = (error - state->linear_error_prev) /
                 (time_curr - state->linear_time_prev);
  }

  state->linear_time_prev = time_curr;
  state->linear_error_prev = error;

  return (Kp_linear * error) + (Kd_linear * derivative) * 0.75;
}

static double quaternion_to_yaw(geometry_msgs::msg::Quaternion pos) {
  double siny_cosp = 2 * (pos.w * pos.z);
  double cosy_cosp = 1 - 2 * (pos.z * pos.z);
  return atan2(siny_cosp, cosy_cosp);
}

PIDActionServer::PIDActionServer(const rclcpp::NodeOptions &options)
    : Node("pid_action_server", options) {
  using std::placeholders::_1, std::placeholders::_2;
  this->declare_parameter<bool>("is_sim", false);
  this->is_sim = this->get_parameter("is_sim").as_bool();
  this->declare_parameter<int>("debug_lvl", 0);
  this->debug_lvl = this->get_parameter("debug_lvl").as_int();
  if (this->debug_lvl > 2 || this->debug_lvl < 0) {
    RCLCPP_INFO(this->get_logger(), "give a valid debug lvl");
    rclcpp::shutdown();
    return;
  }
  this->action_server_ = rclcpp_action::create_server<PID>(
      this, "pid", std::bind(&PIDActionServer::handle_goal, this, _1, _2),
      std::bind(&PIDActionServer::handle_cancel, this, _1),
      std::bind(&PIDActionServer::handle_accepted, this, _1));
  RCLCPP_INFO(this->get_logger(), "PID action server initialized");
}

rclcpp_action::GoalResponse PIDActionServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const PID::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PIDActionServer::handle_cancel(
    const std::shared_ptr<GoalHandlePID> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PIDActionServer::handle_accepted(
    const std::shared_ptr<GoalHandlePID> goal_handle) {
  using std::placeholders::_1;
  std::thread{std::bind(&PIDActionServer::execute, this, _1), goal_handle}
      .detach();
}

void PIDActionServer::execute(
    const std::shared_ptr<GoalHandlePID> goal_handle) {
  rclcpp::Rate loop_rate(50);
  PIDState pid_state;
  const auto goal = goal_handle->get_goal();
  std::stringstream robo_pub_name, robo_sub_name;
  std::stringstream init_debug_msg;
  init_debug_msg << "Executing goal for: " << std::to_string(goal->robot_id);
  RCLCPP_INFO(this->get_logger(), init_debug_msg.str().c_str());

  if (this->is_sim) {
    robo_pub_name << "/model/robot_" << std::to_string(goal->robot_id)
                  << "/cmd_vel";
    robo_sub_name << "/model/robot_" << std::to_string(goal->robot_id)
                  << "/pose";
  } else {
    robo_pub_name << "/diffdrive_twist_" << std::to_string(goal->robot_id);
    robo_sub_name << "/robot" << std::to_string(goal->robot_id) << "/pose";
  }

  auto robot_pub = this->create_publisher<geometry_msgs::msg::Twist>(
      robo_pub_name.str(), 10);
  geometry_msgs::msg::Pose current_pose;
  bool _ready = false;
  auto cb_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = cb_group;
  auto subscriber_pos =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
          robo_sub_name.str(), 10,
          [this, &current_pose,
           &_ready](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            current_pose = msg->pose;
            if (!_ready) {
              _ready = true;
              RCLCPP_INFO(this->get_logger(), "PID is ready, starting!!");
            }
          },
          options);

  auto feedback = std::make_shared<PID::Feedback>();
  auto result = std::make_shared<PID::Result>();
  const geometry_msgs::msg::Pose &target_pose = goal->target_pose;
  builtin_interfaces::msg::Time current_time;
  double current_x, current_y, current_theta;
  double target_x = target_pose.position.x;
  double target_y = target_pose.position.y;
  double target_theta;
  double theta_error;
  bool reached_goal = false;
  builtin_interfaces::msg::Time start_time = this->now();
  int debug_state = 0;

  RCLCPP_INFO(this->get_logger(), "PID waiting for pos ");
  while (!reached_goal && rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      result->error_code = result_code::CANCELED;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
    current_time = this->now();
    if (current_time.sec - start_time.sec >= GOAL_TIMEOUT) {
      result->error_code = result_code::TIMED_OUT;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Robot Timeout");
      return;
    }
    if (!_ready) {
      loop_rate.sleep();
      continue;
    }

    current_x = current_pose.position.x;
    current_y = current_pose.position.y;
    current_theta = quaternion_to_yaw(current_pose.orientation);
    target_theta = atan2(target_y - current_y, target_x - current_x);
    theta_error = target_theta - current_theta;
    if (theta_error > M_PI) theta_error -= 2 * M_PI;
    if (theta_error < -M_PI) theta_error += 2 * M_PI;

    if (this->debug_lvl == 2) {
      std::stringstream debug_statement;
      debug_statement << "for robot " << goal->robot_id;
      RCLCPP_INFO(this->get_logger(), debug_statement.str().c_str());
      RCLCPP_INFO(this->get_logger(), std::string("current theta: ")
                                          .append(std::to_string(current_theta))
                                          .append(" --- target theta: ")
                                          .append(std::to_string(target_theta))
                                          .c_str());
      RCLCPP_INFO(this->get_logger(), (std::string("current pos: ")
                                           .append(std::to_string(current_x))
                                           .append(" ")
                                           .append(std::to_string(current_y)))
                                          .c_str());
      RCLCPP_INFO(this->get_logger(),
                  (std::string("current angle: ")
                       .append(std::to_string(current_theta))
                       .c_str()));
      RCLCPP_INFO(this->get_logger(), (std::string("theta error: ")
                                           .append(std::to_string(theta_error))
                                           .c_str()));
    }

    double distance_error =
        sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));

    // Turn robot to face target point
    // TODO(jaxon): we shouldnt have to check this
    if (fabs(theta_error) > ANGLE_TOLERANCE) {
      double angular_velocity =
          angular_error_to_velocity(theta_error, current_time, &pid_state);
      geometry_msgs::msg::Twist robo_msg =
          create_twist(0.0, angular_velocity, current_time).twist;
      robot_pub->publish(robo_msg);
      if (debug_state != 1 && this->debug_lvl >= 1) {
        std::stringstream debug_statement;
        debug_statement << "spinning " << goal->robot_id << " "
                        << fabs(theta_error);
        RCLCPP_INFO(this->get_logger(), debug_statement.str().c_str());
        debug_state = 1;
      }
      loop_rate.sleep();
      continue;
    }

    if (distance_error > DISTANCE_TOLERANCE) {
      /*double linear_velocity =*/
      /*    linear_error_to_velocity(distance_error, current_time, pid_state);*/
      geometry_msgs::msg::Twist robo_msg =
          create_twist(0.8, 0.0, current_time).twist;
      robot_pub->publish(robo_msg);
      if (debug_state != 2 && this->debug_lvl >= 1) {
        std::stringstream debug_statement;
        debug_statement << "moving " << goal->robot_id;
        RCLCPP_INFO(this->get_logger(), debug_statement.str().c_str());
        debug_state = 2;
      }
      loop_rate.sleep();
      continue;
    }

    reached_goal = true;
    geometry_msgs::msg::Twist robo_msg =
        create_twist(0.0, 0.0, current_time).twist;
    robot_pub->publish(robo_msg);
    loop_rate.sleep();
  }

  if (rclcpp::ok()) {
    result->error_code = result_code::SUCCEED;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}

}  // namespace control_algorithms

RCLCPP_COMPONENTS_REGISTER_NODE(control_algorithms::PIDActionServer)
