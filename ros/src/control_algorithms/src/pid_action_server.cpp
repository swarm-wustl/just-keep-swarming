// Copyright 2025 Preston Meek, Jaxon Poentis
#include "control_algorithms/pid_action_server.hpp"

// TODO(preston): test Kd
// TODO(preston): add Ki
// TODO(preston): add reset functionality when going to a very new point

// to reset prev time, prev error, etc.)

#define DISTANCE_TOLERANCE 0.05  // meters
#define ANGLE_TOLERANCE 0.05     // rad

// Angular constsants
#define Kp_angular 2.0
#define Kd_angular 0.5

// Linear constants
#define Kp_linear 1.0
#define Kd_linear 0.3

// Timeout Constants (we are giving the robot 3 minutes to move to a block, if
// it can't do that it is cooked)
#define GOAL_TIMEOUT 360.0

static int angular_time_prev = -1;
static double angular_error_prev = 0.0;

static int linear_time_prev = -1;
static double linear_error_prev = 0.0;

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

static double delta_time(builtin_interfaces::msg::Time final,
                         builtin_interfaces::msg::Time initial) {
  const double NSEC_TO_SEC = 1000000000.0;

  int sec = final.sec - initial.sec;
  int nsec = final.nanosec - initial.nanosec;

  if (nsec < 0) {
    sec -= 1;
    nsec += NSEC_TO_SEC;
  }

  return sec + (nsec / NSEC_TO_SEC);
}

static double angular_error_to_velocity(double error,
                                        builtin_interfaces::msg::Time time) {
  int time_curr = time.sec;
  double derivative;

  // If this is the first packet, don't include a derivative term
  // Otherwise, calculate the derivative based on current and previous values
  if (angular_time_prev == -1 || time_curr == angular_time_prev) {
    derivative = 0;
  } else {
    derivative = (error - angular_error_prev) / (time_curr - angular_time_prev);
  }

  angular_time_prev = time_curr;
  angular_error_prev = error;

  return (Kp_angular * error) + (Kd_angular * derivative);
}

static double linear_error_to_velocity(double error,
                                       builtin_interfaces::msg::Time time) {
  int time_curr = time.sec;
  double derivative;

  // If this is the first packet, don't include a derivative term
  // Otherwise, calculate the derivative based on current and previous values
  if (linear_time_prev == -1 || time_curr == linear_time_prev) {
    derivative = 0;
  } else {
    derivative = (error - linear_error_prev) / (time_curr - linear_time_prev);
  }

  linear_time_prev = time_curr;
  linear_error_prev = error;

  return (Kp_linear * error) + (Kd_linear * derivative);
}

// NOTE: The robot is assumed to only rotate about the z-axis
// Therefore, all quaternion input data should have x=0, y=0, w and z are
// nonzero
static double quaternion_to_yaw(geometry_msgs::msg::Quaternion pos) {
  return atan2(2.0 * (pos.w * pos.z + pos.x * pos.y),
               1.0 - 2.0 * (pos.y * pos.y + pos.z * pos.z));
}

namespace control_algorithms {

PIDActionServer::PIDActionServer(const rclcpp::NodeOptions &options)
    : Node("pid_action_server", options) {
  using std::placeholders::_1, std::placeholders::_2;

  // update this and allow for calls
  this->robot_map_ = std::unordered_map<uint, geometry_msgs::msg::Pose>();

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
/*

  First it gets all the information from the goal, such as the target and the
  robot id Then we create a publisher and subscriber that will then recieve
  positions and send them The subscriber will allow data to be recieved until it
  doens't need to, removing the need to continous recieve messages

  TO DO: Will need to make response or service call that tells the position to
  stop sending messages until the nxt plan time step

  Then we do a while loop until we reach the goal, in which we refernce the
  robot_positions.

  Note on threading: We are using ros.spin_some to allow for the aciton server
  to recieve messages on the call backs and stil execute preventing blocking
  behavior



*/
void PIDActionServer::execute(
    const std::shared_ptr<GoalHandlePID> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  rclcpp::Rate loop_rate(50);  // this is how many iteratiosn per second, so its
                               // HZ, so exec everthing 0.05 seconds
  double angular_time_prev = -1;
  double linear_time_prev = -1;

  const auto goal = goal_handle->get_goal();

  this->robot_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "robo_pub_" + std::to_string(goal->robot_id),
      10);  // sus shit

  rclcpp::Subscription<shared_types::msg::PidPosition>::SharedPtr
      subscriber_pos_ =
          this->create_subscription<shared_types::msg::PidPosition>(
              "robot_filtered_pos_" + std::to_string(goal->robot_id), 10,
              std::bind(&PIDActionServer::position_callback_, this,
                        std::placeholders::_1));

  auto feedback = std::make_shared<PID::Feedback>();
  auto result = std::make_shared<PID::Result>();

  geometry_msgs::msg::Pose current_pose;  // TODO(preston): Need to get the
                                          // actual position from camera feed

  const geometry_msgs::msg::Pose &target_pose = goal->target_pose;

  builtin_interfaces::msg::Time current_time;

  double current_x;
  double current_y;
  double current_theta;

  double target_x = target_pose.position.x;
  double target_y = target_pose.position.y;
  double target_theta;

  double theta_error;
  bool reached_goal = false;

  // make better log messages
  builtin_interfaces::msg::Time start_time = this->now();
  // need to get the current pose
  while (!reached_goal && rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      result->error_code = result_code::CANCELED;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    // fix this lol
    current_time = this->now();

    if (current_time.sec - start_time.sec >= GOAL_TIMEOUT) {
      result->error_code = result_code::TIMED_OUT;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Robot Timeout");
      return;
    }

    // gonan need to actually get the time update

    // grabs the position

    // since we're using spin some, we most likely do not need to use the mutex
    // lock this->position_mutex_.lock();
    current_pose = this->robot_map_[goal->robot_id];
    // this->position_mutex_.unlock();

    current_x = current_pose.position.x;
    current_y = current_pose.position.y;

    current_theta = quaternion_to_yaw(current_pose.orientation);
    target_theta = atan2(target_y - current_y, target_x - current_x);

    theta_error = target_theta - current_theta;
    // Normalize angle to [-pi, pi]
    if (theta_error > M_PI) {
      theta_error -= 2 * M_PI;
    }

    if (theta_error < -1 * M_PI) {
      theta_error += 2 * M_PI;
    }

    // Turn robot to face target point
    if (fabs(theta_error) > ANGLE_TOLERANCE) {
      double angular_velocity =
          angular_error_to_velocity(theta_error, current_time);

      // TODO(preston): do something

      geometry_msgs::msg::TwistStamped robo_msg =
          create_twist(0.0, angular_velocity, current_time);

      // printf("Rotating to target: Theta Error = %.2f\n", theta_error);

      this->robot_pub_->publish(robo_msg);
      // continue loop early so we don't try moving linearly if we still need to
      // fix our angle

      // maybe spin_some can do spin some to prevent full on blocking
      feedback->current_pose = current_pose;
      goal_handle->publish_feedback(feedback);
      rclcpp::spin_some(this->get_node_base_interface());
      loop_rate.sleep();
      continue;
    }

    // If angle is within tolerance, move robot to reach target distance
    double distance_error =
        sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));

    if (distance_error > DISTANCE_TOLERANCE) {
      double linear_velocity =
          linear_error_to_velocity(distance_error, current_time);

      // TODO(preston): do something
      // printf("Moving to target: Distance Error = %.2f\n", distance_error);

      geometry_msgs::msg::TwistStamped robo_msg =
          create_twist(linear_velocity, 0.0, current_time);

      this->robot_pub_->publish(robo_msg);

      // Return early if still trying to reach target position
      feedback->current_pose = current_pose;
      goal_handle->publish_feedback(feedback);
      rclcpp::spin_some(this->get_node_base_interface());

      loop_rate.sleep();
      continue;
    }
    reached_goal = true;

    rclcpp::spin_some(this->get_node_base_interface());
    loop_rate.sleep();
  }

  if (rclcpp::ok()) {
    result->error_code = result_code::SUCCEED;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
  // clean up should be automatic, as the shared_ptr for the subscriber should
  // destroy the subscriber.
}

void PIDActionServer::position_callback_(
    const shared_types::msg::PidPosition pos_msg_) {
  // since we're using spin some, we most likely do not need to use the mutex
  // lock
  uint32_t rob_id_ = pos_msg_.robot_id;
  // this->position_mutex_.lock();
  if (this->robot_map_.find(rob_id_) != this->robot_map_.end()) {
    this->robot_map_.insert({rob_id_, pos_msg_.position});
  } else {
    this->robot_map_[rob_id_].set__position(pos_msg_.position.position);
    this->robot_map_[rob_id_].set__orientation(pos_msg_.position.orientation);
  }
  // this->position_mutex_.unlock();
}

}  // namespace control_algorithms
