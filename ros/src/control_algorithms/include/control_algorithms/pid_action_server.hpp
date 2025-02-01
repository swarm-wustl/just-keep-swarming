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

/*
* camera node initializes all ESP32s
* camera node tells PID controller: hey, this robot exists, here's my ID, make a topic for me
* PID controller responds: ok, here's a topic for the robot
* camera node sends ID back to ESP32
* ESP32 subscribes to PID topic
*/

/*
* subscribe to camera topic
* callback on new data
* for each robot, run pid 
* pid fucking somehow has the target position
* pid sends data to each robot

* TODO: make flowchart
*/


namespace control_algorithms {

class PIDActionServer : public rclcpp::Node {
    public:
        using PID = control_algorithms::action::PID;
        using GoalHandlePID = rclcpp_action::ServerGoalHandle<PID>;

    private:
        rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
}

}