#include "pid_action_server.hpp"

namespace control_algorithms {

PIDActionServer::PIDActionServer(const rclcpp::NodeOptions &options)
        : Node("pid_action_server", options) {
    
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Assemble>(
      this, "pid",
      std::bind(&AssemblerActionServer::handle_goal, this, _1, _2),
      std::bind(&AssemblerActionServer::handle_cancel, this, _1),
      std::bind(&AssemblerActionServer::handle_accepted, this, _1)
    );

    RCLCPP_INFO(this->get_logger(), "PID action server initialized");
}

}