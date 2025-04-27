import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy

X_BUTTON = 2
Y_BUTTON = 3
LB_BUTTON = 4
RB_BUTTON = 5

LEFT_VER_AXIS = 1
LEFT_HOR_AXIS = 0
RIGHT_VER_AXIS = 4
RIGHT_HOR_AXIS = 3


class JoyToTwistNode(Node):
    def __init__(self):
        super().__init__("joy_to_twist")

        # Params
        self.declare_parameter("N", 6)
        self.num_robots = self.get_parameter("N").get_parameter_value().integer_value

        self.max_linear_speed = 1.0  # Max linear speed in m/s
        self.max_angular_speed = 1.0  # Max angular speed in rad/s

        # Create publishers for each robot
        self._publishers = [
            self.create_publisher(Twist, f"/diffdrive_twist_{i}", 10)
            for i in range(self.num_robots)
        ]

        # Subscribe to joystick input
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        # State
        self.current_i = 0  # Current robot index
        self.control_all = False  # Toggle for controlling all robots
        self.split_control = False  # Toggle for split control mode
        self.prev = {
            "rb": 0,
            "lb": 0,
            "prev_button_x": 0,
            "prev_button_y": 0,
        }

    def joy_callback(self, msg):
        axes = msg.axes
        buttons = msg.buttons

        self._handle_robot_selection(buttons)
        self._handle_control_all_toggle(buttons)
        self._handle_split_control_toggle(buttons)

        if self.split_control:
            self._handle_split_control(axes)
        else:
            self._handle_normal_control(axes)

    def _handle_robot_selection(self, buttons):
        lb = buttons[LB_BUTTON]
        rb = buttons[RB_BUTTON]

        if not self.control_all and not self.split_control:
            if rb == 1 and self.prev["rb"] != 1:
                self.current_i = (self.current_i + 1) % self.num_robots
                self.get_logger().info(f"Switched to robot {self.current_i}")
            elif lb == 1 and self.prev["lb"] != 1:
                self.current_i = (self.current_i - 1) % self.num_robots
                self.get_logger().info(f"Switched to robot {self.current_i}")
        self.prev["rb"] = rb
        self.prev["lb"] = lb

    def _handle_control_all_toggle(self, buttons):
        button_y = buttons[Y_BUTTON]
        if button_y == 1 and self.prev["button_y"] == 0:
            self.control_all = not self.control_all
            self.split_control = False  # Disable split mode when toggling control_all
            if self.control_all:
                self.get_logger().info("Controlling all robots")
            else:
                self.get_logger().info(f"Controlling robot {self.current_i}")
        self.prev["button_y"] = button_y

    def _handle_split_control_toggle(self, buttons):
        button_x = buttons[X_BUTTON]
        if button_x == 1 and self.prev["button_x"] == 0:
            self.split_control = not self.split_control
            self.control_all = False  # Disable all mode when toggling split_control
            if self.split_control:
                left_group = ",".join([str(n) for n in range(self.num_robots // 2)])
                right_group = ",".join(
                    [str(n) for n in range(self.num_robots // 2, self.num_robots)]
                )
                self.get_logger().info(
                    f"Split control: Left stick {left_group}, Right stick {right_group}"
                )
            else:
                self.get_logger().info(f"Controlling robot {self.current_i}")
        self.prev["button_x"] = button_x

    def _handle_split_control(self, axes):
        twist_left = Twist()
        twist_right = Twist()

        # Left stick: x velocity and yaw
        twist_left.linear.x = axes[LEFT_VER_AXIS] * self.max_linear_speed
        twist_left.angular.z = axes[LEFT_HOR_AXIS] * self.max_angular_speed

        # Right stick: x velocity and yaw
        twist_right.linear.x = axes[RIGHT_VER_AXIS] * self.max_linear_speed
        twist_right.angular.z = axes[RIGHT_HOR_AXIS] * self.max_angular_speed

        for i in range(self.num_robots // 2):
            self._publishers[i].publish(twist_left)
        for i in range(self.num_robots // 2, self.num_robots):
            self._publishers[i].publish(twist_right)

    def _handle_normal_control(self, axes):
        twist = Twist()

        # Left stick: x velocity (axis 1) and yaw (axis 0)
        twist.linear.x = axes[LEFT_VER_AXIS] * self.max_linear_speed
        twist.angular.z = axes[LEFT_HOR_AXIS] * self.max_angular_speed

        # Right stick: pitch (angular x) and roll (angular y)
        twist.angular.x = axes[RIGHT_HOR_AXIS] * self.max_angular_speed
        twist.angular.y = axes[RIGHT_VER_AXIS] * self.max_angular_speed * 0.25

        if self.control_all:
            for pub in self._publishers:
                pub.publish(twist)
        else:
            self._publishers[self.current_i].publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToTwistNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
