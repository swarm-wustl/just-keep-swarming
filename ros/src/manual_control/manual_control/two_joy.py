import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy

# Axis Mappings (Standard Xbox/Logitech mapping)
LEFT_VER_AXIS = 1     # Robot 1 Linear X
LEFT_HOR_AXIS = 0     # Robot 1 Angular Z
RIGHT_VER_AXIS = 4    # Robot 2 Linear X
RIGHT_HOR_AXIS = 3    # Robot 2 Angular Z
DPAD_HOR_AXIS = 6     # Robot 1 Linear Y (Horizontal)
DPAD_VER_AXIS = 7     # Robot 2 Linear Y (Horizontal)

BOT1_ID = "4164"
BOT2_ID = "9a58"

class DualBotControlNode(Node):
    def __init__(self):
        super().__init__("dual_bot_joy_control")

        self.max_linear_speed = 1.0   # Max linear speed in m/s
        self.max_angular_speed = 1.0  # Max angular speed in rad/s

        # Create specific publishers for the two robots
        self.pub_robot1 = self.create_publisher(Twist, f"/swarmbot_{BOT1_ID}/cmd_vel", 10)
        self.pub_robot2 = self.create_publisher(Twist, f"/swarmbot_{BOT2_ID}/cmd_vel", 10)

        # Subscribe to joystick input
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        
        self.get_logger().info("Dual Bot Control Node Started")
        self.get_logger().info("Left Stick + D-Pad L/R -> swarmbot_4164")
        self.get_logger().info("Right Stick + D-Pad U/D -> swarmbot_9a58")

    def joy_callback(self, msg):
        axes = msg.axes

        # --- ROBOT 1 CONTROL ---
        twist1 = Twist()
        # Main movement (Left Stick)
        twist1.linear.x = axes[LEFT_VER_AXIS] * self.max_linear_speed
        twist1.angular.z = axes[LEFT_HOR_AXIS] * self.max_angular_speed
        
        # Holonomic Horizontal (D-Pad Left/Right)
        # Usually D-Pad L/R is axis 6. Left is often +1, Right is -1 or vice versa depending on hardware.
        # Assuming standard map: +1 is Left, -1 is Right. 
        if DPAD_HOR_AXIS < len(axes):
            twist1.linear.y = axes[DPAD_HOR_AXIS] * self.max_linear_speed

        # --- ROBOT 2 CONTROL ---
        twist2 = Twist()
        # Main movement (Right Stick)
        twist2.linear.x = axes[RIGHT_VER_AXIS] * self.max_linear_speed
        twist2.angular.z = axes[RIGHT_HOR_AXIS] * self.max_angular_speed

        # Holonomic Horizontal (D-Pad Up/Down)
        # User requested D-pad up-down to control horizontal velocity for robot 2.
        if DPAD_VER_AXIS < len(axes):
            twist2.linear.y = axes[DPAD_VER_AXIS] * self.max_linear_speed

        print(twist2)
        # Publish commands
        self.pub_robot1.publish(twist1)
        self.pub_robot2.publish(twist2)

def main(args=None):
    rclpy.init(args=args)
    node = DualBotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
