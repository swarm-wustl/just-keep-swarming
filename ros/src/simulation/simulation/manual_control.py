#!/usr/bin/env python3
"""
Manual control node for the docking demo.

Provides keyboard-based control for:
- Selecting and driving individual robots
- Docking/undocking adjacent robots

Controls:
    W/Up:     Drive forward
    S/Down:   Drive backward
    A/Left:   Turn left
    D/Right:  Turn right
    1-4:      Select robot 0-3
    Space:    Stop selected robot
    X:        Stop all robots
    J:        Dock selected robot to NEXT robot (N -> N+1)
    K:        Undock selected robot from NEXT robot
    L:        Dock selected robot to PREVIOUS robot (N-1 -> N)
    ;:        Undock selected robot from PREVIOUS robot
    Q/Esc:    Quit

The docking commands connect chassis-to-chassis using the AttachablePlugin.
"""

import subprocess
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ManualControlNode(Node):
    """Manual control node for docking demo robots."""

    def __init__(self):
        super().__init__("manual_control")

        self.num_robots = 4
        self.selected_robot = 0
        self.linear_speed = 0.3
        self.angular_speed = 1.0

        # Create publishers for each robot
        self.cmd_vel_pubs = []
        for i in range(self.num_robots):
            pub = self.create_publisher(Twist, f"/model/robot_{i}/cmd_vel", 10)
            self.cmd_vel_pubs.append(pub)

        self.get_logger().info("Manual control node started")
        self.print_help()

    def print_help(self):
        """Print control instructions."""
        print("\n" + "=" * 50)
        print("DOCKING DEMO MANUAL CONTROL")
        print("=" * 50)
        print(f"Selected robot: robot_{self.selected_robot}")
        print("-" * 50)
        print("Movement:")
        print("  W/Up    : Forward")
        print("  S/Down  : Backward")
        print("  A/Left  : Turn left")
        print("  D/Right : Turn right")
        print("  Space   : Stop selected robot")
        print("  X       : Stop all robots")
        print("-" * 50)
        print("Robot selection:")
        print("  1-4     : Select robot 0-3")
        print("-" * 50)
        print("Docking (connects chassis links):")
        print("  J       : Dock selected -> next (N to N+1)")
        print("  K       : Undock selected -> next")
        print("  L       : Dock previous -> selected (N-1 to N)")
        print("  ;       : Undock previous -> selected")
        print("-" * 50)
        print("  Q/Esc   : Quit")
        print("=" * 50 + "\n")

    def send_velocity(self, linear: float, angular: float):
        """Send velocity command to selected robot."""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pubs[self.selected_robot].publish(msg)

    def stop_robot(self, robot_id: int):
        """Stop a specific robot."""
        msg = Twist()
        self.cmd_vel_pubs[robot_id].publish(msg)

    def stop_all(self):
        """Stop all robots."""
        for i in range(self.num_robots):
            self.stop_robot(i)
        print("All robots stopped")

    def dock_robots(self, parent_id: int, child_id: int):
        """Dock two robots together (parent -> child)."""
        if parent_id < 0 or parent_id >= self.num_robots:
            print(f"Invalid parent robot: {parent_id}")
            return
        if child_id < 0 or child_id >= self.num_robots:
            print(f"Invalid child robot: {child_id}")
            return
        if parent_id == child_id:
            print("Cannot dock robot to itself")
            return

        cmd = (
            f'gz topic -t /attach -m gz.msgs.StringMsg -p '
            f"'data:\"[robot_{parent_id}][chassis][robot_{child_id}][chassis][attach]\"'"
        )
        print(f"Docking robot_{parent_id} -> robot_{child_id}")
        subprocess.run(cmd, shell=True, capture_output=True)

    def undock_robots(self, parent_id: int, child_id: int):
        """Undock two robots (parent -> child)."""
        if parent_id < 0 or parent_id >= self.num_robots:
            print(f"Invalid parent robot: {parent_id}")
            return
        if child_id < 0 or child_id >= self.num_robots:
            print(f"Invalid child robot: {child_id}")
            return

        cmd = (
            f'gz topic -t /attach -m gz.msgs.StringMsg -p '
            f"'data:\"[robot_{parent_id}][chassis][robot_{child_id}][chassis][detach]\"'"
        )
        print(f"Undocking robot_{parent_id} -> robot_{child_id}")
        subprocess.run(cmd, shell=True, capture_output=True)

    def select_robot(self, robot_id: int):
        """Select a robot for control."""
        if 0 <= robot_id < self.num_robots:
            self.selected_robot = robot_id
            colors = ["Red", "Green", "Blue", "Yellow"]
            print(f"Selected: robot_{robot_id} ({colors[robot_id]})")


def get_key(settings):
    """Get a single keypress."""
    tty.setraw(sys.stdin.fileno())
    try:
        key = sys.stdin.read(1)
        # Handle escape sequences (arrow keys)
        if key == "\x1b":
            key += sys.stdin.read(2)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    """Main function."""
    rclpy.init()
    node = ManualControlNode()

    # Save terminal settings
    settings = termios.tcgetattr(sys.stdin)

    try:
        while rclpy.ok():
            key = get_key(settings)

            # Movement keys
            if key in ("w", "\x1b[A"):  # W or Up arrow
                node.send_velocity(node.linear_speed, 0.0)
            elif key in ("s", "\x1b[B"):  # S or Down arrow
                node.send_velocity(-node.linear_speed, 0.0)
            elif key in ("a", "\x1b[D"):  # A or Left arrow
                node.send_velocity(0.0, node.angular_speed)
            elif key in ("d", "\x1b[C"):  # D or Right arrow
                node.send_velocity(0.0, -node.angular_speed)
            elif key == " ":  # Space - stop selected
                node.stop_robot(node.selected_robot)
                print(f"robot_{node.selected_robot} stopped")
            elif key == "x":  # Stop all
                node.stop_all()

            # Robot selection
            elif key == "1":
                node.select_robot(0)
            elif key == "2":
                node.select_robot(1)
            elif key == "3":
                node.select_robot(2)
            elif key == "4":
                node.select_robot(3)

            # Docking controls
            elif key == "j":  # Dock to next
                node.dock_robots(node.selected_robot, node.selected_robot + 1)
            elif key == "k":  # Undock from next
                node.undock_robots(node.selected_robot, node.selected_robot + 1)
            elif key == "l":  # Dock from previous
                node.dock_robots(node.selected_robot - 1, node.selected_robot)
            elif key == ";":  # Undock from previous
                node.undock_robots(node.selected_robot - 1, node.selected_robot)

            # Quit
            elif key in ("q", "\x1b", "\x03"):  # Q, Esc, Ctrl+C
                print("\nExiting...")
                break

            # Help
            elif key == "h":
                node.print_help()

            # Process ROS callbacks
            rclpy.spin_once(node, timeout_sec=0.01)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
