#!/usr/bin/env python3
"""
Manual control node for the docking demo.

Features:
- Keyboard control for driving robots
- Automatic command relay to all connected robots
- Snap-to-alignment when docking

Controls:
    W/Up:     Drive forward
    S/Down:   Drive backward
    A/Left:   Turn left
    D/Right:  Turn right
    1-4:      Select robot 0-3
    Space:    Stop selected robot (and connected)
    X:        Stop all robots
    J:        Dock selected robot to NEXT robot (N -> N+1)
    K:        Undock selected robot from NEXT robot
    L:        Dock selected robot to PREVIOUS robot (N-1 -> N)
    ;:        Undock selected robot from PREVIOUS robot
    H:        Show help
    Q/Esc:    Quit
"""

import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from simulation.docking_api import DockingController


class ManualControlNode(DockingController):
    """Manual control node for docking demo robots, extends DockingController."""

    def __init__(self):
        super().__init__(num_robots=4)

        # Override node name
        self.selected_robot = 0
        self.linear_speed = 0.3
        self.angular_speed = 1.0

        self.get_logger().info("Manual control node started")
        self.print_help()

    def print_help(self):
        """Print control instructions."""
        print("\n" + "=" * 50)
        print("DOCKING DEMO MANUAL CONTROL")
        print("=" * 50)
        print(f"Selected robot: robot_{self.selected_robot}")
        print(f"Connections: {self.format_connections()}")
        print("-" * 50)
        print("Movement (relayed to all connected robots):")
        print("  W/Up    : Forward")
        print("  S/Down  : Backward")
        print("  A/Left  : Turn left")
        print("  D/Right : Turn right")
        print("  Space   : Stop selected + connected")
        print("  X       : Stop all robots")
        print("-" * 50)
        print("Robot selection:")
        print("  1-4     : Select robot 0-3")
        print("-" * 50)
        print("Docking (auto-aligns robots):")
        print("  J       : Dock selected -> next (N to N+1)")
        print("  K       : Undock selected -> next")
        print("  L       : Dock previous -> selected (N-1 to N)")
        print("  ;       : Undock previous -> selected")
        print("-" * 50)
        print("  H       : Show this help")
        print("  Q/Esc   : Quit")
        print("=" * 50 + "\n")

    def format_connections(self) -> str:
        """Format connections for display."""
        if not self.connections:
            return "None"
        return ", ".join(f"{p}->{c}" for p, c in sorted(self.connections))

    def send_velocity_to_chain(self, linear: float, angular: float):
        """Send velocity command to ALL robots in the chain."""
        connected = self.get_connected_robots(self.selected_robot)

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular

        for robot_id in connected:
            self.cmd_vel_pubs[robot_id].publish(msg)

    def stop_chain(self, robot_id: int):
        """Stop a specific robot and all connected robots."""
        connected = self.get_connected_robots(robot_id)
        msg = Twist()
        for _ in range(5):
            for rid in connected:
                self.cmd_vel_pubs[rid].publish(msg)
            time.sleep(0.02)

    def stop_all(self):
        """Stop all robots."""
        msg = Twist()
        for _ in range(5):
            for i in range(self.num_robots):
                self.cmd_vel_pubs[i].publish(msg)
            time.sleep(0.02)
        print("All robots stopped")

    def select_robot(self, robot_id: int):
        """Select a robot for control."""
        if 0 <= robot_id < self.num_robots:
            self.selected_robot = robot_id
            colors = ["Red", "Green", "Blue", "Yellow"]
            connected = self.get_connected_robots(robot_id)
            if len(connected) > 1:
                others = [f"robot_{r}" for r in connected if r != robot_id]
                print(f"Selected: robot_{robot_id} ({colors[robot_id]}) + connected: {', '.join(others)}")
            else:
                print(f"Selected: robot_{robot_id} ({colors[robot_id]})")

    def do_dock(self, parent_id: int, child_id: int):
        """Dock two robots with user feedback."""
        if parent_id < 0 or parent_id >= self.num_robots:
            print(f"Invalid parent robot: {parent_id}")
            return
        if child_id < 0 or child_id >= self.num_robots:
            print(f"Invalid child robot: {child_id}")
            return

        success = self.dock(parent_id, child_id, auto_align=True)
        if success:
            print(f"Connections: {self.format_connections()}")
        else:
            print("Docking failed")

    def do_undock(self, parent_id: int, child_id: int):
        """Undock two robots with user feedback."""
        if parent_id < 0 or parent_id >= self.num_robots:
            print(f"Invalid parent robot: {parent_id}")
            return
        if child_id < 0 or child_id >= self.num_robots:
            print(f"Invalid child robot: {child_id}")
            return

        # Stop robots first
        self.stop_robot(parent_id)
        self.stop_robot(child_id)
        time.sleep(0.1)

        success = self.undock(parent_id, child_id)
        print(f"Connections: {self.format_connections()}")


def get_key(settings):
    """Get a single keypress."""
    tty.setraw(sys.stdin.fileno())
    try:
        key = sys.stdin.read(1)
        if key == "\x1b":
            key += sys.stdin.read(2)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    """Main function."""
    rclpy.init()
    node = ManualControlNode()

    settings = termios.tcgetattr(sys.stdin)

    try:
        while rclpy.ok():
            key = get_key(settings)

            # Movement keys
            if key in ("w", "\x1b[A"):  # W or Up arrow
                node.send_velocity_to_chain(node.linear_speed, 0.0)
            elif key in ("s", "\x1b[B"):  # S or Down arrow
                node.send_velocity_to_chain(-node.linear_speed, 0.0)
            elif key in ("a", "\x1b[D"):  # A or Left arrow
                node.send_velocity_to_chain(0.0, node.angular_speed)
            elif key in ("d", "\x1b[C"):  # D or Right arrow
                node.send_velocity_to_chain(0.0, -node.angular_speed)
            elif key == " ":  # Space - stop selected + connected
                node.stop_chain(node.selected_robot)
                connected = node.get_connected_robots(node.selected_robot)
                if len(connected) > 1:
                    print(f"Stopped robots: {', '.join(f'robot_{r}' for r in sorted(connected))}")
                else:
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
            elif key in ("j", "J"):  # Dock to next
                node.do_dock(node.selected_robot, node.selected_robot + 1)
            elif key in ("k", "K"):  # Undock from next
                node.do_undock(node.selected_robot, node.selected_robot + 1)
            elif key in ("l", "L"):  # Dock from previous
                node.do_dock(node.selected_robot - 1, node.selected_robot)
            elif key == ";":  # Undock from previous
                node.do_undock(node.selected_robot - 1, node.selected_robot)

            # Quit
            elif key in ("q", "\x03"):  # Q or Ctrl+C
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
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
