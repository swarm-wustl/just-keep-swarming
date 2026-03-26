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

import json
import math
import os
import subprocess
import sys
import termios
import time
import tty
from typing import Optional, Set, Tuple

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

        # Track connections as a set of (parent, child) tuples
        self.connections: Set[tuple] = set()

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
        print("  T       : Test pose (move selected to 0.5, 0.5)")
        print("  R       : Test read+move (read pose, move +0.2 in X)")
        print("  P       : Position for docking (no attach)")
        print("  H       : Show this help")
        print("  Q/Esc   : Quit")
        print("=" * 50 + "\n")

    def format_connections(self) -> str:
        """Format connections for display."""
        if not self.connections:
            return "None"
        return ", ".join(f"{p}->{c}" for p, c in sorted(self.connections))

    def get_connected_robots(self, robot_id: int) -> Set[int]:
        """Get all robots connected to the given robot (including itself)."""
        connected = {robot_id}
        changed = True
        while changed:
            changed = False
            for parent, child in self.connections:
                if parent in connected and child not in connected:
                    connected.add(child)
                    changed = True
                if child in connected and parent not in connected:
                    connected.add(parent)
                    changed = True
        return connected

    def get_chain_root(self, robot_id: int) -> int:
        """Get the root robot of a connected chain (the one with no parent)."""
        connected = self.get_connected_robots(robot_id)
        # Find robot that is not a child in any connection
        for rid in connected:
            is_child = any(child == rid for _, child in self.connections if _ in connected)
            if not is_child:
                return rid
        return robot_id  # Fallback

    def send_velocity(self, linear: float, angular: float):
        """Send velocity command to the chain root only.

        When robots are connected via fixed joints, only the root robot
        should be driven - physics will move the rest of the chain.
        This prevents conflicting drive commands.
        """
        connected = self.get_connected_robots(self.selected_robot)
        root = self.get_chain_root(self.selected_robot)

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular

        # Only send to the root robot of the chain
        self.cmd_vel_pubs[root].publish(msg)

    def stop_robot(self, robot_id: int):
        """Stop a specific robot and all connected robots."""
        connected = self.get_connected_robots(robot_id)
        msg = Twist()
        # Send stop command multiple times to ensure it takes effect
        for _ in range(5):
            for rid in connected:
                self.cmd_vel_pubs[rid].publish(msg)
            time.sleep(0.02)

    def stop_all(self):
        """Stop all robots."""
        msg = Twist()
        # Send stop command multiple times to ensure it takes effect
        for _ in range(5):
            for i in range(self.num_robots):
                self.cmd_vel_pubs[i].publish(msg)
            time.sleep(0.02)
        print("All robots stopped")

    def get_robot_pose(self, robot_id: int, debug: bool = False) -> Optional[Tuple[float, float, float, float]]:
        """Get robot pose from Gazebo using gz topic to echo pose info."""
        try:
            # Echo one message from the dynamic pose topic
            cmd = (
                f'gz topic -e -t /world/docking_demo/dynamic_pose/info -n 1 '
                f'--json-output 2>/dev/null'
            )
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=3)

            if result.returncode == 0 and result.stdout.strip():
                data = json.loads(result.stdout)

                if debug:
                    # Print all available model names
                    names = [p.get("name", "?") for p in data.get("pose", [])]
                    print(f"    DEBUG: Available models: {names}")

                # Find the robot in the pose list
                robot_name = f"robot_{robot_id}"
                for pose in data.get("pose", []):
                    if pose.get("name") == robot_name:
                        pos = pose.get("position", {})
                        ori = pose.get("orientation", {})

                        x = pos.get("x", 0.0)
                        y = pos.get("y", 0.0)
                        z = pos.get("z", 0.0)

                        # Convert quaternion to yaw
                        qx = ori.get("x", 0.0)
                        qy = ori.get("y", 0.0)
                        qz = ori.get("z", 0.0)
                        qw = ori.get("w", 1.0)

                        if debug:
                            print(f"    DEBUG: {robot_name} raw pos={pos} ori={ori}")

                        # Yaw from quaternion
                        siny_cosp = 2.0 * (qw * qz + qx * qy)
                        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                        yaw = math.atan2(siny_cosp, cosy_cosp)

                        return (x, y, z, yaw)

                print(f"    WARNING: robot_{robot_id} not found in pose data")
            else:
                print(f"    WARNING: gz topic command failed or returned empty")

        except json.JSONDecodeError as e:
            self.get_logger().warn(f"JSON parse error: {e}")
        except Exception as e:
            self.get_logger().warn(f"Failed to get pose for robot_{robot_id}: {e}")
        return None

    def set_robot_pose(self, robot_id: int, x: float, y: float, z: float, yaw: float) -> bool:
        """Set robot pose in Gazebo using gz service."""
        # Round to 3 decimal places like manual command
        x = round(x, 3)
        y = round(y, 3)
        z = round(z, 3)

        qz = round(math.sin(yaw / 2.0), 6)
        qw = round(math.cos(yaw / 2.0), 6)

        # Build the request string exactly as it would be typed manually
        req = f'name: "robot_{robot_id}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: 0, y: 0, z: {qz}, w: {qw}}}'

        # Use os.system for direct shell execution (closest to manual terminal)
        cmd = f"gz service -s /world/docking_demo/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 --req '{req}'"

        print(f"    CMD: {cmd}")

        ret = os.system(cmd)
        print(f"    Return code: {ret}")

        time.sleep(0.15)  # Let simulation process the change
        return ret == 0

    def get_free_docking_direction(self, robot_id: int) -> Optional[float]:
        """Get the direction (yaw) where robot has no existing connection.

        Returns the yaw angle pointing away from any connected robots,
        or None if robot has no connections (any direction is fine).
        """
        connected = self.get_connected_robots(robot_id)
        if len(connected) <= 1:
            return None  # No connections, any direction works

        # Get this robot's pose
        my_pose = self.get_robot_pose(robot_id)
        if my_pose is None:
            return None
        mx, my, _, _ = my_pose

        # Find direction toward connected robots
        for other_id in connected:
            if other_id == robot_id:
                continue
            other_pose = self.get_robot_pose(other_id)
            if other_pose is None:
                continue
            ox, oy, _, _ = other_pose

            # Direction from this robot to connected robot
            to_other = math.atan2(oy - my, ox - mx)

            # Return opposite direction (away from connected robot)
            free_dir = to_other + math.pi
            if free_dir > math.pi:
                free_dir -= 2 * math.pi
            return free_dir

        return None

    def align_for_docking(self, parent_id: int, child_id: int) -> bool:
        """Position robots for docking.

        Detects which robot is already in a chain and moves the unconnected one.
        Positions new robot on the FREE side of anchor (opposite from existing connections).
        """
        print(f"  Aligning robot_{parent_id} to dock with robot_{child_id}...")

        # Stop both robots and try to cancel velocities
        for _ in range(10):
            self.cmd_vel_pubs[parent_id].publish(Twist())
            self.cmd_vel_pubs[child_id].publish(Twist())
            time.sleep(0.02)
        time.sleep(0.3)

        # Check which robot is in a chain (has existing connections)
        parent_connected = len(self.get_connected_robots(parent_id)) > 1
        child_connected = len(self.get_connected_robots(child_id)) > 1

        # Decide which robot to move
        # If parent is in a chain, move child to parent
        # Otherwise, move parent to child (default)
        if parent_connected and not child_connected:
            move_robot = child_id
            anchor_robot = parent_id
            print(f"  robot_{parent_id} is in a chain, moving robot_{child_id} to it")
        else:
            move_robot = parent_id
            anchor_robot = child_id
            if child_connected:
                print(f"  robot_{child_id} is in a chain, moving robot_{parent_id} to it")

        # Get pose of anchor robot (the one that stays still)
        anchor_pose = self.get_robot_pose(anchor_robot)
        if anchor_pose is None:
            print(f"  ERROR: Could not get anchor robot pose")
            return False

        ax, ay, az, a_yaw = anchor_pose
        print(f"  Anchor robot_{anchor_robot} at ({ax:.3f}, {ay:.3f}) yaw={math.degrees(a_yaw):.1f}deg")

        # Find direction to place the new robot
        # If anchor has existing connections, use the FREE side (opposite from connections)
        # Otherwise, use anchor's facing direction
        free_dir = self.get_free_docking_direction(anchor_robot)
        if free_dir is not None:
            dock_direction = free_dir
            print(f"  Using free side of anchor: {math.degrees(dock_direction):.1f}deg")
        else:
            dock_direction = a_yaw
            print(f"  Using anchor's facing direction: {math.degrees(dock_direction):.1f}deg")

        # Position moving robot in the dock_direction from anchor
        docking_distance = 0.105  # 10.5cm center-to-center (nearly touching)

        move_x = ax + docking_distance * math.cos(dock_direction)
        move_y = ay + docking_distance * math.sin(dock_direction)
        move_z = az

        # Moving robot faces back toward anchor (opposite of dock_direction)
        move_yaw = dock_direction + math.pi
        if move_yaw > math.pi:
            move_yaw -= 2 * math.pi

        print(f"  Moving robot_{move_robot} to ({move_x:.3f}, {move_y:.3f}) yaw={math.degrees(move_yaw):.1f}deg")

        # Move only the moving robot
        ok = self.set_robot_pose(move_robot, move_x, move_y, move_z, move_yaw)
        time.sleep(0.3)

        # Stop again to cancel any residual motion
        for _ in range(5):
            self.cmd_vel_pubs[move_robot].publish(Twist())
            time.sleep(0.02)

        return ok

    def dock_robots(self, parent_id: int, child_id: int):
        """Dock two robots together (parent -> child) with alignment."""
        if parent_id < 0 or parent_id >= self.num_robots:
            print(f"Invalid parent robot: {parent_id}")
            return
        if child_id < 0 or child_id >= self.num_robots:
            print(f"Invalid child robot: {child_id}")
            return
        if parent_id == child_id:
            print("Cannot dock robot to itself")
            return

        # Check if already connected
        if (parent_id, child_id) in self.connections:
            print(f"robot_{parent_id} already docked to robot_{child_id}")
            return

        # Align parent robot to dock with child (stops and positions)
        if not self.align_for_docking(parent_id, child_id):
            print(f"  Alignment failed, aborting dock")
            return

        # Send attach command
        cmd = (
            f'gz topic -t /attach -m gz.msgs.StringMsg -p '
            f"'data:\"[robot_{parent_id}][chassis][robot_{child_id}][chassis][attach]\"'"
        )
        print(f"Sending dock command: robot_{parent_id} -> robot_{child_id}")
        subprocess.run(cmd, shell=True, capture_output=True)

        # Track connection
        self.connections.add((parent_id, child_id))
        print(f"Connections: {self.format_connections()}")

    def undock_robots(self, parent_id: int, child_id: int):
        """Undock two robots (parent -> child)."""
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

        cmd = (
            f'gz topic -t /attach -m gz.msgs.StringMsg -p '
            f"'data:\"[robot_{parent_id}][chassis][robot_{child_id}][chassis][detach]\"'"
        )
        print(f"Undocking robot_{parent_id} -> robot_{child_id}")
        subprocess.run(cmd, shell=True, capture_output=True)

        # Remove connection
        self.connections.discard((parent_id, child_id))
        print(f"Connections: {self.format_connections()}")

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

    def test_pose_set(self):
        """Test pose setting with a fixed known position."""
        robot_id = self.selected_robot
        print(f"\n=== TEST: Moving robot_{robot_id} to (0.5, 0.5, 0.05) yaw=0 ===")
        self.stop_robot(robot_id)
        time.sleep(0.3)
        result = self.set_robot_pose(robot_id, 0.5, 0.5, 0.05, 0.0)
        print(f"=== TEST RESULT: {'SUCCESS' if result else 'FAILED'} ===\n")

    def test_pose_read_and_move(self):
        """Test reading pose and moving robot +0.2 in X direction."""
        robot_id = self.selected_robot
        print(f"\n=== TEST READ+MOVE: Reading robot_{robot_id} pose and moving +0.2 in X ===")
        self.stop_robot(robot_id)
        time.sleep(0.3)

        pose = self.get_robot_pose(robot_id, debug=True)
        if pose is None:
            print("=== TEST FAILED: Could not read pose ===\n")
            return

        x, y, z, yaw = pose
        print(f"  Current pose: ({x:.4f}, {y:.4f}, {z:.4f}) yaw={math.degrees(yaw):.1f}deg")

        new_x = x + 0.2
        print(f"  Moving to: ({new_x:.4f}, {y:.4f}, {z:.4f}) yaw={yaw:.4f}")
        result = self.set_robot_pose(robot_id, new_x, y, z, yaw)
        print(f"=== TEST RESULT: {'SUCCESS' if result else 'FAILED'} ===\n")

    def test_position_for_docking(self):
        """Position selected robot next to the target robot for docking (no attach).

        Keeps the target (child) robot still, moves only the docking (parent) robot.
        """
        parent_id = self.selected_robot
        child_id = parent_id + 1

        if child_id >= self.num_robots:
            print(f"No robot after robot_{parent_id} to dock with")
            return

        print(f"\n=== POSITION FOR DOCKING: robot_{parent_id} -> robot_{child_id} ===")

        # Stop both robots and try to cancel velocities
        print(f"  Stopping robots...")
        for _ in range(10):  # Send multiple stop commands to cancel momentum
            self.cmd_vel_pubs[parent_id].publish(Twist())
            self.cmd_vel_pubs[child_id].publish(Twist())
            time.sleep(0.02)
        time.sleep(0.3)

        # Get pose of TARGET robot (child) - this one stays still
        child_pose = self.get_robot_pose(child_id)
        if child_pose is None:
            print("  ERROR: Could not get target robot pose")
            return

        cx, cy, cz, c_yaw = child_pose
        print(f"  Target robot_{child_id} pos: ({cx:.3f}, {cy:.3f}, {cz:.3f}) yaw={math.degrees(c_yaw):.1f}deg")

        # Calculate where parent should be positioned
        # Place parent in front of child (direction child is facing), facing back toward child
        # Robots are 10cm, so center-to-center distance of 10.5cm means ~0.5cm gap
        docking_distance = 0.105  # 10.5cm center-to-center (nearly touching)

        # Position parent in front of child (in the direction child is facing)
        parent_x = cx + docking_distance * math.cos(c_yaw)
        parent_y = cy + docking_distance * math.sin(c_yaw)
        parent_z = cz

        # Parent faces back toward child (opposite direction)
        parent_yaw = c_yaw + math.pi
        # Normalize to [-π, π]
        if parent_yaw > math.pi:
            parent_yaw -= 2 * math.pi

        print(f"  Moving robot_{parent_id} to: ({parent_x:.3f}, {parent_y:.3f}, {parent_z:.3f}) yaw={math.degrees(parent_yaw):.1f}deg")
        print(f"  (robot_{child_id} stays at current position)")

        # Move only the parent robot
        self.set_robot_pose(parent_id, parent_x, parent_y, parent_z, parent_yaw)
        time.sleep(0.3)

        # Stop again to cancel any residual motion
        for _ in range(5):
            self.cmd_vel_pubs[parent_id].publish(Twist())
            time.sleep(0.02)

        print(f"=== POSITIONING COMPLETE - Press J to attach ===\n")


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
            elif key == " ":  # Space - stop selected + connected
                node.stop_robot(node.selected_robot)
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
                node.dock_robots(node.selected_robot, node.selected_robot + 1)
            elif key in ("k", "K"):  # Undock from next
                node.undock_robots(node.selected_robot, node.selected_robot + 1)
            elif key in ("l", "L"):  # Dock from previous
                node.dock_robots(node.selected_robot - 1, node.selected_robot)
            elif key == ";":  # Undock from previous
                node.undock_robots(node.selected_robot - 1, node.selected_robot)

            # Test pose setting
            elif key in ("t", "T"):
                node.test_pose_set()
            elif key in ("r", "R"):
                node.test_pose_read_and_move()
            elif key in ("p", "P"):
                node.test_position_for_docking()

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
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
