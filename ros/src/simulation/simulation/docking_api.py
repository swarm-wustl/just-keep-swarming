#!/usr/bin/env python3
"""
Programmatic API for robot docking operations.

Provides functions to dock and undock robots with automatic alignment.
"""

import json
import math
import os
import subprocess
import time
from typing import Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DockingController(Node):
    """Controller for docking operations between modular robots."""

    def __init__(self, num_robots: int = 4):
        super().__init__("docking_controller")

        self.num_robots = num_robots
        self.connections: Set[tuple] = set()

        # Create publishers for each robot
        self.cmd_vel_pubs = []
        for i in range(self.num_robots):
            pub = self.create_publisher(Twist, f"/model/robot_{i}/cmd_vel", 10)
            self.cmd_vel_pubs.append(pub)

        self.get_logger().info(f"Docking controller initialized with {num_robots} robots")

    def get_robot_pose(self, robot_id: int) -> Optional[Tuple[float, float, float, float]]:
        """Get robot pose (x, y, z, yaw) from Gazebo."""
        try:
            cmd = (
                f'gz topic -e -t /world/docking_demo/dynamic_pose/info -n 1 '
                f'--json-output 2>/dev/null'
            )
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=3)

            if result.returncode == 0 and result.stdout.strip():
                first_line = result.stdout.strip().split('\n')[0]
                data = json.loads(first_line)

                robot_name = f"robot_{robot_id}"
                for pose in data.get("pose", []):
                    if pose.get("name") == robot_name:
                        pos = pose.get("position", {})
                        ori = pose.get("orientation", {})

                        x = pos.get("x", 0.0)
                        y = pos.get("y", 0.0)
                        z = pos.get("z", 0.0)

                        qx = ori.get("x", 0.0)
                        qy = ori.get("y", 0.0)
                        qz = ori.get("z", 0.0)
                        qw = ori.get("w", 1.0)

                        siny_cosp = 2.0 * (qw * qz + qx * qy)
                        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                        yaw = math.atan2(siny_cosp, cosy_cosp)

                        return (x, y, z, yaw)

        except Exception as e:
            self.get_logger().warn(f"Failed to get pose for robot_{robot_id}: {e}")
        return None

    def set_robot_pose(self, robot_id: int, x: float, y: float, z: float, yaw: float, verify: bool = True) -> bool:
        """Set robot pose in Gazebo with optional verification."""
        x = round(x, 3)
        y = round(y, 3)
        z = round(z, 3)

        qz = round(math.sin(yaw / 2.0), 6)
        qw = round(math.cos(yaw / 2.0), 6)

        req = f'name: "robot_{robot_id}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: 0, y: 0, z: {qz}, w: {qw}}}'
        cmd = f"gz service -s /world/docking_demo/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 --req '{req}'"

        ret = os.system(cmd)
        time.sleep(0.15)

        if ret != 0:
            return False

        if verify:
            # Verify pose was set correctly (within tolerance)
            time.sleep(0.1)
            actual = self.get_robot_pose(robot_id)
            if actual is None:
                return False
            ax, ay, _, ayaw = actual
            pos_error = math.sqrt((ax - x) ** 2 + (ay - y) ** 2)
            if pos_error > 0.05:  # 5cm tolerance
                self.get_logger().warn(f"Pose verification failed: expected ({x:.3f}, {y:.3f}), got ({ax:.3f}, {ay:.3f})")
                return False

        return True

    def send_velocity(self, robot_id: int, linear: float, angular: float):
        """Send velocity command to a robot."""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pubs[robot_id].publish(msg)

    def stop_robot(self, robot_id: int):
        """Stop a robot."""
        msg = Twist()
        for _ in range(5):
            self.cmd_vel_pubs[robot_id].publish(msg)
            time.sleep(0.02)

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

    def get_free_docking_direction(self, robot_id: int) -> Optional[float]:
        """Get the direction (yaw) where robot has no existing connection."""
        connected = self.get_connected_robots(robot_id)
        if len(connected) <= 1:
            return None

        my_pose = self.get_robot_pose(robot_id)
        if my_pose is None:
            return None
        mx, my_y, _, _ = my_pose

        for other_id in connected:
            if other_id == robot_id:
                continue
            other_pose = self.get_robot_pose(other_id)
            if other_pose is None:
                continue
            ox, oy, _, _ = other_pose

            to_other = math.atan2(oy - my_y, ox - mx)
            free_dir = to_other + math.pi
            if free_dir > math.pi:
                free_dir -= 2 * math.pi
            return free_dir

        return None

    def align_for_docking(self, parent_id: int, child_id: int, verbose: bool = True) -> bool:
        """Align robots for docking. Moves the unconnected robot to the connected one."""
        if verbose:
            print(f"  Aligning robot_{parent_id} to dock with robot_{child_id}...")

        # Stop both robots and try to cancel velocities (matches working code)
        for _ in range(10):
            self.cmd_vel_pubs[parent_id].publish(Twist())
            self.cmd_vel_pubs[child_id].publish(Twist())
            time.sleep(0.02)
        time.sleep(0.3)

        # Check which robot is in a chain
        parent_connected = len(self.get_connected_robots(parent_id)) > 1
        child_connected = len(self.get_connected_robots(child_id)) > 1

        # If both in chains, skip repositioning
        if parent_connected and child_connected:
            if verbose:
                print("  Both robots in chains - skipping repositioning")
            return True

        # Decide which robot to move
        if parent_connected and not child_connected:
            move_robot = child_id
            anchor_robot = parent_id
            if verbose:
                print(f"  robot_{parent_id} is in a chain, moving robot_{child_id} to it")
        else:
            move_robot = parent_id
            anchor_robot = child_id
            if child_connected and verbose:
                print(f"  robot_{child_id} is in a chain, moving robot_{parent_id} to it")

        # Get anchor pose
        anchor_pose = self.get_robot_pose(anchor_robot)
        if anchor_pose is None:
            if verbose:
                print(f"  ERROR: Could not get pose for robot_{anchor_robot}")
            return False

        ax, ay, az, a_yaw = anchor_pose
        if verbose:
            print(f"  Anchor robot_{anchor_robot} at ({ax:.3f}, {ay:.3f}) yaw={math.degrees(a_yaw):.1f}deg")

        # Find docking direction
        free_dir = self.get_free_docking_direction(anchor_robot)
        if free_dir is not None:
            dock_direction = free_dir
            if verbose:
                print(f"  Using free side of anchor: {math.degrees(dock_direction):.1f}deg")
        else:
            dock_direction = a_yaw + math.pi
            if dock_direction > math.pi:
                dock_direction -= 2 * math.pi
            if verbose:
                print(f"  Placing behind anchor: {math.degrees(dock_direction):.1f}deg")

        # Position moving robot behind anchor
        docking_distance = 0.105
        move_x = ax + docking_distance * math.cos(dock_direction)
        move_y = ay + docking_distance * math.sin(dock_direction)
        move_z = az
        move_yaw = a_yaw  # Same direction as anchor (front-to-back)

        if verbose:
            print(f"  Moving robot_{move_robot} to ({move_x:.3f}, {move_y:.3f}) yaw={math.degrees(move_yaw):.1f}deg")

        # Move only the moving robot (matches working code exactly)
        ok = self.set_robot_pose(move_robot, move_x, move_y, move_z, move_yaw, verify=False)
        time.sleep(0.3)

        # Stop again to cancel any residual motion
        for _ in range(5):
            self.cmd_vel_pubs[move_robot].publish(Twist())
            time.sleep(0.02)

        if verbose and ok:
            final_pose = self.get_robot_pose(move_robot)
            if final_pose:
                fx, fy, _, fyaw = final_pose
                print(f"  Final position: ({fx:.3f}, {fy:.3f}) yaw={math.degrees(fyaw):.1f}deg")

        return ok

    def dock(self, parent_id: int, child_id: int, auto_align: bool = True) -> bool:
        """
        Dock two robots together.

        Args:
            parent_id: The robot initiating the dock
            child_id: The robot being docked to
            auto_align: If True, automatically align robots before docking

        Returns:
            True if docking succeeded
        """
        if parent_id < 0 or parent_id >= self.num_robots:
            self.get_logger().error(f"Invalid parent robot: {parent_id}")
            return False
        if child_id < 0 or child_id >= self.num_robots:
            self.get_logger().error(f"Invalid child robot: {child_id}")
            return False
        if parent_id == child_id:
            self.get_logger().error("Cannot dock robot to itself")
            return False
        if (parent_id, child_id) in self.connections:
            self.get_logger().warn(f"robot_{parent_id} already docked to robot_{child_id}")
            return True

        # 1. Stop sending cmd_vel commands
        self.stop_robot(parent_id)
        self.stop_robot(child_id)
        time.sleep(0.1)

        # 2. Freeze the world to prevent collision repulsions
        self.pause_sim()
        
        try:
            # 3. Teleport into perfect alignment while physics is suspended
            if auto_align:
                if not self.align_for_docking(parent_id, child_id, verbose=True):
                    print("  Alignment failed, aborting dock")
                    self.unpause_sim()
                    return False

            # 4. Lock the joint. Since physics is paused, no micro-movements can occur
            cmd = (
                f'gz topic -t /attach -m gz.msgs.StringMsg -p '
                f"'data:\"[robot_{parent_id}][chassis][robot_{child_id}][chassis][attach]\"'"
            )
            print(f"Sending dock command: robot_{parent_id} -> robot_{child_id}")
            os.system(cmd)
            
            # Short sleep to ensure the Ignition transport layer processes the message
            time.sleep(0.2) 

        finally:
            # 5. Safely unpause the world
            self.unpause_sim()

        # Track connection
        self.connections.add((parent_id, child_id))
        return True

    def undock(self, parent_id: int, child_id: int) -> bool:
        """
        Undock two robots.

        Args:
            parent_id: The parent robot in the connection
            child_id: The child robot in the connection

        Returns:
            True if undocking succeeded
        """
        if (parent_id, child_id) not in self.connections:
            self.get_logger().warn(f"No connection between robot_{parent_id} and robot_{child_id}")
            return False

        # Stop robots
        self.stop_robot(parent_id)
        self.stop_robot(child_id)
        time.sleep(0.1)

        # Send detach command
        cmd = (
            f'gz topic -t /attach -m gz.msgs.StringMsg -p '
            f"'data:\"[robot_{parent_id}][chassis][robot_{child_id}][chassis][detach]\"'"
        )
        print(f"Undocking robot_{parent_id} -> robot_{child_id}")
        os.system(cmd)

        # Remove connection
        self.connections.discard((parent_id, child_id))
        return True

    def drive(self, robot_id: int, linear: float, angular: float, duration: float):
        """
        Drive a robot for a specified duration.

        Args:
            robot_id: Robot to drive
            linear: Linear velocity (m/s)
            angular: Angular velocity (rad/s)
            duration: Duration in seconds
        """
        # Get all connected robots to send same velocity
        connected = self.get_connected_robots(robot_id)

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular

        start_time = time.time()
        while time.time() - start_time < duration:
            for rid in connected:
                self.cmd_vel_pubs[rid].publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        # Stop
        stop_msg = Twist()
        for _ in range(5):
            for rid in connected:
                self.cmd_vel_pubs[rid].publish(stop_msg)
            time.sleep(0.02)

    def pause_sim(self) -> bool:
        """Pause the Gazebo physics engine."""
        cmd = "gz service -s /world/docking_demo/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 2000 --req 'pause: true'"
        print(f"!!!!!! Pausing Sim: {cmd}")
        return os.system(cmd) == 0

    def unpause_sim(self) -> bool:
        """Resume the Gazebo physics engine."""
        cmd = "gz service -s /world/docking_demo/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 2000 --req 'pause: false'"
        return os.system(cmd) == 0
