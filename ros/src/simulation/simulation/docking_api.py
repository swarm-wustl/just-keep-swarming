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
        """
        Teleport a robot to the specified pose.

        Args:
            robot_id: The robot to move
            x, y, z: Target position
            yaw: Target orientation (radians)
            verify: If True, read back pose and verify within tolerance

        Returns:
            True if pose was set successfully
        """
        qz = round(math.sin(yaw / 2.0), 6)
        qw = round(math.cos(yaw / 2.0), 6)

        req = f'name: "robot_{robot_id}", position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, orientation: {{x: 0, y: 0, z: {qz}, w: {qw}}}'
        cmd = f"gz service -s /world/docking_demo/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 --req '{req}'"

        if os.system(cmd) != 0:
            return False

        if verify:
            time.sleep(0.1)
            actual = self.get_robot_pose(robot_id)
            if actual is None:
                return False
            ax, ay, _, _ = actual
            pos_error = math.sqrt((ax - x) ** 2 + (ay - y) ** 2)
            if pos_error > 0.05:
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

    def align_for_docking(self, parent_id: int, child_id: int) -> bool:
        """
        Align robots for docking by teleporting the unconnected robot to the connected one.

        If one robot is already in a chain, the free robot is moved to it.
        If neither is in a chain, the parent is moved to the child.
        """
        parent_connected = len(self.get_connected_robots(parent_id)) > 1
        child_connected = len(self.get_connected_robots(child_id)) > 1

        if parent_connected and child_connected:
            return True

        if parent_connected and not child_connected:
            move_robot = child_id
            anchor_robot = parent_id
        else:
            move_robot = parent_id
            anchor_robot = child_id

        anchor_pose = self.get_robot_pose(anchor_robot)
        if anchor_pose is None:
            return False

        ax, ay, az, a_yaw = anchor_pose

        free_dir = self.get_free_docking_direction(anchor_robot)
        if free_dir is not None:
            dock_direction = free_dir
        else:
            dock_direction = a_yaw + math.pi
            if dock_direction > math.pi:
                dock_direction -= 2 * math.pi

        docking_distance = 0.105
        move_x = ax + docking_distance * math.cos(dock_direction)
        move_y = ay + docking_distance * math.sin(dock_direction)
        move_z = az
        move_yaw = a_yaw

        return self.set_robot_pose(move_robot, move_x, move_y, move_z, move_yaw, verify=False)

    def dock(self, parent_id: int, child_id: int, auto_align: bool = True) -> bool:
        """
        Dock two robots together using a fixed joint.

        The simulation is paused during docking to prevent physics momentum from
        causing misalignment between the teleport and joint creation.

        Args:
            parent_id: The parent robot in the connection
            child_id: The child robot in the connection
            auto_align: If True, automatically teleport robots into alignment

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
            return True

        self.stop_robot(parent_id)
        self.stop_robot(child_id)

        self.pause_sim()
        try:
            if auto_align and not self.align_for_docking(parent_id, child_id):
                return False

            cmd = (
                f'gz topic -t /attach -m gz.msgs.StringMsg -p '
                f"'data:\"[robot_{parent_id}][chassis][robot_{child_id}][chassis][attach]\"'"
            )
            os.system(cmd)
            time.sleep(0.1)
        finally:
            self.unpause_sim()

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

        cmd = (
            f'gz topic -t /attach -m gz.msgs.StringMsg -p '
            f"'data:\"[robot_{parent_id}][chassis][robot_{child_id}][chassis][detach]\"'"
        )
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
        """Pause the Gazebo simulation."""
        cmd = "gz service -s /world/docking_demo/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 2000 --req 'pause: true'"
        return os.system(cmd) == 0

    def unpause_sim(self) -> bool:
        """Resume the Gazebo simulation."""
        cmd = "gz service -s /world/docking_demo/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 2000 --req 'pause: false'"
        return os.system(cmd) == 0
