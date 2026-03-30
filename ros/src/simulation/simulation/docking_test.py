#!/usr/bin/env python3
"""
Simple test node for the docking API.

Drives robot 0 forward for half a second, stops, and docks to robot 1.
"""

import rclpy
from simulation.docking_api import DockingController
import time


def main():
    rclpy.init()

    # Create docking controller
    controller = DockingController(num_robots=4)
    controller.get_logger().info("Docking test starting...")

    # Drive robot 0 forward briefly
    controller.get_logger().info("Driving robot_0 forward...")
    controller.drive(robot_id=0, linear=0.3, angular=0.0, duration=0.5)

    # Stop both robots and let physics settle with ROS spinning
    controller.get_logger().info("Stopping robots and letting physics settle...")
    for _ in range(10):
        controller.stop_robot(0)
        controller.stop_robot(1)
        rclpy.spin_once(controller, timeout_sec=0.02)

    # Dock robot 0 to robot 1
    controller.get_logger().info("Docking robot_0 to robot_1...")
    success = controller.dock(parent_id=0, child_id=1, auto_align=True)

    if success:
        controller.get_logger().info("Docking successful!")
    else:
        controller.get_logger().error("Docking failed!")

    # Keep node alive briefly to see result
    time.sleep(1.0)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
