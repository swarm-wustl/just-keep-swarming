"""
Launch the docking demo with ROS 2 integration.

Launches:
    - Gazebo Harmonic with the docking demo world
    - ros_gz_bridge for cmd_vel and odometry topics

Requirements:
    - Gazebo Harmonic
    - ROS 2 Jazzy
    - ros_gz_bridge
    - AttachablePlugin built in workspace
    - GZ_SIM_SYSTEM_PLUGIN_PATH set to include the plugin

Usage:
    # Terminal 1: Launch simulation
    export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build/attachable_joint_plugin:$GZ_SIM_SYSTEM_PLUGIN_PATH
    ros2 launch simulation docking_demo_harmonic_launch.py

    # Terminal 2: Run manual control
    ros2 run simulation manual_control

Controls (in the manual_control terminal):
    W/Up:     Forward
    S/Down:   Backward
    A/Left:   Turn left
    D/Right:  Turn right
    1-4:      Select robot 0-3
    J:        Dock selected -> next robot
    K:        Undock selected -> next robot
    Space:    Stop selected robot
    X:        Stop all robots
    Q:        Quit
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generates launch description for Harmonic docking demo with ROS 2 integration.
    """
    package_share_dir = get_package_share_directory("simulation")
    world_sdf_path = os.path.join(
        package_share_dir, "description", "docking_demo_harmonic.sdf"
    )

    # Launch Gazebo Harmonic
    run_gz_sim = ExecuteProcess(
        cmd=["/usr/bin/gz", "sim", world_sdf_path],
        output="screen",
    )

    # Bridge configuration for 4 robots
    bridge_args = []
    for i in range(4):
        # cmd_vel: ROS 2 -> Gazebo
        bridge_args.append(
            f"/model/robot_{i}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"
        )
        # odometry: Gazebo -> ROS 2
        bridge_args.append(
            f"/model/robot_{i}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"
        )

    # ros_gz_bridge node
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=bridge_args,
        output="screen",
    )

    return LaunchDescription([
        run_gz_sim,
        ros_gz_bridge,
    ])
