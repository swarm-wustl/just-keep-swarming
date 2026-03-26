"""
Launch the scalable N-robot docking demo using AttachablePlugin (Gazebo Harmonic).

Each robot has differential drive and can be controlled independently.
When docked, sending velocity commands to ANY robot moves the whole chain.

Requirements:
    - Gazebo Harmonic
    - ROS 2 Jazzy
    - AttachablePlugin built in workspace
    - GZ_SIM_SYSTEM_PLUGIN_PATH set to include the plugin

Usage:
    export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build/attachable_joint_plugin:$GZ_SIM_SYSTEM_PLUGIN_PATH
    ros2 launch simulation docking_demo_harmonic_launch.py

Robots start DISCONNECTED. Connect them:

    # Form a chain: robot_0 -> robot_1 -> robot_2 -> robot_3
    gz topic -t /attach -m gz.msgs.StringMsg -p 'data:"[robot_0][chassis][robot_1][chassis][attach]"'
    gz topic -t /attach -m gz.msgs.StringMsg -p 'data:"[robot_1][chassis][robot_2][chassis][attach]"'
    gz topic -t /attach -m gz.msgs.StringMsg -p 'data:"[robot_2][chassis][robot_3][chassis][attach]"'

Drive the chain (command ANY robot - all should move together):

    # Drive robot_0 forward
    gz topic -t /model/robot_0/cmd_vel -m gz.msgs.Twist -p 'linear:{x:0.3}'

    # Or drive robot_2 (middle of chain) - whole chain moves!
    gz topic -t /model/robot_2/cmd_vel -m gz.msgs.Twist -p 'linear:{x:0.3}'

    # Stop
    gz topic -t /model/robot_0/cmd_vel -m gz.msgs.Twist -p 'linear:{x:0}'

Detach:
    gz topic -t /attach -m gz.msgs.StringMsg -p 'data:"[robot_1][chassis][robot_2][chassis][detach]"'
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    """
    Generates launch description for Harmonic docking demo.
    """
    package_share_dir = get_package_share_directory("simulation")
    world_sdf_path = os.path.join(
        package_share_dir, "description", "docking_demo_harmonic.sdf"
    )

    # Harmonic uses 'gz sim' instead of 'ign gazebo'
    # Use absolute path since ros2 launch may not inherit full PATH
    run_gz_sim = ExecuteProcess(
        cmd=["/usr/bin/gz", "sim", world_sdf_path],
        output="screen",
    )

    return LaunchDescription([run_gz_sim])
