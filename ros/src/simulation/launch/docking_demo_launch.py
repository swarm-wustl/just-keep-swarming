"""
Launch the scalable N-robot docking demo simulation.

Demonstrates runtime docking/undocking between modular robots using
the DetachableJoint system plugin. Each robot has 4 docking ports
(front, back, left, right) with independent attach/detach control.

Initial configuration (T-shape):
    robot_0 (red) --front--> robot_1 (green) --front--> robot_2 (blue)
                                    |
                                   left
                                    |
                                    v
                               robot_3 (yellow)

Usage:
    ros2 launch simulation docking_demo_launch.py

Topic naming convention:
    /robot_{i}/dock_{port}/attach   - Attach joint
    /robot_{i}/dock_{port}/detach   - Detach joint
    /robot_{i}/dock_{port}/state    - Joint state

Example commands (run in separate terminal):
    # Detach robot_0 from robot_1
    ign topic -t /robot_0/dock_front/detach -m ignition.msgs.Empty -p ''

    # Reattach robot_0 to robot_1 (must be adjacent)
    ign topic -t /robot_0/dock_front/attach -m ignition.msgs.Empty -p ''

    # Detach robot_1 from robot_2
    ign topic -t /robot_1/dock_front/detach -m ignition.msgs.Empty -p ''

    # Detach robot_3 from robot_1 (side branch)
    ign topic -t /robot_1/dock_left/detach -m ignition.msgs.Empty -p ''

    # List all dock state topics
    ign topic -l | grep dock
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    """
    Generates launch description for scalable docking demo.
    """
    package_share_dir = get_package_share_directory("simulation")
    world_sdf_path = os.path.join(
        package_share_dir, "description", "docking_demo.sdf"
    )

    run_ign_gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", world_sdf_path],
        output="screen",
    )

    return LaunchDescription([run_ign_gazebo])
