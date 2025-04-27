from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "N", default_value="6", description="Number of robots"
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                parameters=[{"dev": "/dev/input/js0"}],
                output="screen",
            ),
            Node(
                package="manual_control",
                executable="joy_to_twist",
                name="joy_to_twist",
                parameters=[{"N": LaunchConfiguration("N")}],
                output="screen",
            ),
        ]
    )
