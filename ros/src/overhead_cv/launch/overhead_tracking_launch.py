import launch_ros.actions

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return launch.LaunchDescription(
        [
            # Declare launch arguments
            DeclareLaunchArgument(
                "N", default_value="2", description="Number of robots"
            ),
            DeclareLaunchArgument(
                "cam_input", default_value="0", description="Camera input index"
            ),
            # Camera Feed Node
            launch_ros.actions.Node(
                package="overhead_cv",
                executable="camera_feed",
                name="camera_feed",
                parameters=[
                    {
                        "delay": 0.002,
                        "focal_length": 0.013387,
                        "cam_height": 1.2573,
                        "fov_x": 130.0,
                        "fov_y": 103.0,
                        "resolution_x": 1920.0,
                        "resolution_y": 1080.0,
                        "cam_input": LaunchConfiguration("cam_input"),
                    }
                ],
                output="screen",
            ),
            # Robot Tracker Node
            launch_ros.actions.Node(
                package="overhead_cv",
                executable="robot_tracker",
                name="robot_tracker",
                parameters=[{"display": True}],
                output="screen",
            ),
            # Position Estimator Node
            launch_ros.actions.Node(
                package="overhead_cv",
                executable="position_estimator",
                name="position_estimator",
                parameters=[
                    {"N": LaunchConfiguration("N"), "q": 1.0, "r": 5.0}
                ],  # Require tuning
                output="screen",
            ),
            # # Map Maker Node
            # launch_ros.actions.Node(
            #     package="map_maker",
            #     executable="map_maker",
            #     name="map_maker",
            #     parameters=[{"resolution": 0.5}],
            #     output="screen",
            # ),
        ]
    )
