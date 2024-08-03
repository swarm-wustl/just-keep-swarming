"""
Launch the Gazebo simulation
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.actions.execute_local import LaunchConfiguration
from simulation.utils.compile_world import compile_robot_world


def generate_launch_description():
    """
    Generates launch description for simulator launch
    """

    def get_launch_arguments():
        return [
            DeclareLaunchArgument(
                "n_robots", default_value="50", description="Number of robots (#)"
            ),
            DeclareLaunchArgument(
                "robot_offset",
                default_value="0.5",
                description="Offset between robots (m)",
            ),
            DeclareLaunchArgument(
                "robot_arrangement",
                default_value="CIRCLE",
                description="Arrangement of robots (LINE, GRID, CIRCLE)",
            ),
        ]

    def get_launch_configurations():
        return {
            "n_robots": LaunchConfiguration("n_robots"),
            "robot_offset": LaunchConfiguration("robot_offset"),
            "robot_arrangement": LaunchConfiguration("robot_arrangement"),
        }

    def get_filenames(package_share_dir):
        return {
            "robot_sdf_filename": os.path.join(
                package_share_dir, "description", "robot.template.sdf"
            ),
            "world_sdf_filename": os.path.join(
                package_share_dir, "description", "world.template.sdf"
            ),
            "params_yaml_filename": os.path.join(
                package_share_dir, "description", "robot_params.yaml"
            ),
            "compiled_world_filename": os.path.join(
                package_share_dir, "description", "world.generated.sdf"
            ),
        }

    def compile_world(context, filenames, configs):
        compile_robot_world(
            robot_sdf_filename=filenames["robot_sdf_filename"],
            world_sdf_filename=filenames["world_sdf_filename"],
            params_yaml_filename=filenames["params_yaml_filename"],
            output_filename=filenames["compiled_world_filename"],
            n_robots=int(context.perform_substitution(configs["n_robots"])),
            robot_offset=float(context.perform_substitution(configs["robot_offset"])),
            robot_arrangement=context.perform_substitution(
                configs["robot_arrangement"]
            ),
        )

    def generate_topic_bridges(context, configs):
        n_robots_value = int(context.perform_substitution(configs["n_robots"]))
        return [
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "ros_gz_bridge",
                    "parameter_bridge",
                    f"/model/robot_{n}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                ]
            )
            for n in range(n_robots_value)
        ]

    def init_random_control(context, configs):
        n_robots_value = int(context.perform_substitution(configs["n_robots"]))
        return [
            Node(
                package="demo_control",
                executable="random_control",
                name="random_control",
                parameters=[{"num_robots": n_robots_value}],
            )
        ]

    package_share_dir = get_package_share_directory("simulation")
    filenames = get_filenames(package_share_dir)
    configs = get_launch_configurations()

    compile_world_action = OpaqueFunction(
        function=lambda context: compile_world(context, filenames, configs)
    )
    topic_bridges_action = OpaqueFunction(
        function=lambda context: generate_topic_bridges(context, configs)
    )
    init_random_control_action = OpaqueFunction(
        function=lambda context: init_random_control(context, configs)
    )
    run_ign_gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", filenames["compiled_world_filename"]]
    )

    return LaunchDescription(
        get_launch_arguments()
        + [
            compile_world_action,
            run_ign_gazebo,
            topic_bridges_action,
            init_random_control_action,
        ]
    )
