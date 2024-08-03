import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import launch
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.actions.execute_local import LaunchConfiguration
from simulation.utils.compile_world import compile_robot_world


def generate_launch_description():
    n_robots_arg = DeclareLaunchArgument(
        "n_robots", default_value="50", description="Number of robots (#)"
    )
    robot_offset_arg = DeclareLaunchArgument(
        "robot_offset", default_value="0.5", description="Offset between robots (m)"
    )
    robot_arrangement_arg = DeclareLaunchArgument(
        "robot_arrangement",
        default_value="CIRCLE",
        description="Arrangement of robots (LINE, GRID, CIRCLE)",
    )
    n_robots = LaunchConfiguration("n_robots")
    robot_offset = LaunchConfiguration("robot_offset")
    robot_arrangement = LaunchConfiguration("robot_arrangement")

    package_share_dir = get_package_share_directory("simulation")

    robot_sdf_filename = os.path.join(
        package_share_dir, "description", "robot.template.sdf"
    )
    world_sdf_filename = os.path.join(
        package_share_dir, "description", "world.template.sdf"
    )
    params_yaml_filename = os.path.join(
        package_share_dir, "description", "robot_params.yaml"
    )
    compiled_world_filename = os.path.join(
        package_share_dir, "description", "world.generated.sdf"
    )

    def compile_world(context):
        compile_robot_world(
            robot_sdf_filename=robot_sdf_filename,
            world_sdf_filename=world_sdf_filename,
            params_yaml_filename=params_yaml_filename,
            output_filename=compiled_world_filename,
            n_robots=int(context.perform_substitution(n_robots)),
            robot_offset=float(context.perform_substitution(robot_offset)),
            robot_arrangement=context.perform_substitution(robot_arrangement),
        )

    def generate_topic_bridges(context):
        n_robots_value = int(context.perform_substitution(n_robots))
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

    def init_random_control(context):
        n_robots_value = int(context.perform_substitution(n_robots))
        return [
            Node(
                package="demo_control",
                executable="random_control",
                name="random_control",
                parameters=[
                    {
                        "num_robots": n_robots_value,
                    }
                ],
            )
        ]

    compile_world_action = OpaqueFunction(function=compile_world)
    topic_bridges_action = OpaqueFunction(function=generate_topic_bridges)
    init_random_control_action = OpaqueFunction(function=init_random_control)
    run_ign_gazebo = ExecuteProcess(cmd=["ign", "gazebo", compiled_world_filename])

    return launch.LaunchDescription(
        [
            n_robots_arg,
            robot_offset_arg,
            robot_arrangement_arg,
            compile_world_action,
            run_ign_gazebo,
            topic_bridges_action,
            init_random_control_action,
        ]
    )
