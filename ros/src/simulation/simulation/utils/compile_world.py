import os
from datetime import datetime
from math import ceil, cos, pi, sin, sqrt
from typing import Any, Dict, List, Literal

import yaml

from simulation.utils.compile_sdf import compile_sdf_file


def load_yaml(filename: str) -> Dict[str, Any]:
    try:
        with open(filename, "r") as stream:
            return yaml.safe_load(stream)
    except yaml.YAMLError as e:
        raise ValueError(f"Improperly formatted YAML file {filename}: {e}")
    except IOError as e:
        raise IOError(f"Error reading file {filename}: {e}")


def generate_robot_poses(
    n_robots: int, offset: float, arrangement: Literal["LINE", "GRID", "CIRCLE"]
) -> List[str]:
    match arrangement:
        case "LINE":
            return [f"{n * offset} 0 0 0 0 0" for n in range(n_robots)]
        case "GRID":
            sq_size = ceil(sqrt(n_robots))
            return [
                f"{n % sq_size * offset} {n // sq_size} 0 0 0 0"
                for n in range(n_robots)
            ]
        case "CIRCLE":
            deg_per_robot = 2 * pi / n_robots
            r = offset / deg_per_robot
            return [
                f"{r*cos(n*deg_per_robot)} {r*sin(n*deg_per_robot)} 0 0 0 0"
                for n in range(n_robots)
            ]


def compile_robot_world(**kwargs) -> None:
    required_params = [
        "robot_sdf_filename",
        "world_sdf_filename",
        "params_yaml_filename",
        "output_filename",
        "n_robots",
        "robot_offset",
        "robot_arrangement",
    ]

    for param in required_params:
        if param not in kwargs:
            raise ValueError(f"Missing required parameter: {param}")

    robot_sdf_filename = kwargs["robot_sdf_filename"]
    world_sdf_filename = kwargs["world_sdf_filename"]
    params_yaml_filename = kwargs["params_yaml_filename"]
    output_filename = kwargs["output_filename"]
    n_robots = kwargs.get("n_robots", 1)
    robot_offset = kwargs.get("robot_offset", 0.5)
    robot_arrangement = kwargs.get("robot_arrangement", "LINE")

    params = load_yaml(params_yaml_filename)
    wheel_y_offset = params.get("WHEEL_Y_OFFSET")
    if not isinstance(wheel_y_offset, float):
        raise ValueError("WHEEL_Y_OFFSET must be specified")
    wheel_separation = wheel_y_offset * 2

    poses = generate_robot_poses(n_robots, robot_offset, robot_arrangement)
    compiled_robots = [
        compile_sdf_file(
            robot_sdf_filename,
            {
                **params,
                "ROBOT_NUM": n,
                "POSE": poses[n],
                "WHEEL_SEPARATION": wheel_separation,
            },
        )
        for n in range(n_robots)
    ]

    robots = "\n".join(compiled_robots)
    compiled_world = compile_sdf_file(world_sdf_filename, {"ROBOTS": robots})

    preamble = f"""<?xml version="1.0" ?>
<!--
    This file was generated automatically on {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} by {os.path.basename(__file__)}
    Manual edits will be overwritten
-->
"""
    compiled_world = preamble + compiled_world

    try:
        with open(output_filename, "w") as f:
            f.write(compiled_world)
    except IOError as e:
        raise IOError(f"Error writing to file {output_filename}: {e}")


if __name__ == "__main__":
    compile_robot_world(
        robot_sdf_filename="../description/robot.template.sdf",
        world_sdf_filename="../description/world.template.sdf",
        params_yaml_filename="../description/robot_params.yaml",
        output_filename="../description/world.generated.sdf",
        n_robots=50,
        robot_offset=0.5,
        robot_arrangement="CIRCLE",
    )
