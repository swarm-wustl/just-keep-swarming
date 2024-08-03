import os
import tempfile
from unittest.mock import mock_open, patch

import pytest

from simulation.utils.compile_world import (
    compile_robot_world,
    compile_robots,
    generate_preamble,
    generate_robot_poses,
    load_yaml,
    write_compiled_world,
)


# Test for load_yaml
def test_load_yaml_success():
    yaml_content = "key: value"
    with patch("builtins.open", mock_open(read_data=yaml_content)):
        result = load_yaml("dummy.yaml")
    assert result == {"key": "value"}


def test_load_yaml_format_error():
    with patch("builtins.open", mock_open(read_data="!!invalid")):
        with pytest.raises(ValueError, match="Improperly formatted YAML file"):
            load_yaml("dummy.yaml")


def test_load_yaml_io_error():
    with patch("builtins.open", side_effect=IOError):
        with pytest.raises(IOError, match="Error reading file dummy.yaml"):
            load_yaml("dummy.yaml")


# Test for generate_robot_poses
def test_generate_robot_poses_line():
    result = generate_robot_poses(3, 1.0, "LINE")
    assert result == ["0.0 0 0 0 0 0", "1.0 0 0 0 0 0", "2.0 0 0 0 0 0"]


def test_generate_robot_poses_grid():
    result = generate_robot_poses(4, 1.0, "GRID")
    assert result == [
        "0.0 0 0 0 0 0",
        "1.0 0 0 0 0 0",
        "0.0 1 0 0 0 0",
        "1.0 1 0 0 0 0",
    ]


def test_generate_robot_poses_circle():
    result = generate_robot_poses(4, 1.0, "CIRCLE")
    assert len(result) == 4  # Check if 4 poses are generated
    assert all(
        pose.split(" ")[0] != "0" and pose.split(" ")[1] != "0" for pose in result
    )  # All poses should have 0 in Z R P Y


# Test for generate_preamble
def test_generate_preamble():
    preamble = generate_preamble()
    assert '<?xml version="1.0" ?>' in preamble
    assert "This file was generated automatically" in preamble


# Test for write_compiled_world
def test_write_compiled_world_success():
    with tempfile.NamedTemporaryFile(delete=False) as temp_file:
        output_filename = temp_file.name
    try:
        write_compiled_world(output_filename, "compiled world content")
        with open(output_filename, "r") as f:
            assert f.read() == "compiled world content"
    finally:
        os.remove(output_filename)


def test_write_compiled_world_io_error():
    with patch("builtins.open", side_effect=IOError):
        with pytest.raises(IOError, match="Error writing to file"):
            write_compiled_world("dummy_output.sdf", "compiled world content")


# Test for compile_robots
def test_compile_robots():
    # Mock the compile_sdf_file function
    with patch(
        "simulation.utils.compile_sdf.compile_sdf_file"
    ) as mock_compile_sdf_file, tempfile.NamedTemporaryFile(delete=False) as robot_sdf:
        robot_sdf.write(
            b"robot {% POSE %} {% ROBOT_NUM %} {% WHEEL_SEPARATION %} mass:{% WHEEL_MASS %}"
        )
        robot_sdf.close()
        mock_compile_sdf_file.return_value = "compiled_robot"
        try:
            result = compile_robots(
                params={"WHEEL_MASS": 0.1},
                wheel_separation=0.5,
                poses=["0 0 0 0 0 0"],
                n_robots=1,
                robot_sdf_filename=robot_sdf.name,
            )
        finally:
            os.remove(robot_sdf.name)
    assert "robot" in result
    assert "mass:0.1" in result


# Test for compile_robot_world
def test_compile_robot_world_success():
    with tempfile.NamedTemporaryFile(
        delete=False
    ) as robot_sdf, tempfile.NamedTemporaryFile(
        delete=False
    ) as world_sdf, tempfile.NamedTemporaryFile(
        delete=False
    ) as params_yaml, tempfile.NamedTemporaryFile(
        delete=False
    ) as output_sdf:
        robot_sdf_name = robot_sdf.name
        world_sdf_name = world_sdf.name
        params_yaml_name = params_yaml.name
        output_sdf_name = output_sdf.name

        # Write example content to the files
        robot_sdf.write(
            b"robot template {% POSE %} {% WHEEL_Y_OFFSET %} {% ROBOT_NUM %} {% WHEEL_SEPARATION %}"
        )
        world_sdf.write(b"world template {% ROBOTS %}")
        params_yaml.write(b"WHEEL_Y_OFFSET: 0.2")

        robot_sdf.close()
        world_sdf.close()
        params_yaml.close()
        output_sdf.close()

    try:
        compile_robot_world(
            robot_sdf_filename=robot_sdf_name,
            world_sdf_filename=world_sdf_name,
            params_yaml_filename=params_yaml_name,
            output_filename=output_sdf_name,
            n_robots=2,
            robot_offset=1.0,
            robot_arrangement="LINE",
        )

        with open(output_sdf_name, "r") as f:
            content = f.read()
            assert "This file was generated automatically" in content
            assert "robot template" in content

    finally:
        os.remove(robot_sdf_name)
        os.remove(world_sdf_name)
        os.remove(params_yaml_name)
        os.remove(output_sdf_name)


def test_compile_robot_world_missing_param():
    with pytest.raises(
        ValueError, match="Missing required parameter: robot_sdf_filename"
    ):
        compile_robot_world(
            world_sdf_filename="world.sdf",
            params_yaml_filename="params.yaml",
            output_filename="output.sdf",
            n_robots=1,
            robot_offset=0.5,
            robot_arrangement="LINE",
        )
