# SWARM

# Install

## Pre-Commit Linters

This repository uses a pre-commit hook to validate code quality. First, install the linters with

```
./install_linters.sh
```

Then change the Git hooks path:

```
git config core.hooksPath .githooks
```

Committing changes will now automatically trigger the linting process.

## Gazebo

Gazebo is the physics simulator used by Swarm. The `simulation` package requires that it be installed. To install Gazebo and the bridge for ROS2 Humble, run:

```
sudo apt-get install ros-humble-ros-ign-bridge ros-humble-ros-gz
```

# Build

`cd` into the `ros/` directory, then run:

```
colcon build
```

# Run

For information on how to run packages, see the `README.md` of each package.

## Overhead CV robot tracking README

The overheadCV component of this project can be found in the overhead_cv package