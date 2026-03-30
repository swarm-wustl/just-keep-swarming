# Simulation

This package provides Gazebo simulation environments for modular robots with runtime docking capabilities.

## Requirements

- **Gazebo Harmonic** (gz-sim8)
- **ROS 2 Jazzy**
- **ros_gz_bridge**
- **AttachablePlugin** (included in workspace)

## Setup

The `AttachablePlugin` must be in Gazebo's plugin path. Before launching, run:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build/attachable_joint_plugin:$GZ_SIM_SYSTEM_PLUGIN_PATH
```

You can add this to your shell profile or source it before each session.

## Modular Robot Docking Demo

The primary simulation demonstrates modular robots that can dock and undock at runtime. Four differential-drive robots can form chains by creating fixed joints between their chassis.

### Launching

```bash
# Terminal 1: Launch simulation
ros2 launch simulation docking_demo_harmonic_launch.py

# Terminal 2: Run manual control
ros2 run simulation manual_control
# or
ros2 run simulation docking_test
```

Or use the API directly:

```python
from simulation.docking_api import DockingController
import rclpy

rclpy.init()
controller = DockingController(num_robots=4)

controller.drive(robot_id=0, linear=0.3, angular=0.0, duration=1.0)
controller.dock(parent_id=0, child_id=1)  # Creates fixed joint
controller.undock(parent_id=0, child_id=1)

controller.destroy_node()
rclpy.shutdown()
```

### How Docking Works

The docking system pauses the simulation during joint creation to ensure perfect alignment:

1. **Stop robots** - Zero velocity commands sent to both robots
2. **Pause simulation** - Physics frozen to prevent momentum drift
3. **Teleport** - The free robot is moved into alignment behind the anchor
4. **Create joint** - Fixed joint created via AttachablePlugin
5. **Resume** - Physics unpaused

Pausing is necessary because Gazebo's `set_pose` service only sets position/orientation without zeroing velocity. Without pausing, residual momentum causes the robot to drift between teleport and joint creation.

## AttachablePlugin

This simulation uses [AttachablePlugin](../AttachablePlugin/) for runtime joint creation, which is superior to Gazebo's built-in `DetachableJoint` system plugin for modular robotics:

**Built-in DetachableJoint limitations:**
- Joints must be pre-defined in the SDF world file
- Each possible connection requires explicit configuration
- Cannot create joints between arbitrary models at runtime
- Requires world modifications to add new docking configurations

**AttachablePlugin advantages:**
- **Topic-based control** - Attach/detach ANY model to ANY other model via topic messages
- **No pre-configuration** - Joints created dynamically without SDF changes
- **Arbitrary combinations** - Any model/link pair can be connected at runtime
- **Ideal for modular robotics** - Robots can dock with each other in configurations not anticipated at design time

The plugin listens on `/attach` for messages in the format:
```
[parent_model][parent_link][child_model][child_link][attach|detach]
```

Example:
```bash
gz topic -t /attach -m gz.msgs.StringMsg -p 'data:"[robot_0][chassis][robot_1][chassis][attach]"'
```

## API Reference

| Method | Description |
|--------|-------------|
| `dock(parent_id, child_id, auto_align=True)` | Dock two robots |
| `undock(parent_id, child_id)` | Undock two robots |
| `drive(robot_id, linear, angular, duration)` | Drive for duration |
| `stop_robot(robot_id)` | Stop a robot |
| `get_robot_pose(robot_id)` | Get pose (x, y, z, yaw) |
| `set_robot_pose(robot_id, x, y, z, yaw)` | Teleport robot |
| `get_connected_robots(robot_id)` | Get all robots in chain |
| `pause_sim()` / `unpause_sim()` | Control physics |

---

## Legacy: SDF-Generated Multi-Robot Simulation

The original simulation system generates SDF world files from templates at launch time. This approach is useful for spawning large numbers of identical robots but does not support the AttachablePlugin.

### Launching

```bash
ros2 launch simulation simulation_launch.py n_robots:=10 robot_offset:=1.0 robot_arrangement:=CIRCLE
```

| Parameter | Description | Default |
|-----------|-------------|---------|
| n_robots | Number of robots | 1 |
| robot_offset | Minimum distance between robots (m) | 1.0 |
| robot_arrangement | LINE, GRID, or CIRCLE | LINE |

### How It Works

- **Templates**: `description/robot.template.sdf`, `description/world.template.sdf`
- **Parameters**: `description/robot_params.yaml`
- **Compilation**: Python scripts substitute `{% VARIABLES %}` with values from launch parameters

This system uses the older Ignition Gazebo (`ign gazebo`) and the built-in `DetachableJoint` plugin with pre-configured joints.

### Legacy Docking Demo

```bash
ros2 launch simulation docking_demo_launch.py
```

This launches a demo using the built-in DetachableJoint with pre-defined connections. Docking commands use the `ign topic` syntax and require joints to be pre-configured in the world file.
