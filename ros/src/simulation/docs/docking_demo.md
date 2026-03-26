# Modular Robot Docking Demo

Runtime docking and undocking demonstration for modular robots in Gazebo Harmonic using the AttachablePlugin.

## Overview

This demo shows 4 differential-drive robots that can dynamically attach and detach at runtime. When attached, robots form a rigid chain that can be driven as a single unit.

### Environment

The environment is designed to test cooperative robot behaviors:

```
+===============================+  y = 2.2m
|         END ZONE              |
|                               |
+===+---------------------------+  y = 1.7m
    |  narrow wall              |
+---+                           +  y = 1.6m
|12cm                           |
| gap   MIDDLE PLATFORM         |
|       (docking area)          |
+===============================+  y = 0.9m
|                               |
|       GAP (20cm chasm)        |
|                               |
+===============================+  y = 0.7m
|                               |
|       START ZONE              |
|       (robots spawn)          |
+===============================+  y = 0.0m
x=0                          x=1.2m
```

**Key challenges:**
1. **Gap crossing (y=0.7-0.9m):** 20cm wide chasm - robots must form a chain to bridge across
2. **Narrow passage (y=1.6-1.7m):** 12cm gap on LEFT side (x=0-0.12) - robots must turn left to approach, then detach to fit through individually

### Robot Specifications

| Dimension | Value |
|-----------|-------|
| Overall footprint | 10 x 10 x 10 cm |
| Chassis | 8 x 8 x 8 cm |
| Drive wheels | 10 cm diameter, 1 cm thick |
| Support wheels | Frictionless spheres at docking faces |
| Mass | ~1 kg total |

## Requirements

- **Gazebo Harmonic** (gz-sim 8.x)
- **ROS 2 Jazzy**
- **ros_gz_bridge** - For ROS 2 / Gazebo topic bridging
- **AttachablePlugin** - Third-party plugin for dynamic joint creation

### Installing AttachablePlugin

```bash
cd ~/ros2_ws/src
git clone https://github.com/akinami3/AttachablePlugin.git
cd ~/ros2_ws
colcon build --packages-select attachable_joint_plugin
```

## Launching the Demo

```bash
# Terminal 1: Set plugin path and launch simulation
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build/attachable_joint_plugin:$GZ_SIM_SYSTEM_PLUGIN_PATH
ros2 launch simulation docking_demo_harmonic_launch.py

# Terminal 2: Run manual control
ros2 run simulation manual_control
```

## Manual Control

The `manual_control` node provides keyboard-based control for driving and docking robots.

### Controls

| Key | Action |
|-----|--------|
| **W** / **Up** | Drive forward |
| **S** / **Down** | Drive backward |
| **A** / **Left** | Turn left |
| **D** / **Right** | Turn right |
| **1-4** | Select robot 0-3 |
| **Space** | Stop selected robot |
| **X** | Stop all robots |
| **J** | Dock selected robot to next (N → N+1) |
| **K** | Undock selected robot from next |
| **L** | Dock previous robot to selected (N-1 → N) |
| **;** | Undock previous robot from selected |
| **H** | Show help |
| **Q** / **Esc** | Quit |

### Example Workflow

1. Select robot 0: press **1**
2. Drive forward to approach robot 1: hold **W**
3. Stop: press **Space**
4. Dock robot 0 to robot 1: press **J**
5. Now both robots move together when you drive

## Command-Line Control (Alternative)

You can also control robots directly via command line.

### Attach/Detach Commands

```bash
# Attach robot_0 to robot_1
gz topic -t /attach -m gz.msgs.StringMsg -p 'data:"[robot_0][chassis][robot_1][chassis][attach]"'

# Detach robot_0 from robot_1
gz topic -t /attach -m gz.msgs.StringMsg -p 'data:"[robot_0][chassis][robot_1][chassis][detach]"'
```

### Drive Commands (via ROS 2)

```bash
# Drive robot_0 forward
ros2 topic pub /model/robot_0/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

# Turn robot_0
ros2 topic pub /model/robot_0/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Stop robot_0
ros2 topic pub /model/robot_0/cmd_vel geometry_msgs/msg/Twist "{}"
```

### Driving Attached Chains

When robots are attached, driving **any** robot in the chain moves the entire chain.

## Physics Tuning

Smooth motion of attached robots required specific physics parameters to prevent constraint error accumulation.

### The Problem

When robots are attached with fixed joints and driven:
1. Small position/orientation errors accumulate between constrained bodies
2. Rigid constraints cause errors to build up until the solver fails
3. Results in periodic "jerky" motion (pause, then sudden jump)

### The Solution

Two key parameters in the SDF physics configuration:

```xml
<physics name="1ms" type="ode">
  <ode>
    <constraints>
      <!-- Constraint Force Mixing - adds compliance to joints -->
      <cfm>0.00001</cfm>
      <erp>0.2</erp>
    </constraints>
  </ode>
</physics>
```

**CFM (Constraint Force Mixing):** Adds slight "springiness" to fixed joints, allowing small errors to dissipate rather than accumulate.

Additionally, wheel surfaces include slip parameters:

```xml
<surface>
  <friction>
    <ode>
      <mu>1.0</mu>
      <mu2>1.0</mu2>
      <slip1>0.002</slip1>
      <slip2>0.002</slip2>
    </ode>
  </friction>
</surface>
```

**Slip:** Allows wheels to slip slightly, preventing ground friction from fighting the joint constraints when attached robots have minor velocity differences.

## Architecture

### Robot Structure

Each robot has 4 identical wheels (10cm diameter, 1cm thick):
- **left_wheel**, **right_wheel** - Differential drive wheels (dark gray, with friction)
- **front_wheel**, **back_wheel** - Frictionless support wheels at docking faces (chassis color)

The front/back wheels represent the docking faces in the real hardware. They are colored to match the chassis to visually distinguish them from the drive wheels.

Overall footprint: 10x10x10 cm

### Plugins

| Plugin | Purpose |
|--------|---------|
| `gz-sim-diff-drive-system` | Differential drive control per robot |
| `libattachable_joint_plugin.so` | Dynamic joint attachment/detachment |

### Topics

**Gazebo Topics:**

| Topic | Type | Purpose |
|-------|------|---------|
| `/attach` | `gz.msgs.StringMsg` | Attach/detach commands |

**ROS 2 Topics (bridged):**

| Topic | Type | Purpose |
|-------|------|---------|
| `/model/robot_N/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/model/robot_N/odometry` | `nav_msgs/Odometry` | Odometry feedback |

## Known Limitations

1. **Joint Hierarchy:** AttachablePlugin internally uses DetachableJoint, which creates parent-child relationships. The first model in the attach command becomes the parent. This affects GUI drag behavior but not differential drive control.

2. **Tree Topology Only:** Cannot create closed kinematic loops (rings). Attached robots must form a tree structure.

3. **Proximity Not Required:** Unlike some docking systems, AttachablePlugin doesn't require robots to be adjacent to attach - it will "teleport" them together. For realistic docking, ensure robots are positioned correctly before attaching.

## File Locations

```
simulation/
├── description/
│   └── docking_demo_harmonic.sdf    # World file with robots and physics config
├── launch/
│   └── docking_demo_harmonic_launch.py  # Launches Gazebo + ros_gz_bridge
├── simulation/
│   └── manual_control.py            # Keyboard control node
└── docs/
    └── docking_demo.md              # This file
```
