# Modular Robot Docking Demo

Runtime docking and undocking demonstration for modular robots in Gazebo Harmonic using the AttachablePlugin.

## Overview

This demo shows 4 differential-drive robots that can dynamically attach and detach at runtime. When attached, robots form a rigid chain that can be driven as a single unit.

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
# Set plugin path (required - plugin installs to build directory)
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build/attachable_joint_plugin:$GZ_SIM_SYSTEM_PLUGIN_PATH

# Launch
ros2 launch simulation docking_demo_harmonic_launch.py
```

## Attaching and Detaching Robots

Robots start **disconnected**. Use the `/attach` topic to create or remove joints.

### Message Format

```
[parent_model][parent_link][child_model][child_link][attach|detach]
```

### Attach Examples

```bash
# Attach robot_0 to robot_1
gz topic -t /attach -m gz.msgs.StringMsg -p 'data:"[robot_0][chassis][robot_1][chassis][attach]"'

# Attach robot_1 to robot_2
gz topic -t /attach -m gz.msgs.StringMsg -p 'data:"[robot_1][chassis][robot_2][chassis][attach]"'

# Attach robot_2 to robot_3 (form a 4-robot chain)
gz topic -t /attach -m gz.msgs.StringMsg -p 'data:"[robot_2][chassis][robot_3][chassis][attach]"'
```

### Detach Examples

```bash
# Detach robot_1 from robot_2 (splits chain into two pairs)
gz topic -t /attach -m gz.msgs.StringMsg -p 'data:"[robot_1][chassis][robot_2][chassis][detach]"'

# Detach robot_0 from robot_1
gz topic -t /attach -m gz.msgs.StringMsg -p 'data:"[robot_0][chassis][robot_1][chassis][detach]"'
```

## Driving Robots

Each robot has a differential drive that accepts velocity commands.

### Drive Commands

```bash
# Drive robot_0 forward
gz topic -t /model/robot_0/cmd_vel -m gz.msgs.Twist -p 'linear:{x:0.3}' &
gz topic -t /model/robot_1/cmd_vel -m gz.msgs.Twist -p 'linear:{x:0.3}' &
gz topic -t /model/robot_2/cmd_vel -m gz.msgs.Twist -p 'linear:{x:0.3}' &
gz topic -t /model/robot_3/cmd_vel -m gz.msgs.Twist -p 'linear:{x:0.3}'

# Drive robot_1 forward
gz topic -t /model/robot_1/cmd_vel -m gz.msgs.Twist -p 'linear:{x:0.3}'

# Turn robot_0 (angular velocity)
gz topic -t /model/robot_0/cmd_vel -m gz.msgs.Twist -p 'angular:{z:0.5}'

# Stop robot_0
gz topic -t /model/robot_0/cmd_vel -m gz.msgs.Twist -p 'linear:{x:0}'
```

### Driving Attached Chains

When robots are attached, driving **any** robot in the chain will move the entire chain:

```bash
# Connect robots 0, 1, 2
gz topic -t /attach -m gz.msgs.StringMsg -p 'data:"[robot_0][chassis][robot_1][chassis][attach]"'
gz topic -t /attach -m gz.msgs.StringMsg -p 'data:"[robot_1][chassis][robot_2][chassis][attach]"'

# Drive from the middle - whole chain moves
gz topic -t /model/robot_1/cmd_vel -m gz.msgs.Twist -p 'linear:{x:0.3}'
```

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

| Topic | Type | Purpose |
|-------|------|---------|
| `/attach` | `gz.msgs.StringMsg` | Attach/detach commands |
| `/model/robot_N/cmd_vel` | `gz.msgs.Twist` | Velocity commands |
| `/model/robot_N/odometry` | `gz.msgs.Odometry` | Odometry feedback |

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
│   └── docking_demo_harmonic_launch.py
└── docs/
    └── docking_demo.md              # This file
```
