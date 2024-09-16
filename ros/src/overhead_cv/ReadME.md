### OverheadCV

## Quick Summary?

Reads a video stream, outputs and tracks the qr codes, and publishes the positions and id to an array.

The published array is formatted as so

[x1, y1, id1, x2, y2, id2, ... ]

Read robot will have a qr attached correlating to its id in the swarm

## Setup

Source in /ros

```
source install/setup.bash
```

Packages

```
pip3 install opencv-python
```

build in /ros directory

```
colcon build --packages-select overhead_cv
colcon build --packages-select camera_feed
```

## Running

In /ros

```
ros2 run camera_feed camera_feed
ros2 run overhead_cv obs_qr --ros-args -p display:=[true or false]
```