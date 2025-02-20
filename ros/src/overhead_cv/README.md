# OverheadCV

Reads a video stream, tracks and filters the robot positions, and publishes the estimated positions and ids

The published array is formatted as so

[x1, y1, id1, x2, y2, id2, ... ]

The node camera_feed will run the publishing of camera

The package map_maker will handle producing an occupancy grid

## Dependencies

```
pip3 install opencv-python
```

## Running

```
ros2 launch overhead_cv overhead_tracking_launch.py
```

## Hardware Description

Camera data:

Fov_X = 130 deg
Fov_Y = 103 deg
focal_length (guesstimate) = 13.387mm = 0.013387m
cam_height: 49.5 inches -> 1.2573m
res_x = 1920
res_y = 1080

## Testing

Tests can be run with the command below

```
colcon test --packages-select overhead_cv --event-handlers console_cohesion+
```

unit tests can be found in the /test directory for each package
