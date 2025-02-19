# OverheadCV

Reads a video stream, tracks and filters the robot positions, and publishes the estimated positions and ids

The published array is formatted as so

[x1, y1, id1, x2, y2, id2, ... ]

The node camera_feed will run the publishing of camera

The package map_maker will handle producing an OG grid

## Dependencies

```
pip3 install opencv-python
```

## Hardware Description

Camera data:

Fov_X = 130 deg
Fov_Y = 103 deg
focal_length (guesstimate) = 13.387mm = 0.013387m
cam_height: 49.5 inches -> 1.2573m
res_x = 1920
res_y = 1080

## Running

In /ros

In the future we need to make a yaml file for inputting parameters

```
ros2 run overhead_cv camera_feed --ros-args -p delay:=0.002 -p focal_length:=0.013387 -p cam_height:=1.2573 -p fov_x:=130.0 -p fov_y:=103.0 -p resolution_x:=1920.0 -p resolution_y:=1080.0 -p cam_input:=0

ros2 run overhead_cv robot_tracker --ros-args -p display:=true

ros2 run overhead_cv position_estimator --ros-args -p q:=1.0 -p r:=5.0 # q and r require tuning

ros2 run map_maker map_maker --ros-args -p resolution:=0.5
```

## Testimg

Tests can be run with the command below

```
colcon test --packages-select overhead_cv --event-handlers console_cohesion+
```

unit tests can be found in the /test directory for each package

