ros2 launch depthai_ros_driver camera.launch.py

# Camera module

Uses an Oak
Publishes a video stream on `/video` from the webcam, tracks and filters the robot positions, and publishes the estimated positions and ids on `/robot{i}/pose` where `i=0..N-1` for the `N` specified robots.

Uses an RBGD camera (OAK-D Lite) to create a point cloud using RTAB that then is used for mapping purposes.

- `__` point cloud slicer publishes 3 point clouds correlating to gaps, ground, and obstacles
- `__` uses the 3 point clouds to create an occupancy style map
- `__` additional filtering such as robot removal from obstacles

## Dependencies

```

sudo apt install ros-humble-depthai-ros


cd ~/ros2_ws
git clone https://github.com/introlab/rtabmap.git src/rtabmap
git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
rosdep update && rosdep install --from-paths src --ignore-src -r -y
export MAKEFLAGS="-j6" # Can be ignored if you have a lot of RAM (>16GB)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release```

## Running

```
ros2 launch camera_module map_maker.launch.py
```

## Hardware Description


## Testing
