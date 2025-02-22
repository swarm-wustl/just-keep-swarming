# OverheadCV

Publishes a video stream on `/video` from the webcam, tracks and filters the robot positions, and publishes the estimated positions and ids on `/robot{i}/pose` where `i=0..N-1` for the `N` specified robots.

- `camera_feed` publishes a video stream from the webcam
- `robot_tracker` detects robot positions based on color and publishes `/robot_observations` (real world, not pixel positions)
- `position_estimator` uses global nearest neighbor tracking and a Kalman filter to estimate the actual robot positions

## Dependencies

```
pip3 install opencv-python
```

## Running

```
ros2 launch overhead_cv overhead_tracking_launch.py
```

## Hardware Description

Camera information can be found in `cam_meta.yaml`.

## Testing

Tests can be run with the command below

```
colcon test --packages-select overhead_cv --event-handlers console_cohesion+
```
