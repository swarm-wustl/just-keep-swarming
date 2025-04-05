echo "ARG: $1"
. ~/ros2_humble/install/setup.sh
ros2 topic pub /diffdrive_twist_$1 geometry_msgs/msg/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
