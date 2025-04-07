# Make sure this is run in its parent directory (i.e., esp)
# eg: source scripts/[name].sh
# MUST BE RUN AS A SOURCE COMMAND
. ~/ros2_humble/install/setup.sh
cd ./components/micro_ros_espidf_component/extra_packages
colcon build
. ./install/setup.sh