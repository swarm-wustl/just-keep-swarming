# Make sure this is run in its parent directory (i.e., esp)
# ex: sh scripts/[name].sh
. ~/ros2_humble/install/setup.sh
cd ./components/micro_ros_espidf_component/extra_packages
colcon build
cd -
tset
