cmake_minimum_required(VERSION 3.8)
project(shared_types)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(type_files "srv/CamMeta.srv" "msg/RobotPosition.msg" "msg/PidPosition.msg"
               "msg/RobotPoints.msg")

rosidl_generate_interfaces(${PROJECT_NAME} ${type_files} DEPENDENCIES
                           geometry_msgs)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
