cmake_minimum_required(VERSION 3.8)
project(control_algorithms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "action/MultiRobotPathPlan.action"
                           DEPENDENCIES geometry_msgs)

# === Action Server ===
set(ALGORITHM_SOURCES src/algorithms/astar.cpp)
add_library(action_server SHARED src/multi_robot_path_planner_action_server.cpp
                                 ${ALGORITHM_SOURCES})
target_include_directories(
  action_server
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
          ${CMAKE_CURRENT_SOURCE_DIR}/include/control_algorithms/algorithms)
ament_target_dependencies(action_server "rclcpp" "rclcpp_action"
                          "rclcpp_components")
rclcpp_components_register_node(
  action_server PLUGIN "control_algorithms::MultiRobotPathPlannerActionServer"
  EXECUTABLE multi_robot_path_planner_action_server)
install(
  TARGETS action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# === Action Client ===
add_library(action_client SHARED src/demo_mrpp_client.cpp)
target_include_directories(
  action_client PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(action_client "rclcpp" "rclcpp_action"
                          "rclcpp_components")
rclcpp_components_register_node(
  action_client PLUGIN "control_algorithms::MRPPActionClient" EXECUTABLE
  demo_mrpp_action_client)
install(
  TARGETS action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION BIN)

# --- necessary for using actions in same package in which they are defined
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
                              "rosidl_typesupport_cpp")
target_link_libraries(action_server "${cpp_typesupport_target}")
target_link_libraries(action_client "${cpp_typesupport_target}")

# === Testing ===
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_astar_test test/astar_test.cpp)

  target_include_directories(
    ${PROJECT_NAME}_astar_test
    PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
            ${CMAKE_CURRENT_SOURCE_DIR}/include/control_algorithms/algorithms)

  target_link_libraries(${PROJECT_NAME}_astar_test action_server)
endif()

ament_package()
