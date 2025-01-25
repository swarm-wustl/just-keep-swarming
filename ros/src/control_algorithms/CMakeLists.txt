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
find_package(nav_msgs REQUIRED)
find_package(drone_msg REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME} "action/MultiRobotPathPlan.action" "action/Assemble.action"
  "msg/GridCell.msg" DEPENDENCIES geometry_msgs)

set(ALGORITHM_SOURCES
    src/algorithms/common.cpp src/algorithms/astar.cpp src/algorithms/sstar.cpp
    src/algorithms/pplan.cpp src/algorithms/assemble_graph.cpp)

# === MRPP Action Server ===
add_library(mrpp_action_server SHARED
            src/multi_robot_path_planner_action_server.cpp ${ALGORITHM_SOURCES})

target_include_directories(
  mrpp_action_server
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
          ${CMAKE_CURRENT_SOURCE_DIR}/include/control_algorithms/algorithms)
ament_target_dependencies(mrpp_action_server "rclcpp" "rclcpp_action"
                          "rclcpp_components" "nav_msgs" "drone_msg")
rclcpp_components_register_node(
  mrpp_action_server PLUGIN
  "control_algorithms::MultiRobotPathPlannerActionServer" EXECUTABLE
  multi_robot_path_planner_action_server)
install(
  TARGETS mrpp_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# === Assembler Action Server ===
add_library(asm_action_server SHARED src/assembler_action_server.cpp
                                     src/algorithms/assemble_graph.cpp)
target_include_directories(
  asm_action_server
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
          ${CMAKE_CURRENT_SOURCE_DIR}/include/control_algorithms/algorithms)
ament_target_dependencies(asm_action_server "rclcpp" "rclcpp_action"
                          "rclcpp_components" "nav_msgs")
rclcpp_components_register_node(
  asm_action_server PLUGIN "control_algorithms::AssemblerActionServer"
  EXECUTABLE assembler_action_server)
install(
  TARGETS asm_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# === MRPP Action Client ===
add_library(
  mrpp_action_client SHARED
  src/demo_mrpp_client.cpp src/algorithms/assemble_graph.cpp
  src/algorithms/common.cpp)
target_include_directories(
  mrpp_action_client
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mrpp_action_client "rclcpp" "rclcpp_action"
                          "rclcpp_components" "nav_msgs")
rclcpp_components_register_node(
  mrpp_action_client PLUGIN "control_algorithms::MRPPActionClient" EXECUTABLE
  demo_mrpp_action_client)
install(
  TARGETS mrpp_action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION BIN)

# === Assembler Action Client ===
add_library(assembler_action_client SHARED src/demo_assembler_client.cpp)
target_include_directories(
  assembler_action_client
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>)
ament_target_dependencies(assembler_action_client "rclcpp" "rclcpp_action"
                          "rclcpp_components")
rclcpp_components_register_node(
  assembler_action_client PLUGIN "control_algorithms::AssemblerActionClient"
  EXECUTABLE demo_assembler_action_client)
install(
  TARGETS assembler_action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION BIN)

# --- necessary for using actions in same package in which they are defined
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
                              "rosidl_typesupport_cpp")
target_link_libraries(mrpp_action_server "${cpp_typesupport_target}")
target_link_libraries(mrpp_action_client "${cpp_typesupport_target}")
target_link_libraries(asm_action_server "${cpp_typesupport_target}")
target_link_libraries(assembler_action_client "${cpp_typesupport_target}")

# === Testing ===
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # List of test names and corresponding source files
  set(TESTS common_test astar_test sstar_test pplan_test assemble_graph_test)

  foreach(TEST ${TESTS})
    ament_add_gtest(${PROJECT_NAME}_${TEST} test/${TEST}.cpp)

    target_include_directories(
      ${PROJECT_NAME}_${TEST}
      PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
              $<INSTALL_INTERFACE:include>
              ${CMAKE_CURRENT_SOURCE_DIR}/include/control_algorithms/algorithms)

    target_link_libraries(${PROJECT_NAME}_${TEST} asm_action_server
                          mrpp_action_server)
  endforeach()
endif()

ament_package()
