cmake_minimum_required(VERSION 3.16)
project(ur5e_moveit_actions)

# --- Compiler -------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- Generate action interfaces ------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PlanToPose.action"
  "action/ExecutePlan.action"
  DEPENDENCIES geometry_msgs std_msgs
)

# --- Action server executable --------------------------------------------
add_executable(moveit_action_server
  src/moveit_action_server.cpp
)

target_include_directories(moveit_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(moveit_action_server
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  sensor_msgs
  trajectory_msgs
  moveit_msgs
  moveit_ros_planning_interface
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Link against the generated interfaces (same package)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(moveit_action_server "${cpp_typesupport_target}")

install(TARGETS moveit_action_server
  DESTINATION lib/${PROJECT_NAME}
)

# Python bridge (no build step — install as executable)
install(PROGRAMS
  src/unity_action_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME unity_action_bridge
)

# --- Launch + config ------------------------------------------------------
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
