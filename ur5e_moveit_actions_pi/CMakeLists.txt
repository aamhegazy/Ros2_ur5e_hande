cmake_minimum_required(VERSION 3.16)
project(ur5e_moveit_actions_pi)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PlanToPose.action"
  "action/ExecutePlan.action"
  DEPENDENCIES geometry_msgs std_msgs
)

install(PROGRAMS
  ../ur5e_moveit_actions/src/rpi_fake_action_server.py
  ../ur5e_moveit_actions/src/rpi_unity_action_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME})

ament_package()