# Mantis XR — Raspberry Pi 4 ROS2 Bridge

Hardware: Raspberry Pi 4, Ubuntu Server 22.04, ROS2 Humble base

## What Runs on the Pi
- ros_tcp_endpoint (patched for action message support)
- ur5e_moveit_actions (message types only)

## Network
- Pi IP: 172.20.10.3
- Ubuntu PC: 172.20.10.4
- Windows/Unity: 172.20.10.6
- Hotspot gateway: 172.20.10.1

## Build
source /opt/ros/humble/setup.bash
colcon build --packages-select ur5e_moveit_actions ros_tcp_endpoint

## Run
ros2 run ros_tcp_endpoint default_server_endpoint \
  --ros-args -p ROS_IP:=172.20.10.3 -p ROS_TCP_PORT:=10000

## Unity Settings
ROS IP: 172.20.10.3, Port: 10000, Protocol: ROS2
