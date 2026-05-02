# Launch only Moveit Action server 

# Prequusutes:
#   1st: spawn_ur5e_hande.launch.py from the MoveIt_config_package is must 

#   2nd: ros2 launch ur5e_moveit_actions action_server.launch.py 

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    action_server = Node(
        package="ur5e_moveit_actions",
        executable="moveit_action_server",
        name="moveit_action_server",
        output="screen",
        parameters=[{
            "planning_group": "ur5e",
            "base_frame": "base_link",
            "tcp_link": "tool0",
            "default_plan_time": 5.0,
            "default_vel_scale": 0.3,
            "default_accel_scale": 0.3,
            "plan_ttl_seconds": 300.0,
            "waypoint_stride": 5,
        }],
    )

    unity_bridge = Node(
        package="ur5e_moveit_actions",
        executable="unity_action_bridge",
        name="unity_action_bridge",
        output="screen",
    )

    return LaunchDescription([
        action_server,
        unity_bridge
        ])