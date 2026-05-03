from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import socket

def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"

def generate_launch_description():
    local_ip = get_local_ip()

    return LaunchDescription([

        DeclareLaunchArgument(
            'ros_ip',
            default_value=local_ip,
            description='IP address for ROS TCP endpoint'
        ),

        DeclareLaunchArgument(
            'ros_port',
            default_value='10000',
            description='Port for ROS TCP endpoint'
        ),

        Node(
            package='ur5e_moveit_actions_pi',
            executable='rpi_fake_action_server.py',
            name='fake_action_server',
            output='screen',
        ),

        Node(
            package='ur5e_moveit_actions_pi',
            executable='rpi_unity_action_bridge.py',
            name='unity_action_bridge',
            output='screen',
        ),

        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='ros_tcp_endpoint',
            output='screen',
            parameters=[{
                'ROS_IP': LaunchConfiguration('ros_ip'),
                'ROS_TCP_PORT': LaunchConfiguration('ros_port'),
            }],
        ),
    ])