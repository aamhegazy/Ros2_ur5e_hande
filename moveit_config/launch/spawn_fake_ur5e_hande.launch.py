from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # --- EAGERLY RESOLVED PATHS ---
    ur_driver_share = get_package_share_directory("ur_robot_driver")
    script_filename         = os.path.join(ur_driver_share, "resources", "ros_control.urscript")
    output_recipe_filename  = os.path.join(ur_driver_share, "resources", "rtde_output_recipe.txt")
    input_recipe_filename   = os.path.join(ur_driver_share, "resources", "rtde_input_recipe.txt")

    moveit_config_share = get_package_share_directory("moveit_config")
    initial_positions_file  = os.path.join(moveit_config_share, "config", "initial_positions.yaml")

    for f in [script_filename, output_recipe_filename, input_recipe_filename, initial_positions_file]:
        if not os.path.isfile(f):
            raise FileNotFoundError(f"Required file not found: {f}")

    # --- ARGUMENTS ---
    declared_args = []

    declared_args.append(DeclareLaunchArgument(
        "ur_ip",
        default_value="192.168.1.10",
        description="Not used in fake mode, kept for config compatibility"
    ))

    declared_args.append(DeclareLaunchArgument(
        "reverse_ip",
        default_value="192.168.1.11",
        description="Not used in fake mode, kept for config compatibility"
    ))

    ur_ip      = LaunchConfiguration("ur_ip")
    reverse_ip = LaunchConfiguration("reverse_ip")

    # --- 1. MOVEIT CONFIG ---
    moveit_config = (
        MoveItConfigsBuilder("moveit_config", package_name="moveit_config")
        .robot_description(
            file_path="config/ur5e_hande.urdf.xacro",
            mappings={
                "name":             "ur5e",
                "robot_ip":         ur_ip,
                "reverse_ip":       reverse_ip,
                "socat_ip_address": ur_ip,
                "use_fake_hardware": "true",
                "sim_gazebo":        "false",
                "sim_ignition":      "false",
                "script_filename":        script_filename,
                "output_recipe_filename": output_recipe_filename,
                "input_recipe_filename":  input_recipe_filename,
                "initial_positions_file": initial_positions_file,
                "create_socat_tty": "false",
                "socat_port":       "54321",
                "tty_port":         "/tmp/ttyUR",
            },
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # --- 2. CONTROL NODE ---
    control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            PathJoinSubstitution([FindPackageShare("moveit_config"), "config", "ros2_controllers.yaml"]),
            {"reverse_ip":         reverse_ip},
            {"script_sender_port": 50002},
            {"headless_mode":      False},
            {"tf_prefix":          ""},
        ],
        output="screen",
    )

    # --- 3. MOVE GROUP ---
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
            {"trajectory_execution.allowed_execution_duration_scaling": 1.2},
            {"trajectory_execution.allowed_goal_duration_margin": 0.5},
            {"publish_robot_description_semantic": True},
        ],
    )

    # --- 4. RVIZ ---
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": False},
        ],
        arguments=["-d", PathJoinSubstitution([FindPackageShare("moveit_config"), "config", "moveit.rviz"])],
    )

    # --- 5. ROBOT STATE PUBLISHER ---
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # --- 6. SPAWN CONTROLLERS ---
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    spawn_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scaled_joint_trajectory_controller", "-c", "/controller_manager"],
    )

    spawn_gripper = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
    )

    # --- LAUNCH ORDER ---
    return LaunchDescription(declared_args + [
        robot_state_publisher_node,
        control_node,
        spawn_jsb,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_jsb,
                on_exit=[spawn_arm, spawn_gripper, move_group_node, rviz_node],
            )
        ),
    ])