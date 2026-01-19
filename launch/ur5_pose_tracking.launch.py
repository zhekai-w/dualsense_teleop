import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch_param_builder import ParameterBuilder
from launch.substitutions import Command, FindExecutable


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5",
            description="Type/series of used UR robot.",
        )
    )

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")

    # Get paths for UR configuration files
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "visual_parameters.yaml"]
    )
    initial_positions_file = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "initial_positions.yaml"]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]
            ),
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=",
            '""',
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            "false",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "use_fake_hardware:=",
            "true",
            " ",
            "fake_sensor_commands:=",
            "false",
            " ",
            "initial_positions_file:=",
            initial_positions_file,
            " ",
            "robot_ip:=",
            "0.0.0.0",
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Get SRDF via xacro
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]
            ),
            " ",
            "name:=",
            "ur",
            " ",
            "prefix:=",
            '""',
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }

    # Get planning parameters
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]
    )

    # Get parameters for the Pose Tracking node
    servo_params = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml("config/pose_tracking_settings.yaml")
        .yaml("config/ur_servo.yaml")
        .to_dict()
    }

    # RViz
    rviz_config_file = (
        get_package_share_directory("moveit_servo")
        + "/config/demo_rviz_pose_tracking.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    # Publishes tf's for the robot
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # A node to publish world -> base_link transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    pose_tracking_node = Node(
        package="moveit_servo",
        executable="servo_pose_tracking_demo",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            servo_params,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("dualsense_teleop"),
        "config",
        "ur5_ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    ur_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription(
        declared_arguments
        + [
            rviz_node,
            static_tf,
            pose_tracking_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            ur_arm_controller_spawner,
            robot_state_publisher,
        ]
    )
