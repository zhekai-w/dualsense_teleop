from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch_param_builder import ParameterBuilder
from launch.substitutions import Command, FindExecutable
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ParameterFile


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
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.100",  # Change to your robot's IP
            # default_value="0.0.0.0",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",  # Set to false for real robot
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='',
            description="Prefix of the joint names, useful for multi-robot setup.",
        )
    )


    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    tf_prefix = LaunchConfiguration("tf_prefix")


    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")

    # Get paths for UR configuration files
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "joint_limits.yaml"]
    )
    # kinematics_params = PathJoinSubstitution(
    #     [FindPackageShare("ur_description"), "config", ur_type, "default_kinematics.yaml"]
    # )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("dualsense_teleop"), "config", "robot_calibration.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "visual_parameters.yaml"]
    )
    script_filename = PathJoinSubstitution(
        [FindPackageShare("ur_client_library"), "resources", "external_control.urscript"]
    )
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"]
    )
    initial_positions_file = PathJoinSubstitution(
        [FindPackageShare("dualsense_teleop"), "config", "ur5_initial_positions.yaml"]
    )
    update_rate_config_file = PathJoinSubstitution(
        [FindPackageShare("dualsense_teleop"), "config", "ur5_update_rate.yaml"]
    )
    ur_contoller = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "config", "ur_controllers.yaml"]
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
            tf_prefix,
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
            "script_filename:=",
            script_filename,
            " ",
            "input_recipe_filename:=",
            input_recipe_filename,
            " ",
            "output_recipe_filename:=",
            output_recipe_filename,
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
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            "false",
            " ",
            "initial_positions_file:=",
            initial_positions_file,
            " ",
            "robot_ip:=",
            robot_ip,
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
        "moveit_servo": ParameterBuilder("dualsense_teleop")
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
        package="ur_pose_tracking",
        executable="ur_pose_tracking",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            servo_params,
        ],
    )

    # ros2_control using FakeSystem as hardware
    # ros2_controllers_path = os.path.join(
    #     get_package_share_directory("dualsense_teleop"),
    #     "config",
    #     "ur5_ros2_controllers.yaml",
    # )

    ros2_controllers_path = PathJoinSubstitution(
        [FindPackageShare("dualsense_teleop"), "config", "ur5_ros2_controllers.yaml"]
    )

    ur_contoller = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "config", "ur_controllers.yaml"]
    )


    ros2_control_node_fake = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="screen",
        condition=IfCondition(use_fake_hardware),
    )

    ros2_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(ur_contoller, allow_substs=True),
        ],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
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

    ur_arm_controller_spawner_fake = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "-c", "/controller_manager"],
        condition=IfCondition(use_fake_hardware),
    )

    ur_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        # arguments=["scaled_joint_trajectory_controller", "-c", "/controller_manager"],
        arguments=["forward_position_controller", "-c", "/controller_manager"],
        condition=UnlessCondition(use_fake_hardware),
    )


    return LaunchDescription(
        declared_arguments
        + [
            rviz_node,
            static_tf,
            pose_tracking_node,
            ros2_control_node,
            ros2_control_node_fake,
            joint_state_broadcaster_spawner,
            ur_arm_controller_spawner_fake,
            ur_arm_controller_spawner,
            robot_state_publisher,
        ]
    )
