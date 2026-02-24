from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # DualSense Joy Publisher (buttons and axes)
        Node(
            package='dualsense_teleop',
            executable='dualsense_joy_publisher',
            name='dualsense_joy',
            output='screen',
        ),

        # DualSense IMU Publisher
        Node(
            package='dualsense_teleop',
            executable='dualsense_publish_imu',
            name='dualsense_imu',
            output='screen',
            parameters=[{
                'rotation_x_deg': -90.0,
                'rotation_y_deg': 0.0,
                'rotation_z_deg': 180.0,
                # 'rotation_x_deg': 0.0,
                # 'rotation_y_deg': 0.0,
                # 'rotation_z_deg': 0.0,
            }]
        ),

        # IMU Filter (Madgwick) - for orientation estimation
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': True,
                'world_frame': 'enu',
                'fixed_frame': 'world',
            }]
        ),

        # IMU Orientation Offset - applies fixed yaw offset
        Node(
            package='dualsense_teleop',
            executable='imu_orientation_offset',
            name='imu_orientation_offset',
            output='screen',
            parameters=[{
                'rotation_x_deg': 180.0,
                'rotation_y_deg': 0.0,
                'rotation_z_deg': 0.0,
                'world_frame': 'world',  # World frame from TF tree
                'imu_frame': 'dualsense_imu_frame',  # IMU frame
                # 'imu_frame': 'imu_link',
                'auto_calibrate_on_start': True,
            }]
        ),

        # Teleop Gripper
        Node(
            package='dualsense_teleop',
            executable='teleop_gripper',
            name='teleop_gripper',
            output='screen',
        ),

        # DualSense Pose Tracking (joystick from DualSense + orientation from IMU filter)
        Node(
            package='dualsense_teleop',
            executable='dualsense_pose_tracking',
            name='dualsense_pose_tracking',
            output='screen',
            parameters=[{
                'linear_scale': 3.0,  # m/s max velocity
                'update_rate': 30.0,  # Hz
                # for panda
                # 'planning_frame': 'panda_link0',
                # 'ee_frame': 'panda_link8',
                # for  ur5
                'planning_frame': 'base_link',
                'ee_frame': 'tool0',
                'device_index': 0,
                'joystick_deadzone': 0.15,
                'imu_topic': '/imu/data_offset',  # Filtered IMU with orientation
                'use_normalized_mapping': True,  # Use normalized [-1, 1] values
            }]
        ),
    ])
