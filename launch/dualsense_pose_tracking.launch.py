from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # DualSense IMU Publisher
        Node(
            package='dualsense_teleop',
            executable='dualsense_publish_imu',
            name='dualsense_imu',
            output='screen',
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
                'imu_topic': '/imu/data',  # Filtered IMU with orientation
                'use_normalized_mapping': True,  # Use normalized [-1, 1] values
            }]
        ),
    ])
