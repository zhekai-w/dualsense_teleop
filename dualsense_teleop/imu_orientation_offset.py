#!/usr/bin/env python3
"""
IMU Orientation Offset Node

Applies a yaw offset to IMU orientation to ensure consistent
orientation relative to world frame on startup.
Uses TF tree to get world frame reference.

L1 button control:
- L1 pressed: tracking mode (orientation updates continuously)
- L1 released: hold mode (orientation frozen at last position)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Joy
from std_srvs.srv import Empty
import numpy as np
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster
import tf2_ros
from geometry_msgs.msg import TransformStamped


class ImuOrientationOffset(Node):
    # Button index for L1 (from dualsense_joy_publisher)
    BTN_L1 = 8

    def __init__(self):
        super().__init__('imu_orientation_offset')

        # Declare parameters
        # self.declare_parameter('target_yaw_deg', 180.0)  # Target yaw offset in degrees

        self.declare_parameter('rotation_x_deg', 180.0)
        self.declare_parameter('rotation_y_deg', 0.0)
        self.declare_parameter('rotation_z_deg', 90.0)

        self.declare_parameter('world_frame', 'world')  # World frame to use as reference
        self.declare_parameter('imu_frame', 'dualsense_imu_frame')  # IMU frame from TF tree
        self.declare_parameter('auto_calibrate_on_start', True)  # Auto-calibrate on first message
        self.declare_parameter('joy_topic', '/joy')  # Joy topic for L1 button

        # target_yaw_deg = self.get_parameter('target_yaw_deg').value


        self.rx_deg = self.get_parameter('rotation_x_deg').value
        self.ry_deg = self.get_parameter('rotation_y_deg').value
        self.rz_deg = self.get_parameter('rotation_z_deg').value

        self.world_frame = self.get_parameter('world_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.auto_calibrate = self.get_parameter('auto_calibrate_on_start').value
        self.joy_topic = self.get_parameter('joy_topic').value

        # Target rotation (e.g., 180 degrees around z-axis)
        # self.target_rotation = R.from_euler('z', np.radians(target_yaw_deg))
        self.target_rotation = R.from_euler('xyz', np.radians([self.rx_deg, self.ry_deg, self.rz_deg]))

        # Reference rotation (from TF tree)
        self.reference_rotation = None
        self.calibrated = False

        # L1 button tracking state
        self.l1_pressed = False
        self.tracking_enabled = False  # Whether to update orientation
        self.held_rotation = None  # Stored rotation when L1 is released
        self.last_imu_rotation = None  # Last raw IMU rotation for computing deltas
        self.resume_imu_rotation = None  # IMU rotation when L1 was pressed (for computing delta)

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # TF broadcaster for publishing the offset frame
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber to filtered IMU
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Subscriber to Joy for L1 button
        self.joy_sub = self.create_subscription(
            Joy,
            self.joy_topic,
            self.joy_callback,
            10
        )

        # Publisher for offset-corrected IMU
        self.imu_pub = self.create_publisher(Imu, '/imu/data_offset', 10)

        # Service to recalibrate
        self.calibrate_srv = self.create_service(
            Empty,
            '~/calibrate',
            self.calibrate_callback
        )

        self.get_logger().info('IMU Orientation Offset Node started')
        # self.get_logger().info(f'Target yaw offset: {target_yaw_deg}°')
        self.get_logger().info(f'World frame: {self.world_frame}')
        self.get_logger().info(f'IMU frame: {self.imu_frame}')
        self.get_logger().info(f'Auto-calibrate on start: {self.auto_calibrate}')
        self.get_logger().info(f'Joy topic: {self.joy_topic}')
        self.get_logger().info('L1 button: hold to track orientation, release to freeze')
        if self.auto_calibrate:
            self.get_logger().info('Waiting for first IMU message to calibrate...')
        else:
            self.get_logger().info('Call ~/calibrate service to set reference orientation')

    def joy_callback(self, msg: Joy):
        """Handle Joy messages for L1 button state."""
        if len(msg.buttons) <= self.BTN_L1:
            return

        l1_now_pressed = bool(msg.buttons[self.BTN_L1])

        # Detect L1 press (rising edge)
        if l1_now_pressed and not self.l1_pressed:
            self.tracking_enabled = True
            # Calculate offset to continue from held orientation
            # When resuming, we want: held_rotation * delta = new_output
            # So we store the current IMU rotation to compute delta later
            self.resume_imu_rotation = self.last_imu_rotation if self.last_imu_rotation is not None else None
            self.get_logger().info('L1 pressed: tracking enabled')

        # Detect L1 release (falling edge)
        elif not l1_now_pressed and self.l1_pressed:
            self.tracking_enabled = False
            # held_rotation is already set in imu_callback during tracking
            if self.held_rotation is not None:
                euler = self.held_rotation.as_euler('xyz', degrees=True)
                self.get_logger().info(
                    f'L1 released: orientation frozen at '
                    f'roll={euler[0]:.1f}°, pitch={euler[1]:.1f}°, yaw={euler[2]:.1f}°'
                )
            else:
                self.get_logger().info('L1 released: orientation frozen')

        self.l1_pressed = l1_now_pressed

    def calibrate_callback(self, request, response):
        """Service callback to recalibrate reference orientation."""
        self.calibrated = False
        self.reference_rotation = None
        self.get_logger().info('Calibration reset. Waiting for next IMU message...')
        return response

    def get_world_to_imu_rotation(self):
        """Look up the rotation from world frame to IMU frame via TF tree."""
        try:
            # Look up transform from world to IMU frame
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.imu_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Extract rotation from transform
            q = transform.transform.rotation
            rotation = R.from_quat([q.x, q.y, q.z, q.w])

            euler = rotation.as_euler('xyz', degrees=True)
            self.get_logger().info(
                f'TF lookup successful: {self.world_frame} -> {self.imu_frame} = '
                f'roll={euler[0]:.1f}°, pitch={euler[1]:.1f}°, yaw={euler[2]:.1f}°'
            )

            return rotation

        except tf2_ros.LookupException as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f'TF extrapolation failed: {e}')
            return None
        except Exception as e:
            self.get_logger().warn(f'TF error: {e}')
            return None

    def imu_callback(self, msg):
        """Process IMU message and apply orientation offset."""

        # Extract current orientation as rotation
        q = msg.orientation
        current_rotation = R.from_quat([q.x, q.y, q.z, q.w])

        # Store the last IMU rotation for delta computation
        self.last_imu_rotation = current_rotation

        # If not calibrated and auto-calibrate is enabled, calibrate now
        if not self.calibrated:
            if self.auto_calibrate or self.reference_rotation is None:
                # Get world frame orientation from TF tree
                world_to_imu = self.get_world_to_imu_rotation()

                if world_to_imu is not None:
                    # Use the TF tree orientation as reference
                    self.reference_rotation = world_to_imu
                    self.calibrated = True

                    # Get current yaw from IMU message
                    current_euler = current_rotation.as_euler('xyz', degrees=True)
                    self.get_logger().info(
                        f'Calibrated using TF tree! '
                        f'Current IMU orientation: roll={current_euler[0]:.1f}°, '
                        f'pitch={current_euler[1]:.1f}°, yaw={current_euler[2]:.1f}°'
                    )
                    self.get_logger().info(
                        'Reference set from world frame. '
                        # f'Applying target offset of {self.get_parameter("target_yaw_deg").value}° around z-axis'
                    )
                else:
                    # Fallback: use current IMU orientation if TF lookup fails
                    self.get_logger().warn(
                        'TF lookup failed, using current IMU orientation as reference (fallback mode)'
                    )
                    self.reference_rotation = current_rotation
                    self.calibrated = True

        # If calibrated, apply offset
        if self.calibrated and self.reference_rotation is not None:
            # L1 button logic: tracking vs hold mode
            if self.tracking_enabled:
                # Tracking mode: compute delta from last frame and apply to held rotation
                if self.resume_imu_rotation is not None and self.held_rotation is not None:
                    # Calculate how much the IMU has rotated since last frame
                    delta_rotation = self.resume_imu_rotation.inv() * current_rotation
                    # Apply delta to the held rotation (continue from where we left off)
                    final_rotation = self.held_rotation * delta_rotation
                else:
                    # First time tracking or no held rotation yet - use standard calculation
                    relative_rotation = self.reference_rotation.inv() * current_rotation
                    final_rotation = self.target_rotation * relative_rotation
                
                # Store current state for next frame
                self.held_rotation = final_rotation
                self.resume_imu_rotation = current_rotation  # Update for next frame's delta
            else:
                # Hold mode: use the stored rotation if available
                if self.held_rotation is not None:
                    final_rotation = self.held_rotation
                else:
                    # First run before L1 pressed - initialize held_rotation
                    relative_rotation = self.reference_rotation.inv() * current_rotation
                    final_rotation = self.target_rotation * relative_rotation
                    self.held_rotation = final_rotation

            # Convert back to quaternion
            final_quat = final_rotation.as_quat()  # [x, y, z, w]

            # Create output message
            output_msg = Imu()
            output_msg.header = msg.header
            output_msg.header.frame_id = 'imu_offset_frame'

            # Set corrected orientation
            output_msg.orientation.x = final_quat[0]
            output_msg.orientation.y = final_quat[1]
            output_msg.orientation.z = final_quat[2]
            output_msg.orientation.w = final_quat[3]
            output_msg.orientation_covariance = msg.orientation_covariance

            # Keep angular velocity and linear acceleration unchanged
            output_msg.angular_velocity = msg.angular_velocity
            output_msg.angular_velocity_covariance = msg.angular_velocity_covariance
            output_msg.linear_acceleration = msg.linear_acceleration
            output_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

            # Publish TF transform
            transform = TransformStamped()
            transform.header.stamp = output_msg.header.stamp
            transform.header.frame_id = self.world_frame
            transform.child_frame_id = 'imu_offset_frame'
            transform.transform.rotation.x = final_quat[0]
            transform.transform.rotation.y = final_quat[1]
            transform.transform.rotation.z = final_quat[2]
            transform.transform.rotation.w = final_quat[3]
            # Position is at origin (no translation)
            transform.transform.translation.x = 0.0
            transform.transform.translation.y = 0.0
            transform.transform.translation.z = 0.0

            # Publish
            self.imu_pub.publish(output_msg)
            self.tf_broadcaster.sendTransform(transform)



def main(args=None):
    rclpy.init(args=args)

    try:
        node = ImuOrientationOffset()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
