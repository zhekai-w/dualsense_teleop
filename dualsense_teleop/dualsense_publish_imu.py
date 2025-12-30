#!/usr/bin/env python3
"""
DualSense IMU Publisher for ROS2

Publishes calibrated IMU data from DualSense controller to ROS2
as sensor_msgs/msg/Imu on topic "imu/data_raw"
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from dualsense_controller import DualSenseController, Gyroscope, Accelerometer
import json
import os
import threading
import numpy as np


class DualSenseImuPublisher(Node):
    def __init__(self):
        super().__init__('dualsense_imu_publisher')
        
        # Declare rotation parameters (in degrees)
        self.declare_parameter('rotation_x_deg', 90.0)
        self.declare_parameter('rotation_y_deg', 180.0)
        self.declare_parameter('rotation_z_deg', 0.0)
        
        # Get rotation angles
        rx_deg = self.get_parameter('rotation_x_deg').value
        ry_deg = self.get_parameter('rotation_y_deg').value
        rz_deg = self.get_parameter('rotation_z_deg').value
        
        # Convert to radians
        rx_rad = np.radians(rx_deg)
        ry_rad = np.radians(ry_deg)
        rz_rad = np.radians(rz_deg)
        
        # Publisher for IMU data
        self.imu_publisher = self.create_publisher(Imu, 'imu/data_raw', 10)
        
        # Load calibration
        self.calibration = None
        self.load_calibration()
        
        # Create rotation matrices
        # Rx(angle) rotation matrix
        rx = np.array([[1,  0,                 0],
                       [0,  np.cos(rx_rad), -np.sin(rx_rad)],
                       [0,  np.sin(rx_rad),  np.cos(rx_rad)]], dtype=float)
        
        # Ry(angle) rotation matrix
        ry = np.array([[np.cos(ry_rad),   0,  np.sin(ry_rad)],
                       [0,                 1,  0],
                       [-np.sin(ry_rad),  0,  np.cos(ry_rad)]], dtype=float)
        
        # Rz(angle) rotation matrix
        rz = np.array([[np.cos(rz_rad), -np.sin(rz_rad),  0],
                       [np.sin(rz_rad),  np.cos(rz_rad),  0],
                       [0,               0,                1]], dtype=float)
        
        # Combined rotation: R = Rz * Ry * Rx
        self.rotation_matrix = rz @ ry @ rx
        self.get_logger().info(f'Applying rotation: X={rx_deg:.1f}°, Y={ry_deg:.1f}°, Z={rz_deg:.1f}°')
        
        # Thread lock for data access
        self.data_lock = threading.Lock()
        
        # Latest sensor data
        self.latest_gyro = None
        self.latest_accel = None
        
        # Initialize DualSense controller
        self.get_logger().info('Initializing DualSense controller...')
        device_infos = DualSenseController.enumerate_devices()
        if len(device_infos) < 1:
            self.get_logger().error('No DualSense Controller available!')
            raise Exception('No DualSense Controller available.')
        
        self.controller = DualSenseController()
        
        # Register callbacks
        self.controller.gyroscope.on_change(self._on_gyroscope_change)
        self.controller.accelerometer.on_change(self._on_accelerometer_change)
        
        # Activate controller
        self.controller.activate()
        self.get_logger().info('DualSense controller activated')
        
        # Timer to publish IMU data at a fixed rate
        self.timer = self.create_timer(0.01, self.publish_imu)  # 100 Hz
        
        self.get_logger().info('DualSense IMU Publisher started')
        self.get_logger().info(f'Publishing to topic: imu/data_raw')
        
    def load_calibration(self):
        """Load calibration from JSON file."""
        # Get the directory where this script is located
        script_dir = os.path.dirname(os.path.abspath(__file__))
        calibration_file = os.path.join(script_dir, 'dualsense_calibration', 'dualsense_calibration_simple.json')
        
        try:
            with open(calibration_file, 'r') as f:
                self.calibration = json.load(f)
            self.get_logger().info(f'✓ Loaded calibration from: {calibration_file}')
            self.get_logger().info(f'  Accel scale: {self.calibration["accelerometer_y"]["scale"]:.2f} LSB/(m/s²)')
            self.get_logger().info(f'  Accel variance: {self.calibration["accelerometer_y"]["variance"]:.6f} (m/s²)²')
            self.get_logger().info(f'  Gyro scale: {self.calibration["gyroscope_y"]["scale"]:.2f} LSB/(rad/s)')
            self.get_logger().info(f'  Gyro variance: {self.calibration["gyroscope_y"]["variance"]:.8f} (rad/s)²')
        except FileNotFoundError:
            self.get_logger().error(f'✗ Calibration file not found: {calibration_file}')
            self.get_logger().error('  Please run calibrate_simple.py first!')
            raise
        except Exception as e:
            self.get_logger().error(f'✗ Error loading calibration: {e}')
            raise
    
    def _on_gyroscope_change(self, gyroscope: Gyroscope):
        """Callback for gyroscope data."""
        with self.data_lock:
            self.latest_gyro = gyroscope
    
    def _on_accelerometer_change(self, accelerometer: Accelerometer):
        """Callback for accelerometer data."""
        with self.data_lock:
            self.latest_accel = accelerometer
    
    def convert_gyro_to_rad_per_sec(self, gyro: Gyroscope):
        """Convert raw gyro values to rad/s using calibration."""
        scale = self.calibration['gyroscope_y']['scale']
        bias = self.calibration['gyroscope_y']['bias']
        
        # Apply calibration (output is in rad/s)
        x_rps = (gyro.x - bias) / scale
        y_rps = (gyro.y - bias) / scale
        z_rps = (gyro.z - bias) / scale
        
        return x_rps, y_rps, z_rps
    
    def convert_accel_to_m_per_s2(self, accel: Accelerometer):
        """Convert raw accel values to m/s² using calibration."""
        scale = self.calibration['accelerometer_y']['scale']
        
        # Apply calibration (output is in m/s²)
        x_ms2 = accel.x / scale
        y_ms2 = accel.y / scale
        z_ms2 = accel.z / scale
        
        return x_ms2, y_ms2, z_ms2
    
    def publish_imu(self):
        """Publish IMU message."""
        with self.data_lock:
            if self.latest_gyro is None or self.latest_accel is None:
                return
            
            gyro = self.latest_gyro
            accel = self.latest_accel
        
        # Create IMU message
        imu_msg = Imu()
        
        # Header
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'dualsense_imu_frame'
        
        # Convert and fill angular velocity (rad/s)
        x_rps, y_rps, z_rps = self.convert_gyro_to_rad_per_sec(gyro)
        
        # Apply rotation matrix to angular velocity
        gyro_vec = np.array([x_rps, y_rps, z_rps])
        gyro_rotated = self.rotation_matrix @ gyro_vec
        
        imu_msg.angular_velocity.x = float(gyro_rotated[0])
        imu_msg.angular_velocity.y = float(gyro_rotated[1])
        imu_msg.angular_velocity.z = float(gyro_rotated[2])
        
        # Fill angular velocity covariance (diagonal matrix from calibration)
        gyro_covariance = self.calibration['gyroscope_y']['covariance']
        imu_msg.angular_velocity_covariance = [float(x) for x in gyro_covariance]
        
        # Convert and fill linear acceleration (m/s²)
        x_ms2, y_ms2, z_ms2 = self.convert_accel_to_m_per_s2(accel)
        
        # Apply rotation matrix to linear acceleration
        accel_vec = np.array([x_ms2, y_ms2, z_ms2])
        accel_rotated = self.rotation_matrix @ accel_vec
        
        imu_msg.linear_acceleration.x = float(accel_rotated[0])
        imu_msg.linear_acceleration.y = float(accel_rotated[1])
        imu_msg.linear_acceleration.z = float(accel_rotated[2])
        
        # Fill linear acceleration covariance (diagonal matrix from calibration)
        accel_covariance = self.calibration['accelerometer_y']['covariance']
        imu_msg.linear_acceleration_covariance = [float(x) for x in accel_covariance]
        
        # Orientation is not available from DualSense raw data
        # Set first element to -1 to indicate unknown orientation
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 0.0
        imu_msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Publish
        self.imu_publisher.publish(imu_msg)
    
    def destroy_node(self):
        """Cleanup on shutdown."""
        self.get_logger().info('Shutting down DualSense IMU Publisher...')
        if self.controller.is_active:
            self.controller.deactivate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DualSenseImuPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
