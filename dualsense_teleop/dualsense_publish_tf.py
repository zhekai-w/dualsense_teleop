import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from pydualsense import pydualsense
import math
import time
import numpy as np


class Quat:
    """Simple quaternion for orientation tracking"""
    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.w, self.x, self.y, self.z = w, x, y, z
    
    def normalize(self):
        norm = math.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)
        if norm > 0:
            self.w /= norm
            self.x /= norm
            self.y /= norm
            self.z /= norm
    
    def multiply(self, other):
        """Quaternion multiplication"""
        return Quat(
            self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
        )
    
    def conjugate(self):
        """Inverse for unit quaternion"""
        return Quat(self.w, -self.x, -self.y, -self.z)
    
    def rotate_vector(self, v):
        """Rotate a numpy vector by this quaternion"""
        qv = Quat(0, v[0], v[1], v[2])
        result = self.multiply(qv).multiply(self.conjugate())
        return np.array([result.x, result.y, result.z])


def angle_axis_to_quat(angle, axis):
    """Convert angle-axis rotation to quaternion. Axis should be numpy array."""
    half_angle = angle * 0.5
    s = math.sin(half_angle)
    # Normalize axis
    axis_norm = np.linalg.norm(axis)
    if axis_norm > 0:
        axis = axis / axis_norm
    return Quat(math.cos(half_angle), axis[0] * s, axis[1] * s, axis[2] * s)


class DualSenseTFPublisher(Node):
    """
    Publishes DualSense controller orientation using gyro + accelerometer sensor fusion.
    """
    def __init__(self):
        super().__init__('dualsense_tf_publisher')

        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize DualSense controller
        self.ds = pydualsense()
        self.ds.init()
        self.get_logger().info('DualSense controller initialized')
        
        # Parameters
        self.declare_parameter('frame_id', 'dualsense_controller')
        self.declare_parameter('parent_frame', 'world')
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('gravity_correction_speed', 1.0)
        
        self.frame_id = self.get_parameter('frame_id').value
        self.parent_frame = self.get_parameter('parent_frame').value
        rate = self.get_parameter('publish_rate').value
        self.gravity_correction_speed = self.get_parameter('gravity_correction_speed').value
        
        # Orientation state
        self.orientation = Quat()
        self.gravity_estimate = np.array([0.0, 0.0, -1.0])  # Initial gravity pointing down
        
        # Gyro calibration (bias removal)
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.is_calibrated = False
        
        # Timing
        self.last_time = self.get_clock().now()
        
        # Wait for controller to initialize
        time.sleep(0.5)
        
        # Calibrate gyro bias
        self.calibrate_gyro()
        
        # Create timer for publishing TF
        self.timer = self.create_timer(1.0 / rate, self.publish_tf)
        
        self.get_logger().info('Started publishing orientation')
        
    def calibrate_gyro(self):
        """Calibrate gyro bias by averaging samples while still"""
        self.get_logger().info('Calibrating gyro... keep controller still')
        samples = []
        for _ in range(100):
            self.ds.state
            gyro_scale = 2000.0 / 32768.0
            samples.append(np.array([
                self.ds.state.gyro.Roll * gyro_scale,
                self.ds.state.gyro.Pitch * gyro_scale,
                self.ds.state.gyro.Yaw * gyro_scale
            ]))
            time.sleep(0.01)
        
        # Average all samples
        self.gyro_bias = np.mean(samples, axis=0)
        self.is_calibrated = True
        self.get_logger().info(f'Gyro bias: [{self.gyro_bias[0]:.2f}, {self.gyro_bias[1]:.2f}, {self.gyro_bias[2]:.2f}] deg/s')
        
    def publish_tf(self):
        """Read sensors, integrate orientation, apply gravity correction, publish TF"""
        self.ds.state
        
        # Calculate delta time
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0 or dt > 1.0:
            return
        
        # Read and scale sensors
        gyro_scale = 50.0 / 32768.0
        accel_scale = 4.0 / 32768.0
        
        # Gyro (degrees/sec) with bias removal
        gyro = np.array([
            self.ds.state.gyro.Roll * gyro_scale,
            self.ds.state.gyro.Pitch * gyro_scale,
            self.ds.state.gyro.Yaw * gyro_scale
        ]) - self.gyro_bias
        
        # Accel (g's)
        accel = np.array([
            self.ds.state.accelerometer.X * accel_scale,
            self.ds.state.accelerometer.Y * accel_scale,
            self.ds.state.accelerometer.Z * accel_scale
        ])
        
        # 1. Integrate gyro to update orientation
        gyro_rad = np.radians(gyro)
        angle_speed = np.linalg.norm(gyro_rad)
        
        if angle_speed > 0:
            angle = angle_speed * dt
            rotation = angle_axis_to_quat(angle, gyro_rad)
            self.orientation = self.orientation.multiply(rotation)
        
        # 2. Update gravity estimate by rotating it with orientation change
        self.gravity_estimate = self.orientation.conjugate().rotate_vector(self.gravity_estimate)
        
        # 3. Correct gravity using accelerometer (sensor fusion)
        accel_length = np.linalg.norm(accel)
        if accel_length > 0.5:  # Only trust accel if reasonable magnitude
            accel_norm = accel / accel_length
            
            # Expected gravity direction (accel points opposite to gravity)
            expected_gravity = -accel_norm
            
            # Gradually correct gravity estimate toward measured direction
            correction_amount = self.gravity_correction_speed * dt
            self.gravity_estimate += (expected_gravity - self.gravity_estimate) * correction_amount
            self.gravity_estimate = self.gravity_estimate / np.linalg.norm(self.gravity_estimate)
            
            # 4. Apply correction to orientation based on gravity error
            gravity_world = self.orientation.rotate_vector(self.gravity_estimate)
            down = np.array([0.0, 0.0, -1.0])
            
            dot_product = np.dot(gravity_world, down)
            error_angle = math.acos(max(-1, min(1, dot_product)))
            if error_angle > 0.001:  # Only correct if significant error
                correction_axis = np.cross(down, gravity_world)
                correction = angle_axis_to_quat(error_angle * correction_amount, correction_axis)
                self.orientation = self.orientation.multiply(correction)
        
        # 5. Normalize quaternion to prevent drift
        self.orientation.normalize()
        
        # Publish TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.frame_id
        
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        t.transform.rotation.w = self.orientation.w
        t.transform.rotation.x = self.orientation.x
        t.transform.rotation.y = self.orientation.y
        t.transform.rotation.z = self.orientation.z
        
        self.tf_broadcaster.sendTransform(t)
        
    def destroy_node(self):
        self.ds.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DualSenseTFPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()