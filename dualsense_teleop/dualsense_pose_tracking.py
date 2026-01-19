#!/usr/bin/env python3
"""
DualSense Pose Tracking Node for MoveIt Servo

Reads:
  - Joystick directly from DualSense controller
  - Orientation from /imu/data (filtered by imu_filter_madgwick)

Publishes to:
  - /target_pose (geometry_msgs/PoseStamped) - target pose for MoveIt Servo pose tracking

Control mapping:
  - Left stick Y: Linear Z (up/down)
  - Right stick Y: Linear X (forward/back)
  - Right stick X: Linear Y (left/right)
  - IMU orientation (filtered): Target orientation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
from tf2_ros import TransformException
import threading
import time

from dualsense_controller import DualSenseController, Mapping, JoyStick


class DualSensePoseTracking(Node):
    def __init__(self):
        super().__init__('dualsense_pose_tracking')

        # Declare parameters
        self.declare_parameter('linear_scale', 0.3)  # m/s max velocity
        self.declare_parameter('update_rate', 30.0)  # Hz
        self.declare_parameter('planning_frame', 'panda_link')  # Base frame
        self.declare_parameter('ee_frame', 'panda_link8')  # End effector frame
        self.declare_parameter('device_index', 0)  # DualSense device index
        self.declare_parameter('joystick_deadzone', 0.15)  # Joystick deadzone
        self.declare_parameter('imu_topic', '/imu/data')  # Filtered IMU topic
        self.declare_parameter('use_normalized_mapping', True)  # Use normalized [-1, 1] or raw values

        # Get parameters
        self.linear_scale = self.get_parameter('linear_scale').value
        self.update_rate = self.get_parameter('update_rate').value
        self.planning_frame = self.get_parameter('planning_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.device_index = self.get_parameter('device_index').value
        self.joystick_deadzone = self.get_parameter('joystick_deadzone').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.use_normalized_mapping = self.get_parameter('use_normalized_mapping').value

        # State variables
        self.left_stick = JoyStick(x=0.0, y=0.0)
        self.right_stick = JoyStick(x=0.0, y=0.0)
        self.imu_orientation = None  # From filtered IMU
        self.controller_connected = False
        self.controller_running = True

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribe to filtered IMU data
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10
        )

        # Publisher
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            '/target_pose',
            10
        )

        # Initialize DualSense controller
        self.get_logger().info('Initializing DualSense controller...')
        self.controller = None
        try:
            # Enumerate devices
            device_infos = DualSenseController.enumerate_devices()
            if len(device_infos) < 1:
                raise Exception('No DualSense Controller available.')

            device_info = device_infos[self.device_index]
            self.get_logger().info(f'Found DualSense: {device_info}')

            # Create controller with appropriate mapping
            if self.use_normalized_mapping:
                self.controller = DualSenseController(
                    device_index_or_device_info=device_info,
                    mapping=Mapping.NORMALIZED,  # [-1.0, 1.0] range
                    left_joystick_deadzone=self.joystick_deadzone,
                    right_joystick_deadzone=self.joystick_deadzone,
                    left_trigger_deadzone=0.05,
                    right_trigger_deadzone=0.05,
                    gyroscope_threshold=0,
                    accelerometer_threshold=0,
                    orientation_threshold=0,
                )
            else:
                self.controller = DualSenseController(
                    device_index_or_device_info=device_info,
                    mapping=Mapping.RAW,
                    left_joystick_deadzone=5,
                    right_joystick_deadzone=5,
                    left_trigger_deadzone=1,
                    right_trigger_deadzone=1,
                    gyroscope_threshold=0,
                    accelerometer_threshold=0,
                    orientation_threshold=0,
                )

            # Register callbacks for joystick only
            self.controller.left_stick.on_change(self._on_left_stick)
            self.controller.right_stick.on_change(self._on_right_stick)
            self.controller.connection.on_change(self._on_connection_change)
            self.controller.exceptions.on_change(self._on_exception)

            # Optional: Stop on PS button
            self.controller.btn_ps.on_down(self._on_btn_ps)

            # Initialize controller in separate thread
            self.controller_thread = threading.Thread(target=self._run_controller, daemon=True)
            self.controller_thread.start()

        except Exception as e:
            self.get_logger().error(f'Failed to initialize DualSense controller: {e}')
            raise

        # Timer for publishing at fixed rate
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.publish_target_pose
        )

        self.get_logger().info('DualSense Pose Tracking Node started')
        self.get_logger().info(f'  Planning frame: {self.planning_frame}')
        self.get_logger().info(f'  EE frame: {self.ee_frame}')
        self.get_logger().info(f'  Linear scale: {self.linear_scale} m/s')
        self.get_logger().info(f'  Update rate: {self.update_rate} Hz')
        self.get_logger().info(f'  Joystick deadzone: {self.joystick_deadzone}')
        self.get_logger().info(f'  Mapping: {"NORMALIZED" if self.use_normalized_mapping else "RAW"}')
        self.get_logger().info(f'  IMU topic: {self.imu_topic}')
        self.get_logger().info('Control mapping:')
        self.get_logger().info('  - Left stick Y: Linear Z (up/down)')
        self.get_logger().info('  - Right stick Y: Linear X (forward/back)')
        self.get_logger().info('  - Right stick X: Linear Y (left/right)')
        self.get_logger().info('  - Filtered IMU orientation: Target orientation')
        self.get_logger().info('  - PS button: Emergency stop')

    def _run_controller(self):
        """Run controller in separate thread"""
        try:
            self.controller.activate()
            self.get_logger().info('DualSense controller activated')
            while self.controller_running:
                time.sleep(0.1)
            self.controller.deactivate()
            self.get_logger().info('DualSense controller deactivated')
        except Exception as e:
            self.get_logger().error(f'Error running controller: {e}')

    def _on_left_stick(self, state: JoyStick):
        """Callback for left joystick"""
        self.left_stick = state

    def _on_right_stick(self, state: JoyStick):
        """Callback for right joystick"""
        self.right_stick = state

    def _on_connection_change(self, connection):
        """Callback for connection state changes"""
        self.controller_connected = connection.connected
        if connection.connected:
            self.get_logger().info(f'DualSense connected via {connection.connection_type}')
        else:
            self.get_logger().warn('DualSense disconnected!')

    def _on_exception(self, exception: Exception):
        """Callback for exceptions"""
        self.get_logger().error(f'DualSense exception: {exception}')

    def _on_btn_ps(self):
        """Emergency stop on PS button"""
        self.get_logger().warn('PS button pressed - Emergency stop!')
        # Optionally stop the node
        # self.controller_running = False

    def imu_callback(self, msg: Imu):
        """Callback for filtered IMU data from imu_filter_madgwick"""
        self.imu_orientation = msg.orientation

    def get_current_ee_pose(self) -> PoseStamped:
        """Get the current end-effector pose via TF2"""
        try:
            # Look up the transform from planning frame to EE frame
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.planning_frame,
                self.ee_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            # Convert transform to pose
            pose = PoseStamped()
            pose.header.frame_id = self.planning_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation

            return pose

        except TransformException as ex:
            self.get_logger().warn(
                f'Could not get EE transform: {ex}',
                throttle_duration_sec=2.0
            )
            return None

    def publish_target_pose(self):
        """Main loop: compute and publish target pose"""

        # Check if controller is connected
        if not self.controller_connected:
            self.get_logger().warn(
                'DualSense controller not connected...',
                throttle_duration_sec=3.0
            )
            return

        # Check if we have IMU orientation
        if self.imu_orientation is None:
            self.get_logger().warn(
                'Waiting for filtered IMU data...',
                throttle_duration_sec=2.0
            )
            return

        # Get current end-effector pose
        current_ee_pose = self.get_current_ee_pose()
        if current_ee_pose is None:
            return

        # Create target pose starting from current EE pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.planning_frame
        target_pose.header.stamp = self.get_clock().now().to_msg()

        # Extract joystick values
        # If normalized: already in [-1.0, 1.0]
        # If raw: need to normalize from raw values (typically 0-255 centered at 127)
        if self.use_normalized_mapping:
            # Already normalized to [-1.0, 1.0]
            left_stick_y = -self.left_stick.y   # Up = positive Z (invert Y axis)
            right_stick_x = self.right_stick.x  # Right = positive Y
            right_stick_y = -self.right_stick.y # Forward = positive X (invert Y axis)
        else:
            # Raw values: normalize from [0, 255] to [-1.0, 1.0]
            # Center is at 127
            left_stick_y = -(self.left_stick.y - 127) / 127.0
            right_stick_x = (self.right_stick.x - 127) / 127.0
            right_stick_y = -(self.right_stick.y - 127) / 127.0

        # Calculate position delta based on joystick input
        # Scale by linear_scale and time step
        dt = 1.0 / self.update_rate
        delta_x = right_stick_y * self.linear_scale * dt
        delta_y = right_stick_x * self.linear_scale * dt
        delta_z = left_stick_y * self.linear_scale * dt

        # Add delta to current position
        target_pose.pose.position.x = current_ee_pose.pose.position.x + delta_x
        target_pose.pose.position.y = current_ee_pose.pose.position.y + delta_y
        target_pose.pose.position.z = current_ee_pose.pose.position.z + delta_z

        # Use filtered IMU orientation (from imu_filter_madgwick)
        target_pose.pose.orientation = self.imu_orientation

        # Publish the target pose
        self.target_pose_pub.publish(target_pose)

    def shutdown(self):
        """Clean shutdown of DualSense controller"""
        self.get_logger().info('Shutting down DualSense controller...')
        self.controller_running = False
        if self.controller_thread.is_alive():
            self.controller_thread.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = DualSensePoseTracking()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
