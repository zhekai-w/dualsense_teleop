#!/usr/bin/env python3
"""
DualSense Joy Publisher for ROS2

Publishes all DualSense controller inputs (buttons, triggers, sticks) to sensor_msgs/Joy topic.

Button mapping (buttons array index):
  0: btn_cross        8: btn_l1           16: btn_touchpad
  1: btn_circle       9: btn_r1
  2: btn_triangle    10: btn_l2
  3: btn_square      11: btn_r2
  4: btn_up          12: btn_l3
  5: btn_down        13: btn_r3
  6: btn_left        14: btn_ps
  7: btn_right       15: btn_create
                     16: btn_options
                     17: btn_mute
                     18: btn_touchpad

Axes mapping (axes array index):
  0: left_stick_x    (-1.0 left, 1.0 right)
  1: left_stick_y    (-1.0 up, 1.0 down)
  2: right_stick_x   (-1.0 left, 1.0 right)
  3: right_stick_y   (-1.0 up, 1.0 down)
  4: left_trigger    (0.0 released, 1.0 fully pressed)
  5: right_trigger   (0.0 released, 1.0 fully pressed)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from dualsense_controller import DeviceInfo, DualSenseController, Mapping
import threading


class DualSenseJoyPublisher(Node):
    # Button indices
    BTN_CROSS = 0
    BTN_CIRCLE = 1
    BTN_TRIANGLE = 2
    BTN_SQUARE = 3
    BTN_UP = 4
    BTN_DOWN = 5
    BTN_LEFT = 6
    BTN_RIGHT = 7
    BTN_L1 = 8
    BTN_R1 = 9
    BTN_L2 = 10
    BTN_R2 = 11
    BTN_L3 = 12
    BTN_R3 = 13
    BTN_PS = 14
    BTN_CREATE = 15
    BTN_OPTIONS = 16
    BTN_MUTE = 17
    BTN_TOUCHPAD = 18
    NUM_BUTTONS = 19

    # Axes indices
    AXIS_LEFT_STICK_X = 0
    AXIS_LEFT_STICK_Y = 1
    AXIS_RIGHT_STICK_X = 2
    AXIS_RIGHT_STICK_Y = 3
    AXIS_LEFT_TRIGGER = 4
    AXIS_RIGHT_TRIGGER = 5
    NUM_AXES = 6

    def __init__(self):
        super().__init__('dualsense_joy_publisher')

        # Declare parameters
        self.declare_parameter('publish_rate', 100.0)  # Hz
        self.declare_parameter('frame_id', 'dualsense')
        self.declare_parameter('joystick_deadzone', 0.05)
        self.declare_parameter('trigger_deadzone', 0.02)

        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.joystick_deadzone = self.get_parameter('joystick_deadzone').value
        self.trigger_deadzone = self.get_parameter('trigger_deadzone').value

        # State
        self.is_running = True
        self.data_lock = threading.Lock()

        # Initialize button and axes arrays
        self.buttons = [0] * self.NUM_BUTTONS
        self.axes = [0.0] * self.NUM_AXES

        # Publisher
        self.joy_publisher = self.create_publisher(Joy, 'joy', 10)

        # Initialize DualSense controller
        self.get_logger().info('Initializing DualSense controller...')
        device_infos: list[DeviceInfo] = DualSenseController.enumerate_devices()
        if len(device_infos) < 1:
            self.get_logger().error('No DualSense Controller available!')
            raise Exception('No DualSense Controller available.')

        self.controller = DualSenseController(
            device_index_or_device_info=device_infos[0],
            mapping=Mapping.NORMALIZED,
            left_joystick_deadzone=self.joystick_deadzone,
            right_joystick_deadzone=self.joystick_deadzone,
            left_trigger_deadzone=self.trigger_deadzone,
            right_trigger_deadzone=self.trigger_deadzone,
        )

        # Register button callbacks
        self._register_button_callbacks()

        # Register axes callbacks
        self._register_axes_callbacks()

        # Register exception handler
        self.controller.exceptions.on_change(self._on_exception)

        # Activate controller
        self.controller.activate()
        self.get_logger().info('DualSense controller activated')

        # Timer to publish at fixed rate
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_joy)

        self.get_logger().info(f'DualSense Joy Publisher started at {self.publish_rate} Hz')
        self.get_logger().info(f'Publishing to topic: joy')

    def _register_button_callbacks(self):
        """Register callbacks for all buttons."""
        # Symbol buttons
        self.controller.btn_cross.on_change(
            lambda pressed: self._on_button_change(self.BTN_CROSS, pressed))
        self.controller.btn_circle.on_change(
            lambda pressed: self._on_button_change(self.BTN_CIRCLE, pressed))
        self.controller.btn_triangle.on_change(
            lambda pressed: self._on_button_change(self.BTN_TRIANGLE, pressed))
        self.controller.btn_square.on_change(
            lambda pressed: self._on_button_change(self.BTN_SQUARE, pressed))

        # D-pad buttons
        self.controller.btn_up.on_change(
            lambda pressed: self._on_button_change(self.BTN_UP, pressed))
        self.controller.btn_down.on_change(
            lambda pressed: self._on_button_change(self.BTN_DOWN, pressed))
        self.controller.btn_left.on_change(
            lambda pressed: self._on_button_change(self.BTN_LEFT, pressed))
        self.controller.btn_right.on_change(
            lambda pressed: self._on_button_change(self.BTN_RIGHT, pressed))

        # Shoulder buttons
        self.controller.btn_l1.on_change(
            lambda pressed: self._on_button_change(self.BTN_L1, pressed))
        self.controller.btn_r1.on_change(
            lambda pressed: self._on_button_change(self.BTN_R1, pressed))
        self.controller.btn_l2.on_change(
            lambda pressed: self._on_button_change(self.BTN_L2, pressed))
        self.controller.btn_r2.on_change(
            lambda pressed: self._on_button_change(self.BTN_R2, pressed))

        # Stick buttons
        self.controller.btn_l3.on_change(
            lambda pressed: self._on_button_change(self.BTN_L3, pressed))
        self.controller.btn_r3.on_change(
            lambda pressed: self._on_button_change(self.BTN_R3, pressed))

        # System buttons
        self.controller.btn_ps.on_change(
            lambda pressed: self._on_button_change(self.BTN_PS, pressed))
        self.controller.btn_create.on_change(
            lambda pressed: self._on_button_change(self.BTN_CREATE, pressed))
        self.controller.btn_options.on_change(
            lambda pressed: self._on_button_change(self.BTN_OPTIONS, pressed))
        self.controller.btn_mute.on_change(
            lambda pressed: self._on_button_change(self.BTN_MUTE, pressed))
        self.controller.btn_touchpad.on_change(
            lambda pressed: self._on_button_change(self.BTN_TOUCHPAD, pressed))

    def _register_axes_callbacks(self):
        """Register callbacks for all axes (sticks and triggers)."""
        # Left stick
        self.controller.left_stick_x.on_change(
            lambda value: self._on_axis_change(self.AXIS_LEFT_STICK_X, value))
        self.controller.left_stick_y.on_change(
            lambda value: self._on_axis_change(self.AXIS_LEFT_STICK_Y, value))

        # Right stick
        self.controller.right_stick_x.on_change(
            lambda value: self._on_axis_change(self.AXIS_RIGHT_STICK_X, value))
        self.controller.right_stick_y.on_change(
            lambda value: self._on_axis_change(self.AXIS_RIGHT_STICK_Y, value))

        # Triggers
        self.controller.left_trigger.on_change(
            lambda value: self._on_axis_change(self.AXIS_LEFT_TRIGGER, value))
        self.controller.right_trigger.on_change(
            lambda value: self._on_axis_change(self.AXIS_RIGHT_TRIGGER, value))

    def _on_button_change(self, button_index: int, pressed: bool):
        """Callback for button state changes."""
        with self.data_lock:
            self.buttons[button_index] = 1 if pressed else 0

    def _on_axis_change(self, axis_index: int, value: float):
        """Callback for axis value changes."""
        with self.data_lock:
            self.axes[axis_index] = float(value)

    def _on_exception(self, exception: Exception):
        """Callback for controller exceptions."""
        self.get_logger().error(f'Controller exception: {exception}')
        self.is_running = False

    def _publish_joy(self):
        """Publish current controller state as Joy message."""
        if not self.is_running:
            return

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        with self.data_lock:
            msg.axes = self.axes.copy()
            msg.buttons = self.buttons.copy()

        self.joy_publisher.publish(msg)

    def destroy_node(self):
        """Cleanup on shutdown."""
        self.get_logger().info('Shutting down DualSense Joy Publisher...')
        self.is_running = False
        if hasattr(self, 'controller') and self.controller.is_active:
            self.controller.deactivate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = DualSenseJoyPublisher()
        while node.is_running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
