#!/usr/bin/env python3
"""
DualSense Gripper Teleop for ROS2

Controls Robotiq gripper using DualSense R2 trigger via GripperCommand action.
R2 trigger (0.0-1.0) maps to gripper position (0.0-0.8 rad for 2F-85).
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from dualsense_controller import DeviceInfo, DualSenseController, Mapping, Number
import threading


class TeleopGripper(Node):
    def __init__(self):
        super().__init__('teleop_gripper')

        # Declare parameters for gripper limits (2F-85 defaults)
        self.declare_parameter('gripper_position_min', 0.0)    # rad (open)
        self.declare_parameter('gripper_position_max', 0.8)    # rad (closed)
        self.declare_parameter('gripper_max_effort', 50.0)    # N
        self.declare_parameter('action_name', '/gripper/robotiq_gripper_controller/gripper_cmd')
        self.declare_parameter('trigger_deadzone', 0.05)        # normalized deadzone

        # Get parameters
        self.position_min = self.get_parameter('gripper_position_min').value
        self.position_max = self.get_parameter('gripper_position_max').value
        self.max_effort = self.get_parameter('gripper_max_effort').value
        self.action_name = self.get_parameter('action_name').value
        self.trigger_deadzone = self.get_parameter('trigger_deadzone').value

        # R2 trigger range (normalized: 0.0 to 1.0)
        self.trigger_min = 0.0
        self.trigger_max = 1.0

        # State
        self.is_running = True
        self.current_trigger_value = 0
        self.last_sent_position = None
        self.goal_handle = None
        self.data_lock = threading.Lock()

        # Action client for gripper
        self.gripper_action_client = ActionClient(
            self, GripperCommand, self.action_name
        )

        self.get_logger().info(f'Waiting for action server: {self.action_name}')
        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Action server not available, continuing anyway...')
        else:
            self.get_logger().info('Action server connected')

        # Initialize DualSense controller
        self.get_logger().info('Initializing DualSense controller...')
        device_infos: list[DeviceInfo] = DualSenseController.enumerate_devices()
        if len(device_infos) < 1:
            self.get_logger().error('No DualSense Controller available!')
            raise Exception('No DualSense Controller available.')

        self.controller = DualSenseController(
            device_index_or_device_info=device_infos[0],
            mapping=Mapping.NORMALIZED,
            right_trigger_deadzone=self.trigger_deadzone,
        )

        # Register callbacks
        self.controller.right_trigger.on_change(self._on_right_trigger_change)
        self.controller.btn_ps.on_down(self._on_ps_button)
        self.controller.exceptions.on_change(self._on_exception)

        # Activate controller
        self.controller.activate()
        self.get_logger().info('DualSense controller activated')

        # Timer to send gripper commands at fixed rate
        self.timer = self.create_timer(0.05, self._send_gripper_command)  # 20 Hz

        self.get_logger().info('Teleop Gripper started')
        self.get_logger().info(f'  Position range: {self.position_min} - {self.position_max} rad')
        self.get_logger().info(f'  Max effort: {self.max_effort} N')
        self.get_logger().info('  Press R2 to control gripper, PS button to exit')

    def _on_right_trigger_change(self, value: Number):
        """Callback for R2 trigger changes."""
        with self.data_lock:
            self.current_trigger_value = value

    def _on_ps_button(self):
        """Callback for PS button - shutdown."""
        self.get_logger().info('PS button pressed, shutting down...')
        self.is_running = False
        rclpy.shutdown()

    def _on_exception(self, exception: Exception):
        """Callback for controller exceptions."""
        self.get_logger().error(f'Controller exception: {exception}')
        self.is_running = False

    def _map_trigger_to_position(self, trigger_value: float) -> float:
        """Map R2 trigger value (0.0-1.0) to gripper position."""
        # Clamp trigger value to 0-1
        normalized = max(0.0, min(1.0, trigger_value))

        # Map to gripper position (0 trigger = open, 1.0 trigger = closed)
        position = self.position_min + normalized * (self.position_max - self.position_min)
        return position

    def _send_gripper_command(self):
        """Send gripper command based on current trigger value."""
        if not self.is_running:
            return

        with self.data_lock:
            trigger_value = self.current_trigger_value

        # Calculate target position
        target_position = self._map_trigger_to_position(trigger_value)

        # Only send if position changed significantly (avoid spamming)
        if self.last_sent_position is not None:
            if abs(target_position - self.last_sent_position) < 0.01:
                return

        self.last_sent_position = target_position

        # Create and send goal
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = target_position
        goal_msg.command.max_effort = self.max_effort

        self.get_logger().debug(
            f'Sending gripper command: pos={target_position:.3f} rad, effort={self.max_effort:.1f} N'
        )

        # Send goal asynchronously
        self.gripper_action_client.send_goal_async(
            goal_msg,
        ).add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """Callback when goal is accepted/rejected."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Gripper goal rejected')
            return
        self.goal_handle = goal_handle

    def destroy_node(self):
        """Cleanup on shutdown."""
        self.get_logger().info('Shutting down Teleop Gripper...')
        self.is_running = False
        if hasattr(self, 'controller') and self.controller.is_active:
            self.controller.deactivate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = TeleopGripper()
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
