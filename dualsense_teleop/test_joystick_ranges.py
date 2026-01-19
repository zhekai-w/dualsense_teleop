#!/usr/bin/env python3
"""Test script to determine DualSense joystick value ranges with clean display"""

from dualsense_controller import DualSenseController, Mapping, JoyStick

class JoystickTest:
    def __init__(self, use_normalized=False):
        device_infos = DualSenseController.enumerate_devices()
        if len(device_infos) < 1:
            raise Exception('No DualSense Controller available.')

        self.mapping_type = Mapping.NORMALIZED if use_normalized else Mapping.RAW

        self.controller = DualSenseController(
            device_index_or_device_info=device_infos[0],
            mapping=self.mapping_type,
            left_joystick_deadzone=0.0,   # No deadzone for testing
            right_joystick_deadzone=0.0,  # No deadzone for testing
        )

        self.controller.left_stick.on_change(self._on_left_stick)

        # Current values
        self.left_stick_x = 0.0
        self.left_stick_y = 0.0
        self.right_stick_x = 0.0
        self.right_stick_y = 0.0


    def _on_left_stick(self, stick: JoyStick):
        self.left_stick_x = stick.x
        self.left_stick_y = stick.y
        print("X:", self.left_stick_x)
        print("Y:", self.left_stick_y)
