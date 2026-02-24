#!/usr/bin/env python3
"""
Test script for DualSense adaptive trigger with different mappings.

Tests continuous_resistance with both NORMALIZED and RAW mappings
to determine the correct value ranges.
"""

import time
from dualsense_controller import DeviceInfo, DualSenseController, Mapping


def test_normalized_mapping():
    """Test adaptive trigger with NORMALIZED mapping.
    
    Note: Mapping only affects INPUT (trigger readings).
    OUTPUT (adaptive trigger effects) always uses RAW values (0-255).
    """
    print("\n" + "=" * 60)
    print("Testing NORMALIZED mapping")
    print("Note: Adaptive trigger OUTPUT always uses 0-255 (RAW)")
    print("=" * 60)

    device_infos = DualSenseController.enumerate_devices()
    if len(device_infos) < 1:
        print("No DualSense Controller available!")
        return

    controller = DualSenseController(
        device_index_or_device_info=device_infos[0],
        mapping=Mapping.NORMALIZED,
    )
    controller.activate()
    print("Controller activated with NORMALIZED mapping")

    try:
        # Test 1: No resistance
        print("\n[Test 1] No resistance")
        controller.right_trigger.effect.no_resistance()
        input("Press Enter to continue...")

        # Test 2: Full resistance from start (raw values for output)
        print("\n[Test 2] Full resistance from start (start=0, force=255)")
        controller.right_trigger.effect.continuous_resistance(
            start_position=0,
            force=255
        )
        input("Press Enter to continue...")

        # Test 3: Medium resistance from start
        print("\n[Test 3] Medium resistance from start (start=0, force=128)")
        controller.right_trigger.effect.continuous_resistance(
            start_position=0,
            force=128
        )
        input("Press Enter to continue...")

        # Test 4: Full resistance from middle
        print("\n[Test 4] Full resistance from middle (start=127, force=255)")
        controller.right_trigger.effect.continuous_resistance(
            start_position=127,
            force=255
        )
        input("Press Enter to continue...")

        # Test 5: Light resistance
        print("\n[Test 5] Light resistance (start=0, force=50)")
        controller.right_trigger.effect.continuous_resistance(
            start_position=0,
            force=50
        )
        input("Press Enter to continue...")

        # Reset
        print("\n[Reset] No resistance")
        controller.right_trigger.effect.no_resistance()

    finally:
        controller.right_trigger.effect.no_resistance()
        controller.deactivate()
        print("Controller deactivated")


def test_raw_mapping():
    """Test adaptive trigger with RAW mapping (0-255)."""
    print("\n" + "=" * 60)
    print("Testing RAW mapping (values 0 - 255)")
    print("=" * 60)

    device_infos = DualSenseController.enumerate_devices()
    if len(device_infos) < 1:
        print("No DualSense Controller available!")
        return

    controller = DualSenseController(
        device_index_or_device_info=device_infos[0],
        mapping=Mapping.RAW,
    )
    controller.activate()
    print("Controller activated with RAW mapping")

    try:
        # Test 1: No resistance
        print("\n[Test 1] No resistance")
        controller.right_trigger.effect.no_resistance()
        input("Press Enter to continue...")

        # Test 2: Full resistance from start (raw values)
        print("\n[Test 2] Full resistance from start (start=0, force=255)")
        controller.right_trigger.effect.continuous_resistance(
            start_position=0,
            force=255
        )
        input("Press Enter to continue...")

        # Test 3: Medium resistance from start
        print("\n[Test 3] Medium resistance from start (start=0, force=128)")
        controller.right_trigger.effect.continuous_resistance(
            start_position=0,
            force=128
        )
        input("Press Enter to continue...")

        # Test 4: Full resistance from middle
        print("\n[Test 4] Full resistance from middle (start=127, force=255)")
        controller.right_trigger.effect.continuous_resistance(
            start_position=127,
            force=255
        )
        input("Press Enter to continue...")

        # Test 5: Light resistance
        print("\n[Test 5] Light resistance (start=0, force=50)")
        controller.right_trigger.effect.continuous_resistance(
            start_position=0,
            force=50
        )
        input("Press Enter to continue...")

        # Reset
        print("\n[Reset] No resistance")
        controller.right_trigger.effect.no_resistance()

    finally:
        controller.right_trigger.effect.no_resistance()
        controller.deactivate()
        print("Controller deactivated")


def main():
    print("DualSense Adaptive Trigger Test")
    print("================================")
    print("\nThis script tests the adaptive trigger with different mappings.")
    print("Press the R2 trigger after each test to feel the resistance.\n")

    while True:
        print("\nSelect test:")
        print("  1. Test NORMALIZED mapping (0.0 - 1.0)")
        print("  2. Test RAW mapping (0 - 255)")
        print("  3. Run both tests")
        print("  q. Quit")

        choice = input("\nEnter choice: ").strip().lower()

        if choice == '1':
            test_normalized_mapping()
        elif choice == '2':
            test_raw_mapping()
        elif choice == '3':
            test_normalized_mapping()
            time.sleep(1)
            test_raw_mapping()
        elif choice == 'q':
            print("Goodbye!")
            break
        else:
            print("Invalid choice, try again.")


if __name__ == '__main__':
    main()
