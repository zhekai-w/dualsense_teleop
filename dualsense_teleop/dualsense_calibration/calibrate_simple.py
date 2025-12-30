#!/usr/bin/env python3
"""
Simple Single-Axis Calibration for DualSense IMU

Only calibrates Y-axis (most useful for typical controller usage):
- Accelerometer: Measures gravity when controller is flat
- Gyroscope: Measures 90° rotation around vertical Y-axis
"""

import numpy as np
import time
from dualsense_controller import DualSenseController, Gyroscope, Accelerometer
import matplotlib.pyplot as plt
from typing import List, Dict
import json


class SimpleCalibrator:
    def __init__(self):
        self.gyro_samples: List[Gyroscope] = []
        self.accel_samples: List[Accelerometer] = []

        # Initialize controller
        device_infos = DualSenseController.enumerate_devices()
        if len(device_infos) < 1:
            raise Exception('No DualSense Controller available.')

        self.controller = DualSenseController()

        # Register callbacks
        self.controller.gyroscope.on_change(self._on_gyroscope)
        self.controller.accelerometer.on_change(self._on_accelerometer)

    def _on_gyroscope(self, gyro: Gyroscope):
        self.gyro_samples.append(gyro)

    def _on_accelerometer(self, accel: Accelerometer):
        self.accel_samples.append(accel)

    def calibrate_accel_y(self, duration: float = 1.0) -> Dict[str, float]:
        """
        Calibrate accelerometer Y-axis by measuring gravity.
        Controller should be lying flat with buttons facing up.

        Args:
            duration: How long to collect data (seconds)

        Returns:
            Dictionary with scale factor for Y-axis
        """
        print("\n" + "="*70)
        print("ACCELEROMETER Y-AXIS CALIBRATION")
        print("="*70)
        print("Instructions:")
        print("  1. Place controller FLAT on table")
        print("  2. Buttons should be facing UP (normal position)")
        print("  3. Y-axis will point UP and measure 1g from gravity")
        print(f"  4. Data will be collected for {duration} second(s)")
        print()

        input("Press Enter when controller is in position...")

        if not self.controller.is_active:
            self.controller.activate()

        self.accel_samples.clear()
        print(f"Collecting data for {duration} seconds...")
        time.sleep(duration)

        # Get Y-axis values
        accel_y_values = [a.y for a in self.accel_samples]

        # Calculate average (this represents 1g = 9.80665 m/s²)
        avg_y = np.mean(accel_y_values)
        std_y = np.std(accel_y_values)

        # Scale factor: LSB per m/s²
        # Since we're measuring 1g = 9.80665 m/s², divide by that
        scale_y = avg_y / 9.80665

        # Calculate variance in m/s² units
        # std_y is in LSB, convert to m/s² by dividing by scale
        std_ms2 = std_y / scale_y
        variance_ms2 = std_ms2 ** 2

        print(f"\nResults:")
        print(f"  Samples collected: {len(accel_y_values)}")
        print(f"  Y-axis average: {avg_y:.2f} LSB")
        print(f"  Y-axis std dev: {std_y:.2f} LSB ({std_ms2:.4f} m/s²)")
        print(f"  Variance: {variance_ms2:.6f} m/s²²")
        print(f"  Scale factor: {scale_y:.2f} LSB/(m/s²)")
        print(f"  → To convert Y to m/s²: raw_y / {scale_y:.2f}")

        return {
            'scale_y': scale_y,
            'avg_y': avg_y,
            'std_y': std_y,
            'std_ms2': std_ms2,
            'variance_ms2': variance_ms2,
            'num_samples': len(accel_y_values)
        }

    def calibrate_gyro_y(self, rotation_angle: float = 90.0) -> Dict[str, float]:
        """
        Calibrate gyroscope Y-axis by measuring a rotation.
        User rotates controller 90° around Y-axis (yaw).

        Args:
            rotation_angle: Expected rotation angle in degrees (default: 90°)

        Returns:
            Dictionary with scale factor for Y-axis
        """
        print("\n" + "="*70)
        print("GYROSCOPE Y-AXIS CALIBRATION")
        print("="*70)
        print("Instructions:")
        print("  1. Place controller FLAT on table, buttons facing UP")
        print("  2. Y-axis points straight UP (vertical)")
        print(f"  3. You will rotate {rotation_angle}° around Y-axis (like turning a dial)")
        print("  4. Start with controller still")
        print("  5. When ready, rotate smoothly in ~1 second")
        print("  6. Try to maintain constant rotation speed")
        print()
        print("Tip: Imagine the controller is a volume knob, turn it 90°")
        print()

        input("Press Enter when ready to start...")

        if not self.controller.is_active:
            self.controller.activate()

        # Initial noise measurement (to detect motion start/stop)
        print("Measuring noise level (keep still)...")
        self.gyro_samples.clear()
        time.sleep(1.0)

        noise_values = [g.y for g in self.gyro_samples]
        noise_threshold = np.std(noise_values) * 3  # 3-sigma threshold
        noise_mean = np.mean(noise_values)

        print(f"  Noise mean: {noise_mean:.2f}")
        print(f"  Noise threshold: ±{noise_threshold:.2f}")
        print()

        # Now collect rotation data
        print("Get ready to rotate...")
        print("Starting data collection in 2 seconds...")
        time.sleep(2)

        self.gyro_samples.clear()
        print("ROTATE NOW! (smoothly, ~1 second)")

        # Collect data for a bit longer to ensure we catch the full rotation
        time.sleep(3.0)

        # Process the data
        gyro_y_values = np.array([g.y for g in self.gyro_samples])
        time_indices = np.arange(len(gyro_y_values))

        # Detect motion start and stop
        # Motion is detected when abs(value - noise_mean) > noise_threshold
        motion_mask = np.abs(gyro_y_values - noise_mean) > noise_threshold

        # Find first and last motion indices
        motion_indices = np.where(motion_mask)[0]

        if len(motion_indices) == 0:
            print("ERROR: No motion detected! Try rotating faster or with more force.")
            return None

        start_idx = motion_indices[0]
        stop_idx = motion_indices[-1]

        # Extract rotation segment
        rotation_segment = gyro_y_values[start_idx:stop_idx+1]
        num_samples = len(rotation_segment)

        # Calculate sample rate from total collection time
        total_time = 3.0  # Total collection time in seconds
        sample_rate = len(gyro_y_values) / total_time

        # Calculate actual rotation duration
        duration = num_samples / sample_rate

        # Integrate to get total rotation in raw units
        # The integral of angular velocity over time gives angle
        # angle = sum(angular_velocity * dt) where dt = 1/sample_rate
        # In raw units: integrated_raw_value / sample_rate = angle / scale_factor
        # Therefore: scale_factor = (integrated_raw_value / sample_rate) / angle

        integrated_value = np.sum(rotation_segment)

        # Calculate raw-to-degree conversion factor
        # This is a simple ratio: integrated_raw_value / rotation_angle_in_degrees
        raw_to_degree_factor = integrated_value / rotation_angle

        # Calculate scale factor in LSB/(deg/s)
        scale_y_deg = integrated_value / (rotation_angle * sample_rate)

        # Convert to LSB/(rad/s) by multiplying by 180/π
        # Since: rad/s = (deg/s) * (π/180)
        # Therefore: LSB/(rad/s) = LSB/(deg/s) * (180/π)
        scale_y = scale_y_deg * (180.0 / np.pi)

        # Verify by calculating what angle we actually measured
        measured_angle = integrated_value / (scale_y_deg * sample_rate)

        # Calculate variance in rad/s units
        # Use stationary noise for variance estimate
        std_rads = (noise_threshold / scale_y)
        variance_rads = std_rads ** 2

        print(f"\nResults:")
        print(f"  Total samples collected: {len(gyro_y_values)}")
        print(f"  Motion detected: samples {start_idx} to {stop_idx}")
        print(f"  Rotation samples: {num_samples}")
        print(f"  Rotation duration: {duration:.3f} seconds")
        print(f"  Sample rate: ~{sample_rate:.1f} Hz")
        print(f"  Integrated value (raw): {integrated_value:.0f}")
        print(f"  Expected rotation: {rotation_angle:.1f}°")
        print(f"  Measured rotation: {measured_angle:.1f}° (verification)")
        print(f"  Raw-to-degree factor: {raw_to_degree_factor:.2f} (integrated_value / {rotation_angle}°)")
        print(f"  Scale factor: {scale_y:.2f} LSB/(rad/s)")
        print(f"  Noise std dev: {std_rads:.6f} rad/s")
        print(f"  Variance: {variance_rads:.8f} (rad/s)²")
        print(f"  → To convert Y to rad/s: raw_y / {scale_y:.2f}")

        # Prepare return data BEFORE showing plot
        result = {
            'scale_y': scale_y,
            'raw_to_degree_factor': raw_to_degree_factor,
            'integrated_value': integrated_value,
            'duration': duration,
            'sample_rate': sample_rate,
            'start_idx': start_idx,
            'stop_idx': stop_idx,
            'num_rotation_samples': num_samples,
            'noise_mean': noise_mean,
            'noise_threshold': noise_threshold,
            'std_rads': std_rads,
            'variance_rads': variance_rads,
            'measured_angle': measured_angle,
            'expected_angle': rotation_angle
        }

        # Show plot of the rotation (this is blocking - waits for user to close)
        self._plot_rotation_data(gyro_y_values, start_idx, stop_idx, noise_mean, noise_threshold,
                                 sample_rate, rotation_angle, measured_angle)

        return result

    def _plot_rotation_data(self, gyro_y_values, start_idx, stop_idx, noise_mean, noise_threshold,
                            sample_rate, expected_angle, measured_angle):
        """Plot the gyro rotation data to visualize the calibration."""
        plt.figure(figsize=(14, 7))

        # Create time axis in seconds
        time_axis = np.arange(len(gyro_y_values)) / sample_rate

        plt.plot(time_axis, gyro_y_values, 'b-', linewidth=1.5, label='Gyro Y-axis (raw)')
        plt.axhline(y=noise_mean, color='gray', linestyle='--', label=f'Noise mean ({noise_mean:.1f})')
        plt.axhline(y=noise_mean + noise_threshold, color='r', linestyle=':', alpha=0.5, label=f'Threshold (±{noise_threshold:.1f})')
        plt.axhline(y=noise_mean - noise_threshold, color='r', linestyle=':', alpha=0.5)

        # Highlight rotation segment using time
        start_time = start_idx / sample_rate
        stop_time = stop_idx / sample_rate
        plt.axvspan(start_time, stop_time, alpha=0.2, color='green', label='Detected rotation')
        plt.axvline(x=start_time, color='g', linestyle='--', alpha=0.7, linewidth=2, label=f'Start ({start_time:.2f}s)')
        plt.axvline(x=stop_time, color='r', linestyle='--', alpha=0.7, linewidth=2, label=f'Stop ({stop_time:.2f}s)')

        plt.xlabel('Time (seconds)', fontsize=12)
        plt.ylabel('Gyro Y-axis (raw LSB)', fontsize=12)
        plt.title(f'Gyroscope Y-axis Calibration - Expected: {expected_angle:.0f}°, Measured: {measured_angle:.1f}°',
                 fontsize=14, fontweight='bold')
        plt.legend(loc='best', fontsize=10)
        plt.grid(True, alpha=0.3)

        # Add text box with calibration info
        duration = stop_time - start_time
        info_text = f'Rotation Duration: {duration:.2f}s\nSample Rate: {sample_rate:.1f} Hz'
        plt.text(0.02, 0.98, info_text, transform=plt.gca().transAxes,
                fontsize=10, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        plt.tight_layout()
        plt.show()

    def save_calibration(self, filename: str, accel_cal: Dict, gyro_cal: Dict):
        """Save calibration to JSON file."""

        # Convert numpy types to native Python types to avoid JSON serialization issues
        def convert_to_native(obj):
            if isinstance(obj, np.integer):
                return int(obj)
            elif isinstance(obj, np.floating):
                return float(obj)
            elif isinstance(obj, np.ndarray):
                return obj.tolist()
            elif isinstance(obj, dict):
                return {key: convert_to_native(value) for key, value in obj.items()}
            elif isinstance(obj, list):
                return [convert_to_native(item) for item in obj]
            return obj

        calibration_data = {
            'accelerometer_y': {
                'scale': accel_cal['scale_y'],
                'variance': accel_cal['variance_ms2'],
                'covariance': [accel_cal['variance_ms2'], 0, 0, 0, accel_cal['variance_ms2'], 0, 0, 0, accel_cal['variance_ms2']],
                'details': convert_to_native(accel_cal)
            },
            'gyroscope_y': {
                'scale': gyro_cal['scale_y'],
                'bias': gyro_cal['noise_mean'],
                'variance': gyro_cal['variance_rads'],
                'covariance': [gyro_cal['variance_rads'], 0, 0, 0, gyro_cal['variance_rads'], 0, 0, 0, gyro_cal['variance_rads']],
                'details': convert_to_native(gyro_cal)
            },
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'notes': {
                'method': 'Single-axis (Y) calibration',
                'accel_units': 'Scale is in LSB/(m/s²). To convert: raw_y / scale. Output is in m/s².',
                'gyro_units': 'Scale is in LSB/(rad/s). To convert: (raw_y - bias) / scale. Output is in rad/s.',
                'variance_units': 'Variance for accel in (m/s²)², for gyro in (rad/s)²',
                'covariance': 'Diagonal 3x3 covariance matrix assuming all axes have same variance',
                'coordinate_system': 'Y-axis points UP when controller is flat, buttons facing up'
            }
        }

        # Write with explicit flush to ensure data is saved
        try:
            with open(filename, 'w') as f:
                json.dump(calibration_data, f, indent=2)
                f.flush()  # Ensure all data is written to disk
            print(f"\n✓ Calibration data saved to: {filename}")
        except Exception as e:
            print(f"\n✗ Error saving calibration: {e}")
            print("Calibration data (for manual inspection):")
            print(json.dumps(calibration_data, indent=2))
            raise

    def cleanup(self):
        """Cleanup controller."""
        if self.controller.is_active:
            self.controller.deactivate()


def main():
    print("="*70)
    print("DualSense Simple Y-Axis Calibration")
    print("="*70)
    print("\nThis tool calibrates only the Y-axis (most useful for typical usage)")
    print()

    calibrator = SimpleCalibrator()

    try:
        # Step 1: Accelerometer calibration
        accel_cal = calibrator.calibrate_accel_y(duration=1.0)

        # Step 2: Gyroscope calibration
        print("\n")
        input("Press Enter to continue to gyroscope calibration...")

        gyro_cal = calibrator.calibrate_gyro_y(rotation_angle=90.0)

        if gyro_cal is None:
            print("\nGyroscope calibration failed. Please try again.")
            return

        # Step 3: Save calibration
        calibrator.save_calibration('dualsense_calibration_simple.json', accel_cal, gyro_cal)

        # Print summary
        print("\n" + "="*70)
        print("CALIBRATION SUMMARY")
        print("="*70)
        print(f"\nAccelerometer Y-axis:")
        print(f"  Scale: {accel_cal['scale_y']:.2f} LSB/(m/s²)")
        print(f"  Variance: {accel_cal['variance_ms2']:.6f} (m/s²)²")
        print(f"  Usage: accel_y_in_ms2 = raw_y / {accel_cal['scale_y']:.2f}")
        print(f"\nGyroscope Y-axis:")
        print(f"  Scale: {gyro_cal['scale_y']:.2f} LSB/(rad/s)")
        print(f"  Bias: {gyro_cal['noise_mean']:.2f} LSB")
        print(f"  Variance: {gyro_cal['variance_rads']:.8f} (rad/s)²")
        print(f"  Usage: gyro_y_in_rads = (raw_y - {gyro_cal['noise_mean']:.2f}) / {gyro_cal['scale_y']:.2f}")
        print(f"\nNote: All axes assumed to have same variance as Y-axis")
        print("\nCalibration complete! ✓")

    except KeyboardInterrupt:
        print("\n\nCalibration cancelled by user.")
    except Exception as e:
        print(f"\n\nError during calibration: {e}")
        import traceback
        traceback.print_exc()
    finally:
        calibrator.cleanup()


if __name__ == "__main__":
    main()
