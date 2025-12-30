#!/usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
from dualsense_controller import DualSenseController, Gyroscope, Accelerometer
import threading
import numpy as np
import json
import os

# Try to load calibration
calibration = None
use_calibration = False

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))
calibration_file = os.path.join(script_dir, 'dualsense_calibration_simple.json')

try:
    with open(calibration_file, 'r') as f:
        calibration = json.load(f)
    print(f"✓ Using calibration from: {calibration_file}")
    use_calibration = True
except FileNotFoundError:
    print(f"⚠ No calibration file found at: {calibration_file}")
    print("  Showing raw values")
    print(f"  Run 'python calibrate_simple.py' in {script_dir} to create calibration")

# Data storage
max_samples = 200
gyro_data = {
    'x': deque([0]*max_samples, maxlen=max_samples),
    'y': deque([0]*max_samples, maxlen=max_samples),
    'z': deque([0]*max_samples, maxlen=max_samples)
}
accel_data = {
    'x': deque([0]*max_samples, maxlen=max_samples),
    'y': deque([0]*max_samples, maxlen=max_samples),
    'z': deque([0]*max_samples, maxlen=max_samples)
}

# Thread lock for data access
data_lock = threading.Lock()

# Initialize controller
device_infos = DualSenseController.enumerate_devices()
if len(device_infos) < 1:
    raise Exception('No DualSense Controller available.')

controller = DualSenseController()

# Conversion functions
def gyro_to_rad_per_sec(gyro: Gyroscope):
    """Convert raw gyro values to radians per second using calibration."""
    if not use_calibration:
        return gyro.x, gyro.y, gyro.z

    # Get calibration values (scale is already in LSB/(rad/s))
    gyro_y_scale = calibration['gyroscope_y']['scale']
    gyro_y_bias = calibration['gyroscope_y']['bias']

    # Apply Y-axis calibration to all axes (they use the same scale)
    # Output is directly in rad/s
    x_rps = (gyro.x - gyro_y_bias) / gyro_y_scale
    y_rps = (gyro.y - gyro_y_bias) / gyro_y_scale
    z_rps = (gyro.z - gyro_y_bias) / gyro_y_scale

    return x_rps, y_rps, z_rps

def accel_to_m_per_s2(accel: Accelerometer):
    """Convert raw accelerometer values to m/s^2 using calibration."""
    if not use_calibration:
        return accel.x, accel.y, accel.z

    # Get calibration values (scale is already in LSB/(m/s²))
    accel_y_scale = calibration['accelerometer_y']['scale']

    # Apply Y-axis calibration to all axes (they use the same scale)
    # Output is directly in m/s²
    x_ms2 = accel.x / accel_y_scale
    y_ms2 = accel.y / accel_y_scale
    z_ms2 = accel.z / accel_y_scale

    return x_ms2, y_ms2, z_ms2

# Callbacks
def on_gyroscope_change(gyroscope: Gyroscope):
    with data_lock:
        x_rps, y_rps, z_rps = gyro_to_rad_per_sec(gyroscope)
        gyro_data['x'].append(x_rps)
        gyro_data['y'].append(y_rps)
        gyro_data['z'].append(z_rps)

def on_accelerometer_change(accelerometer: Accelerometer):
    with data_lock:
        x_ms2, y_ms2, z_ms2 = accel_to_m_per_s2(accelerometer)
        accel_data['x'].append(x_ms2)
        accel_data['y'].append(y_ms2)
        accel_data['z'].append(z_ms2)

# Register callbacks
controller.gyroscope.on_change(on_gyroscope_change)
controller.accelerometer.on_change(on_accelerometer_change)

# Activate controller
controller.activate()

# Setup plot
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

# Gyroscope plot
line_gyro_x, = ax1.plot([], [], 'r-', label='X (Roll rate)', linewidth=1.5)
line_gyro_y, = ax1.plot([], [], 'g-', label='Y (Pitch rate)', linewidth=1.5)
line_gyro_z, = ax1.plot([], [], 'b-', label='Z (Yaw rate)', linewidth=1.5)
ax1.set_xlim(0, max_samples)

if use_calibration:
    # ax1.set_ylim(-3.5, 3.5)  # ~200°/s in rad/s
    ax1.set_ylim(-3.5, 3.5)
    ax1.set_title('Gyroscope (calibrated)', fontsize=14, fontweight='bold')
    ax1.set_ylabel('Angular Velocity (rad/s)')
else:
    ax1.set_ylim(-1000, 1000)
    ax1.set_title('Gyroscope (raw)', fontsize=14, fontweight='bold')
    ax1.set_ylabel('Angular Velocity (raw)')

ax1.set_xlabel('Sample')
ax1.legend(loc='upper right')
ax1.grid(True, alpha=0.3)
ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)

# Accelerometer plot
line_accel_x, = ax2.plot([], [], 'r-', label='X', linewidth=1.5)
line_accel_y, = ax2.plot([], [], 'g-', label='Y', linewidth=1.5)
line_accel_z, = ax2.plot([], [], 'b-', label='Z', linewidth=1.5)
ax2.set_xlim(0, max_samples)

if use_calibration:
    ax2.set_ylim(-20, 20)  # ~2g in m/s^2
    ax2.set_title('Accelerometer (calibrated)', fontsize=14, fontweight='bold')
    ax2.set_ylabel('Acceleration (m/s²)')
    # Add reference lines for ±1g (~9.81 m/s^2)
    ax2.axhline(y=9.80665, color='gray', linestyle=':', alpha=0.5)
    ax2.axhline(y=-9.80665, color='gray', linestyle=':', alpha=0.5)
else:
    ax2.set_ylim(-10000, 10000)
    ax2.set_title('Accelerometer (raw)', fontsize=14, fontweight='bold')
    ax2.set_ylabel('Acceleration (raw)')

ax2.set_xlabel('Sample')
ax2.legend(loc='upper right')
ax2.grid(True, alpha=0.3)
ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)

# Add calibration status to title
status = "CALIBRATED (ALL AXES)" if use_calibration else "RAW VALUES (NO CALIBRATION)"
fig.suptitle(f'DualSense IMU Data - {status}', fontsize=16, fontweight='bold')

plt.tight_layout()

def animate(frame):
    with data_lock:
        x = range(len(gyro_data['x']))

        # Update gyroscope
        line_gyro_x.set_data(x, list(gyro_data['x']))
        line_gyro_y.set_data(x, list(gyro_data['y']))
        line_gyro_z.set_data(x, list(gyro_data['z']))

        # Auto-scale gyro if calibrated and values exceed range
        if use_calibration and len(gyro_data['x']) > 0:
            all_gyro = list(gyro_data['x']) + list(gyro_data['y']) + list(gyro_data['z'])
            max_val = max(abs(min(all_gyro)), abs(max(all_gyro)))
            if max_val > 2.6:  # ~150°/s in rad/s
                ax1.set_ylim(-max_val * 1.2, max_val * 1.2)

        # Update accelerometer
        line_accel_x.set_data(x, list(accel_data['x']))
        line_accel_y.set_data(x, list(accel_data['y']))
        line_accel_z.set_data(x, list(accel_data['z']))

    return (line_gyro_x, line_gyro_y, line_gyro_z,
            line_accel_x, line_accel_y, line_accel_z)

# Animation
ani = animation.FuncAnimation(fig, animate, interval=20, blit=True, cache_frame_data=False)

print("\n" + "="*70)
print("DualSense IMU Data Visualization")
print("="*70)
if use_calibration:
    print("\nShowing calibrated values:")
    print("  - Gyroscope (all axes): radians per second (rad/s)")
    print("  - Accelerometer (all axes): meters per second squared (m/s²)")
    print("  - Using Y-axis scale for all components")
else:
    print("\nShowing RAW values (uncalibrated)")
    print("  - Run 'python calibrate_simple.py' to create calibration")
print("\nClose the plot window to exit.")
print("="*70 + "\n")

try:
    plt.show()
finally:
    controller.deactivate()
    print("\nController deactivated. Goodbye!")
