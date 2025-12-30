#!/usr/bin/env python3
"""Real-time plotting of DualSense IMU data"""

from pydualsense import pydualsense
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np

# Initialize DualSense
ds = pydualsense()
ds.init()
print("DualSense initialized. Close the plot window to exit.")

# Data storage (keep last 200 samples)
max_len = 200
gyro_roll = deque(maxlen=max_len)
gyro_pitch = deque(maxlen=max_len)
gyro_yaw = deque(maxlen=max_len)
accel_x = deque(maxlen=max_len)
accel_y = deque(maxlen=max_len)
accel_z = deque(maxlen=max_len)
timestamps = deque(maxlen=max_len)

# Create figure with 2 subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
fig.suptitle('DualSense IMU Real-Time Data', fontsize=14, fontweight='bold')

# Gyro plot
line_roll, = ax1.plot([], [], 'r-', label='Roll', linewidth=2)
line_pitch, = ax1.plot([], [], 'g-', label='Pitch', linewidth=2)
line_yaw, = ax1.plot([], [], 'b-', label='Yaw', linewidth=2)
ax1.set_ylabel('Gyroscope (raw units)', fontsize=12)
ax1.set_title('Gyroscope Data')
ax1.legend(loc='upper right')
ax1.grid(True, alpha=0.3)

# Accel plot
line_ax, = ax2.plot([], [], 'r-', label='X', linewidth=2)
line_ay, = ax2.plot([], [], 'g-', label='Y', linewidth=2)
line_az, = ax2.plot([], [], 'b-', label='Z', linewidth=2)
ax2.set_xlabel('Sample', fontsize=12)
ax2.set_ylabel('Accelerometer (raw units)', fontsize=12)
ax2.set_title('Accelerometer Data')
ax2.legend(loc='upper right')
ax2.grid(True, alpha=0.3)

time_counter = 0

def init():
    """Initialize animation"""
    line_roll.set_data([], [])
    line_pitch.set_data([], [])
    line_yaw.set_data([], [])
    line_ax.set_data([], [])
    line_ay.set_data([], [])
    line_az.set_data([], [])
    return line_roll, line_pitch, line_yaw, line_ax, line_ay, line_az

def update(frame):
    """Update plot with new data"""
    global time_counter
    
    # Read sensor data
    ds.state
    
    # Store data
    gyro_roll.append(ds.state.gyro.Roll)
    gyro_pitch.append(ds.state.gyro.Pitch)
    gyro_yaw.append(ds.state.gyro.Yaw)
    accel_x.append(ds.state.accelerometer.X)
    accel_y.append(ds.state.accelerometer.Y)
    accel_z.append(ds.state.accelerometer.Z)
    timestamps.append(time_counter)
    time_counter += 1
    
    # Update gyro plot
    x_data = list(timestamps)
    line_roll.set_data(x_data, list(gyro_roll))
    line_pitch.set_data(x_data, list(gyro_pitch))
    line_yaw.set_data(x_data, list(gyro_yaw))
    
    # Update accel plot
    line_ax.set_data(x_data, list(accel_x))
    line_ay.set_data(x_data, list(accel_y))
    line_az.set_data(x_data, list(accel_z))
    
    # Auto-scale axes
    if len(timestamps) > 0:
        ax1.set_xlim(timestamps[0], timestamps[-1])
        ax2.set_xlim(timestamps[0], timestamps[-1])
        
        # Gyro y-axis
        gyro_data = list(gyro_roll) + list(gyro_pitch) + list(gyro_yaw)
        if gyro_data:
            gyro_min, gyro_max = min(gyro_data), max(gyro_data)
            margin = (gyro_max - gyro_min) * 0.1 + 1
            ax1.set_ylim(gyro_min - margin, gyro_max + margin)
        
        # Accel y-axis
        accel_data = list(accel_x) + list(accel_y) + list(accel_z)
        if accel_data:
            accel_min, accel_max = min(accel_data), max(accel_data)
            margin = (accel_max - accel_min) * 0.1 + 1
            ax2.set_ylim(accel_min - margin, accel_max + margin)
    
    # Print current values
    if time_counter % 20 == 0:
        print(f"Gyro: [{ds.state.gyro.Roll:6d}, {ds.state.gyro.Pitch:6d}, {ds.state.gyro.Yaw:6d}] | "
              f"Accel: [{ds.state.accelerometer.X:6d}, {ds.state.accelerometer.Y:6d}, {ds.state.accelerometer.Z:6d}]")
    
    return line_roll, line_pitch, line_yaw, line_ax, line_ay, line_az

# Create animation
ani = animation.FuncAnimation(
    fig, update, init_func=init,
    interval=20,  # 20ms = 50Hz update rate
    blit=True,
    cache_frame_data=False
)

plt.tight_layout()

try:
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    ds.close()
    print("\nDualSense closed.")
