# DualSense Teleop for ROS2

ROS2 package for publishing calibrated IMU data from Sony DualSense (PS5) controller.

## Features

- Publishes calibrated IMU data (accelerometer & gyroscope) to ROS2
- Outputs in standard units: **m/s²** for acceleration, **rad/s** for angular velocity
- Includes variance/covariance from calibration
- Configurable frame rotation (X, Y, Z axes)
- Compatible with `imu_filter_madgwick` for orientation estimation

## Prerequisites

- Test on ROS2 Humble
- Python 3.10+
- Sony DualSense Controller (PS5 controller)

## Installation

### 1. Install dualsense-controller-python

Follow the official installation instructions:  
**https://github.com/yesbotics/dualsense-controller-python**

```bash
pip install dualsense-controller
```

Or for development:
```bash
git clone https://github.com/yesbotics/dualsense-controller-python.git
cd dualsense-controller-python
pip install -e .
```

**Note:** You may need to set up udev rules for USB access. See the [gyroscope/accelerometer section](https://github.com/yesbotics/dualsense-controller-python/tree/main?tab=readme-ov-file#gyroscope-accelerometer-and-orientation) for details.

### 2. Set up ROS2 Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository (if not already done)
# git clone <your-repo-url>
```

### 3. Clone IMU Tools Package

This package requires `imu_filter_madgwick` for orientation estimation:

```bash
cd ~/ros2_ws/src
git clone -b $ROS_DISTRO https://github.com/CCNYRoboticsLab/imu_tools.git
```

**IMPORTANT:** DualSense does not publish magnetometer data!  
Edit the configuration file:

```bash
nano imu_tools/imu_filter_madgwick/config/imu_filter.yaml
```

Set:
```yaml
use_mag: false
```

### 4. Clone Universal Robots Packages

For UR5 robot support, clone the UR description and MoveIt config packages:

```bash
cd ~/ros2_ws/src
git clone -b $ROS_DISTRO https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
git clone -b $ROS_DISTRO https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
sudo apt install ros-$ROS_DISTRO-moveit
```

### 5. Build the Workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Calibration

Before first use, calibrate the DualSense IMU:

```bash
cd ~/ros2_ws/src/dualsense_teleop/dualsense_teleop/dualsense_calibration
python3 calibrate_simple.py
```

Follow the on-screen instructions:
1. Place controller flat for accelerometer calibration (measures gravity = 1g = 9.80665 m/s²)
2. Rotate controller 90° around Y-axis for gyroscope calibration

This generates `dualsense_calibration_simple.json` with:
- Scale factors (LSB/(m/s²) and LSB/(rad/s))
- Variance and covariance matrices
- Bias correction for gyroscope

## Usage

### 6. Connect DualSense Controller

**Via USB:**
```bash
# Plug in USB-C cable, controller should be detected automatically
```

**Via Bluetooth:**
```bash
# 1. Hold PS + Create buttons until light bar flashes
# 2. Pair via system Bluetooth settings
```

### 7. Run the Nodes

**Terminal 1 - DualSense Nodes:**
inside your ROS2 workspace
```bash
source ./install/setup.bash
ros2 launch dualsense_teleop dualsense_pose_tracking.launch.py
```

**Terminal 2 - UR5 Arm Teleoperation:**
```bash
source ./install/setup.bash
ros2 launch dualsense_teleop ur5_pose_tracking.launch.py
```

### Optional: Custom Rotation

If you need to rotate the IMU frame orientation:

```bash
ros2 run dualsense_teleop dualsense_publish_imu --ros-args \
  -p rotation_x_deg:=90.0 \
  -p rotation_y_deg:=0.0 \
  -p rotation_z_deg:=180.0
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | Raw calibrated IMU data (no orientation) |
| `/imu/data` | `sensor_msgs/msg/Imu` | Filtered IMU data with orientation |

## Parameters

### dualsense_publish_imu

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `rotation_x_deg` | double | 90.0 | Rotation about X-axis (degrees) |
| `rotation_y_deg` | double | 0.0 | Rotation about Y-axis (degrees) |
| `rotation_z_deg` | double | 180.0 | Rotation about Z-axis (degrees) |

### imu_filter_madgwick

See [imu_filter.yaml](../imu_tools/imu_filter_madgwick/config/imu_filter.yaml) for full configuration.

Key parameters:
- `use_mag: false` - **Must be false** (DualSense has no magnetometer)
- `gain: 0.1` - Filter gain (lower = smoother, less noise)
- `world_frame: "enu"` - Coordinate convention
- `fixed_frame: "world"` - Reference frame name

## Troubleshooting

### Calibration file not found
```bash
# Make sure you ran calibrate_simple.py and the JSON file exists
ls ~/ros2_ws/src/dualsense_teleop/dualsense_teleop/dualsense_calibration/dualsense_calibration_simple.json
```

## Data Format

All published data follows ROS2 `sensor_msgs/msg/Imu` standard:

- **Linear Acceleration:** m/s² (NOT g-force)
- **Angular Velocity:** rad/s (NOT degrees/s)
- **Orientation:** Quaternion (from Madgwick filter)
- **Covariance:** 3×3 diagonal matrices (from calibration)

## License

Apache-2.0

## References

- [dualsense-controller-python](https://github.com/yesbotics/dualsense-controller-python)
- [imu_tools (ROS2 Humble)](https://github.com/CCNYRoboticsLab/imu_tools/tree/humble)
- [Madgwick AHRS Algorithm](http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/)
