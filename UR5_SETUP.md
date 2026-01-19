# UR5 Pose Tracking Setup Guide

This guide explains how to set up UR5 robot for pose tracking with DualSense controller.

## Prerequisites

1. **Clone Universal Robots ROS2 Driver** (contains MoveIt config):
   ```bash
   cd ~/ws_moveit2/src
   git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
   ```

2. **You should already have**:
   - `ur_description` package (already in your workspace)
   - `dualsense_teleop` package
   - `moveit_servo` package

## Create Configuration Files

### 1. Create config directory:
```bash
cd ~/ws_moveit2/src/moveit2/dualsense_teleop
mkdir -p config
```

### 2. Create `config/ur5_ros2_controllers.yaml`:
```yaml
# This config file is used by ros2_control for UR5
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    ur_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster


ur_arm_controller:
  ros__parameters:
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
```

### 3. Create `config/ur5_servo_config.yaml`:
```yaml
###############################################
# Servo parameters for UR5
###############################################
use_gazebo: false
command_in_type: "speed_units"  # Commands are in m/s and rad/s

scale:
  linear: 0.4   # Max linear velocity (m/s) - conservative for UR5
  rotational: 0.8  # Max angular velocity (rad/s)
  joint: 0.5

## Properties of outgoing commands
publish_period: 0.034  # 1/Nominal publish rate [seconds] ~29 Hz

# What type of topic does your robot driver expect?
command_out_type: trajectory_msgs/JointTrajectory

publish_joint_positions: true
publish_joint_velocities: true
publish_joint_accelerations: false

## Smoothing filter
smoothing_filter_plugin_name: "online_signal_smoothing::ButterworthFilterPlugin"

## MoveIt properties
move_group_name: ur_manipulator  # Standard UR MoveIt group name
planning_frame: base_link  # UR5 base frame
ee_frame_name: tool0  # UR5 end-effector frame
robot_link_command_frame: tool0  # Commands given in EE frame

## Stopping behaviour
incoming_command_timeout: 0.1
num_outgoing_halt_msgs_to_publish: 4

## Configure handling of singularities and joint limits
lower_singularity_threshold: 17.0
hard_stop_singularity_threshold: 30.0
joint_limit_margin: 0.1  # radians
leaving_singularity_threshold_multiplier: 2.0

## Topic names
cartesian_command_in_topic: ~/delta_twist_cmds
joint_command_in_topic: ~/delta_joint_cmds
joint_topic: /joint_states
status_topic: ~/status
command_out_topic: /ur_arm_controller/joint_trajectory

## Collision checking
check_collisions: true
collision_check_rate: 10.0
self_collision_proximity_threshold: 0.01
scene_collision_proximity_threshold: 0.02
```

### 4. Update `setup.py` to install config files:

Add to the `data_files` section in `setup.py`:
```python
data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),  # ADD THIS LINE
],
```

## Build the Workspace

```bash
cd ~/ws_moveit2
colcon build --packages-select dualsense_teleop --symlink-install
source install/setup.bash
```

## Launch the System

### Option 1: With DualSense (full system):
```bash
ros2 launch dualsense_teleop ur5_pose_tracking.launch.py
```

This launches:
- RViz for visualization
- UR5 robot description and TF publisher
- ros2_control with fake hardware
- Joint state broadcaster and arm controller
- MoveIt Servo pose tracking node
- DualSense IMU publisher
- IMU filter (Madgwick)
- DualSense pose tracking node

### Option 2: Without DualSense (for testing):

If you want to test without DualSense, you can publish target poses manually:
```bash
ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 0.3, y: 0.0, z: 0.5},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

## Key Differences from Panda

| Parameter | Panda | UR5 |
|-----------|-------|-----|
| Planning frame | `panda_link0` | `base_link` |
| End-effector frame | `panda_link8` / `panda_hand` | `tool0` |
| MoveIt group name | `panda_arm` | `ur_manipulator` |
| Controller name | `panda_arm_controller` | `ur_arm_controller` |
| Joint names | `panda_joint1-7` | `shoulder_pan_joint`, `shoulder_lift_joint`, etc. |
| Number of joints | 7 | 6 |

## UR5 Joint Names

The UR5 has 6 joints:
1. `shoulder_pan_joint` - Base rotation
2. `shoulder_lift_joint` - Shoulder pitch
3. `elbow_joint` - Elbow pitch
4. `wrist_1_joint` - Wrist 1 rotation
5. `wrist_2_joint` - Wrist 2 rotation
6. `wrist_3_joint` - Wrist 3 rotation

## Troubleshooting

### TF Errors
If you see "Could not find transform between 'base_link' and 'tool0'":
- Make sure `robot_state_publisher` is running
- Check TF tree: `ros2 run tf2_tools view_frames`
- Verify the robot description loaded correctly

### Controller Not Found
If you see "Could not find controller 'ur_arm_controller'":
- Check the controller manager is running
- List controllers: `ros2 control list_controllers`
- Manually spawn: `ros2 run controller_manager spawner ur_arm_controller`

### Slow Tracking
If tracking is too slow, increase PID gains in `config/pose_tracking_settings.yaml`:
```yaml
x_proportional_gain: 15.0
y_proportional_gain: 15.0
z_proportional_gain: 15.0
angular_proportional_gain: 3.0
```

### Robot Moves Too Fast
If robot moves too aggressively, decrease velocity limits in `ur5_servo_config.yaml`:
```yaml
scale:
  linear: 0.2   # Reduce from 0.4
  rotational: 0.4  # Reduce from 0.8
```

## Using with Real UR5 Robot

To use with a real UR5 (not fake hardware):

1. **Modify xacro mappings** in the launch file:
   ```python
   mappings={
       'ur_type': 'ur5',
       'robot_ip': '192.168.1.100',  # Your robot's IP
       'use_fake_hardware': 'false',  # Change to false
       # ... other params
   }
   ```

2. **Update controller config** to match your robot's controller manager

3. **Ensure proper network connection** between PC and UR5 robot

4. **Follow UR ROS2 Driver setup**: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver

## Next Steps

- Tune PID gains for smoother tracking
- Adjust velocity limits for your application
- Add collision objects in RViz planning scene
- Test with different UR robot models (ur3, ur10, etc.) by changing `ur_type`

## References

- [Universal Robots ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [MoveIt Servo Documentation](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- [UR Description Package](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)