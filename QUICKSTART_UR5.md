# UR5 Pose Tracking Quick Start

Get UR5 working with DualSense controller in 5 minutes!

## Quick Setup

```bash
# 1. Navigate to the package
cd ~/ws_moveit2/src/moveit2/dualsense_teleop

# 2. Run the setup script to create config files
chmod +x create_ur5_configs.sh
./create_ur5_configs.sh

# 3. Rebuild the package
cd ~/ws_moveit2
colcon build --packages-select dualsense_teleop --symlink-install
source install/setup.bash

# 4. Launch!
ros2 launch dualsense_teleop ur5_pose_tracking.launch.py
```

## What You Get

- ✅ UR5 robot in RViz
- ✅ Fake hardware (simulation mode)
- ✅ MoveIt Servo pose tracking
- ✅ DualSense IMU orientation tracking
- ✅ Ready to control with your PS5 controller!

## Control Mapping

- **IMU Orientation**: Tilt controller = robot end-effector orientation
- **Joysticks**: Move target position (configured in your dualsense_pose_tracking node)

## Files Created

The setup script creates:
- `config/ur5_ros2_controllers.yaml` - Controller configuration
- `config/ur5_servo_config.yaml` - Servo parameters

## Differences from Panda

| Feature | Panda | UR5 |
|---------|-------|-----|
| Base frame | `panda_link0` | `base_link` |
| End-effector | `panda_link8` | `tool0` |
| Joints | 7 | 6 |
| Controller | `panda_arm_controller` | `ur_arm_controller` |

## Troubleshooting

### Config files not found?
Make sure you ran `./create_ur5_configs.sh` first!

### Controller spawn fails?
Wait a few seconds for ros2_control to initialize, then manually spawn:
```bash
ros2 run controller_manager spawner ur_arm_controller
```

### TF errors?
Check that robot_state_publisher is running:
```bash
ros2 node list | grep robot_state_publisher
```

### Tracking too slow/fast?
Edit the PID gains in:
```bash
# Copy from moveit_servo and modify
cp install/moveit_servo/share/moveit_servo/config/pose_tracking_settings.yaml \
   src/moveit2/dualsense_teleop/config/ur5_pose_tracking_settings.yaml

# Then edit the gains
nano src/moveit2/dualsense_teleop/config/ur5_pose_tracking_settings.yaml
```

Increase gains for faster tracking:
```yaml
x_proportional_gain: 15.0
y_proportional_gain: 15.0
z_proportional_gain: 15.0
angular_proportional_gain: 3.0
```

## Test Without DualSense

Publish a target pose manually:
```bash
ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 0.3, y: 0.2, z: 0.5},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}" --once
```

Watch it move in RViz!

## Using Other UR Models

Want UR3, UR10, or UR10e instead? Easy!

```bash
ros2 launch dualsense_teleop ur5_pose_tracking.launch.py ur_type:=ur10
```

Supported types: `ur3`, `ur5`, `ur10`, `ur3e`, `ur5e`, `ur10e`, `ur16e`, etc.

## Next Steps

- 📖 Read `UR5_SETUP.md` for detailed documentation
- 🎮 Test DualSense control
- ⚙️ Tune PID gains for your application
- 🤖 Try with real UR5 robot (requires network setup)

## Need Help?

Common issues and solutions in `UR5_SETUP.md` → Troubleshooting section.

Happy robot controlling! 🦾