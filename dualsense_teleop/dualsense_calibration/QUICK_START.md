# Quick Start: DualSense IMU Calibration

Choose your calibration method:

## Method 1: Simple Y-Axis (RECOMMENDED) ⚡

**Time:** ~30 seconds
**Difficulty:** Easy

```bash
# Step 1: Calibrate
python calibrate_simple.py
# → Place flat, press Enter (1 sec)
# → Rotate 90° around Y-axis (1 sec)

# Step 2: Test
python test_simple_calibration.py

# Step 3: Use it!
python plot_dualsense_calibrated.py
```

**Output:** `dualsense_calibration_simple.json`

---

## Method 2: Full 3-Axis (Most Accurate) 🎯

**Time:** ~5 minutes
**Difficulty:** Moderate

```bash
# Step 1: Calibrate
python calibrate_dualsense_imu.py
# → Place flat for gyro bias (5 sec)
# → 6 different positions for accel (2 sec each)
# → Optional: rotate for gyro scale

# Step 2: Test
python simple_calibrated_example.py

# Step 3: Use it!
python plot_dualsense_calibrated.py
```

**Output:** `dualsense_calibration.json`

---

## What Gets Calibrated?

### Accelerometer
- **What it measures:** Linear acceleration + gravity
- **Calibration result:** Convert raw → g (or m/s²)
- **When flat:** Should read ~1.0g on up-axis

### Gyroscope
- **What it measures:** Angular velocity (rotation speed)
- **Calibration result:** Convert raw → deg/s (or rad/s)
- **When still:** Should read ~0 deg/s (after bias removal)

---

## File Structure

```
src/test/
├── calibrate_simple.py          ← Simple Y-axis calibration (30 sec)
├── calibrate_dualsense_imu.py   ← Full 3-axis calibration (5 min)
├── imu_utils.py                 ← Conversion utilities
├── test_simple_calibration.py   ← Test simple calibration
├── simple_calibrated_example.py ← Test full calibration
├── plot_dualsense_calibrated.py ← Visualize with calibration
├── plot_dualsense_data.py       ← Original raw data plot
├── README_SIMPLE_CALIBRATION.md ← Simple method guide
├── README_CALIBRATION.md        ← Full method guide
└── QUICK_START.md              ← This file
```

---

## Using Calibration in Your Code

```python
from imu_utils import IMUConverter
from dualsense_controller import DualSenseController

# Load calibration (works with either simple or full)
converter = IMUConverter('dualsense_calibration_simple.json')

controller = DualSenseController()

def on_accel(accel):
    # Get Y-axis in g
    _, y_g, _ = converter.accel_to_g(accel)
    print(f"Accel Y: {y_g:.3f}g")

def on_gyro(gyro):
    # Get Y-axis in deg/s
    _, y_dps, _ = converter.gyro_to_deg_per_sec(gyro)
    print(f"Gyro Y: {y_dps:.1f}°/s")

controller.accelerometer.on_change(on_accel)
controller.gyroscope.on_change(on_gyro)
controller.activate()
```

---

## Coordinate System

When controller is **flat, buttons facing up**:

```
        Y (UP)
        ↑
        |
        |
   X ←--●--→ -X
        |
        |
        ↓
       -Y

Z points UP out of screen (⊙)
```

**Axes:**
- **X:** Left (-) to Right (+)
- **Y:** Front (-) to Back (+)  ← **Used in simple calibration**
- **Z:** Bottom (-) to Top (+)

**Rotations:**
- **Roll** (around X): Tilt left/right
- **Yaw** (around Y): Turn like a dial  ← **Used in simple calibration**
- **Pitch** (around Z): Tilt forward/back

---

## Expected Calibration Values

### Accelerometer Scale
```
Typical: 4096 - 16384 LSB/g
Common:  ~8192 LSB/g
```

### Gyroscope Scale
```
±250°/s:  ~131 LSB/(deg/s)
±500°/s:  ~65.5 LSB/(deg/s)
±1000°/s: ~32.8 LSB/(deg/s)
±2000°/s: ~16.4 LSB/(deg/s)
```

Your DualSense likely uses one of these ranges.

---

## Troubleshooting

**Calibration file not found?**
```bash
# Run calibration first!
python calibrate_simple.py
```

**Values seem wrong?**
- Re-run calibration
- Check controller was flat during accel calibration
- For gyro: rotate smoothly, not too fast/slow

**Gyroscope drifts over time?**
- Normal behavior for MEMS gyros
- Use complementary filter (included in `plot_dualsense_calibrated.py`)
- Or re-calibrate bias if temperature changed

---

## Which Method Should I Use?

**Use Simple (Y-axis)** if:
- ✓ You want quick results
- ✓ General accuracy is good enough
- ✓ You're just testing/prototyping

**Use Full (3-axis)** if:
- ✓ You need maximum accuracy
- ✓ You'll use all axes equally
- ✓ You're doing scientific work

**Both methods produce compatible files** - all tools work with either!

---

## Need Help?

- Read: `README_SIMPLE_CALIBRATION.md` for simple method details
- Read: `README_CALIBRATION.md` for full method details
- Check: Source code comments in `calibrate_simple.py` or `imu_utils.py`
