# Simple Y-Axis Calibration Guide

Quick and easy calibration using only the Y-axis - the most useful axis for typical controller usage.

## Why Y-Axis Only?

When the DualSense controller is lying flat (buttons up):
- **Y-axis points UP** (vertical)
- Accelerometer Y measures gravity (1g)
- Rotating around Y-axis (yaw) is the most natural rotation

**Assumptions:**
- All three accelerometer axes have similar scale factors
- All three gyroscope axes have similar scale factors
- This is generally true for MEMS sensors

## Quick Start

### 1. Run Calibration

```bash
python calibrate_simple.py
```

Follow the two simple steps:

#### Step A: Accelerometer (1 second)
```
Place controller FLAT, buttons facing UP
Press Enter
→ Collects 1 second of data
→ Calculates: scale = average_Y_value / 1g
```

#### Step B: Gyroscope (rotate 90°)
```
Keep controller FLAT, buttons facing UP
Get ready to rotate controller 90° like turning a dial
Press Enter, wait 2 seconds, then ROTATE smoothly
→ Detects rotation automatically
→ Calculates: scale = integrated_value / 90°
→ Shows plot of rotation data
```

### 2. Test Calibration

```bash
python test_simple_calibration.py
```

Shows real-time Y-axis values in physical units:
- Accelerometer Y in g
- Gyroscope Y in deg/s

### 3. Use in Your Code

The simple calibration works with existing tools:

```bash
# Use with the calibrated plotter
python plot_dualsense_calibrated.py

# Or in your own code
from imu_utils import IMUConverter

converter = IMUConverter('dualsense_calibration_simple.json')
# Works the same as full calibration!
```

## How It Works

### Accelerometer Calibration

When controller is flat:
```
Raw Y-axis value ≈ 8192 (example)
Gravity = 1g
→ Scale = 8192 LSB/g
```

Conversion:
```python
accel_y_g = raw_y / 8192
```

### Gyroscope Calibration

When rotating 90° in ~1 second:
```
1. Measure noise when still → bias
2. Detect rotation start/stop (threshold = 3×noise_std)
3. Integrate values during rotation
4. Calculate: scale = integrated_value / (90° × duration)
```

Conversion:
```python
gyro_y_dps = (raw_y - bias) / scale
```

## Output File Format

`dualsense_calibration_simple.json`:
```json
{
  "accelerometer_y": {
    "scale": 8192.45,
    "details": {
      "avg_y": 8192.45,
      "std_y": 12.34,
      "num_samples": 250
    }
  },
  "gyroscope_y": {
    "scale": 1024.67,
    "bias": -5.23,
    "details": {
      "integrated_value": 98765.43,
      "duration": 0.987,
      "sample_rate": 253.2,
      "noise_mean": -5.23,
      "noise_threshold": 15.67
    }
  },
  "timestamp": "2025-12-28 10:30:45"
}
```

## Tips for Good Calibration

### Accelerometer
- ✓ Use a flat, level surface
- ✓ Let controller settle for a few seconds
- ✗ Don't touch or bump during measurement

### Gyroscope
- ✓ Rotate smoothly and steadily
- ✓ Try to maintain constant speed
- ✓ Complete the full 90° rotation
- ✗ Don't rotate too fast (< 0.5 seconds)
- ✗ Don't rotate too slow (> 2 seconds)

**If calibration fails:** The script will show a plot of detected rotation. Check:
- Was motion detected? (green highlighted region)
- Did you rotate too fast/slow?
- Was there noise or vibration?

## Understanding the Plot

After gyroscope calibration, you'll see a plot showing:

- **Blue line**: Gyro Y-axis raw values over time
- **Gray dashed line**: Noise mean (bias)
- **Red dotted lines**: Detection threshold (±3σ)
- **Green region**: Detected rotation segment
- **Green/Red vertical lines**: Start/stop of rotation

**Good calibration:**
- Clear rotation peak
- Smooth curve
- Returns to near-baseline after rotation

**Bad calibration:**
- Multiple peaks (rotate once!)
- Very noisy/jagged
- Doesn't return to baseline

## Expected Values

Based on typical MEMS sensors:

**Accelerometer:**
- Scale: 4096 - 16384 LSB/g
- Most common: ~8192 LSB/g

**Gyroscope:**
- Scale: 16 - 131 LSB/(deg/s)
- Depends on range setting (±250°/s to ±2000°/s)
- Most common: ~65 LSB/(deg/s)

**Bias:**
- Usually -50 to +50 raw units
- Varies with temperature

## Comparison with Full Calibration

| Feature | Simple (Y-only) | Full (6-position) |
|---------|----------------|-------------------|
| Time required | ~30 seconds | ~5 minutes |
| Positions needed | 1 | 6 |
| Axes calibrated | Y (applied to all) | X, Y, Z separately |
| Accuracy | Good for most uses | Best |
| Use case | Quick calibration | Precision work |

**When to use simple:**
- Quick testing
- General use
- IMU axes are well-matched (usually true)

**When to use full:**
- Maximum precision needed
- Sensor axes vary significantly
- Scientific/research applications

## Troubleshooting

**"No motion detected!"**
- Rotate faster or with more range
- Check controller is on flat surface
- Try increasing rotation angle to 180°

**Values seem wrong after calibration**
- Re-run calibration with more careful rotation
- Check the plot - was rotation detected correctly?
- Ensure controller was flat during accel calibration

**Gyroscope still shows drift**
- This is normal for MEMS gyros
- Use complementary filter (combines gyro + accel)
- Re-calibrate bias if temperature changed

## Next Steps

After calibration:

1. **Test it**: Run `test_simple_calibration.py`
2. **Visualize**: Run `plot_dualsense_calibrated.py`
3. **Use it**: Import `IMUConverter` in your code

The simple calibration file works with all the same tools as full calibration!
