# LiDAR Operations

This guide covers LiDAR sensor operations, setup, and troubleshooting for the PARTS Common Robotics Platform.

## Overview

The robot uses a LiDAR sensor for obstacle detection, mapping, and navigation. The LiDAR provides 360-degree range measurements around the robot.

## LiDAR Specifications

- **Model**: RPLidar A1M8
- **Range**: 0.15m - 12m
- **Angular Resolution**: 0.33°
- **Scan Rate**: 5.5 Hz
- **Interface**: Serial (USB)

## LiDAR Setup

### 1. Hardware Connection

1. **Physical Mounting**
   - Mount LiDAR on designated mounting point
   - Ensure clear 360° rotation
   - Secure mounting to prevent vibration

2. **Power Connection**
   - Connect power cable to robot power system
   - Verify voltage requirements (5V)
   - Check for proper grounding

3. **Data Connection**
   - Connect USB cable to robot controller
   - Verify serial communication
   - Test connection stability

### 2. Software Configuration

**Launch LiDAR:**
```bash
# Launch LiDAR node
cd ~/repos/common_platform/common_platform_ws/
source install/setup.bash
ros2 launch sensors rplidar.launch.py
```

**Verify Data:**
```bash
# Check LiDAR data
ros2 topic echo /scan

# Monitor data rate
ros2 topic hz /scan
```

## LiDAR Calibration

### 1. Mounting Calibration

**Check Mounting:**
- Verify LiDAR is level
- Ensure no obstructions
- Check for vibration

**Test Rotation:**
```bash
ros2 service call /rcr001/stop_motor std_srvs/srv/Empty {}
ros2 service call /rcr001/start_motor std_srvs/srv/Empty {}
```

### 2. Range Calibration

**Test Range Accuracy:**
1. Place known objects at known distances
2. Compare LiDAR readings with actual distances
3. Adjust calibration if needed

**Verify Range Limits:**
- Test minimum range (0.15m)
- Test maximum range (12m)
- Check for blind spots

## LiDAR Data Analysis

**Scan Message:**
```yaml
std_msgs/Header header
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
```

 Check for:
 - Consistent range values
 - No missing data points
 - Reasonable intensity values

## Troubleshooting

### Common Issues

**No LiDAR Data:**
- Check USB connection
- Verify power supply
- Check device permissions
- Test with different USB port

**Poor Data Quality:**
- Clean LiDAR lens
- Check for obstructions
- Verify mounting stability
- Test in different environments

**Inconsistent Readings:**
- Check for vibration
- Verify mounting
- Test rotation mechanism
- Check for interference

### Diagnostic Commands

```bash
# Check USB devices
lsusb | grep -i lidar

# Check serial devices
ls /dev/tty*

# Test serial communication
ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/ttyUSB0

# Monitor data quality
ros2 topic echo /scan
ros2 topic hz /scan
```

### Error Codes

**Common Error Messages:**
- "Failed to open serial port" - Check USB connection
- "No data received" - Check power and connections
- "Invalid data" - Check for interference or damage

## Performance Optimization

### Data Processing

**Filtering:**
- Remove noise from data
- Filter out invalid readings
- Smooth data for better performance

**Downsampling:**
- Reduce data rate if needed
- Maintain sufficient resolution
- Balance performance vs. accuracy

### Parameter Tuning

**Scan Parameters:**
```yaml
# In launch file
scan_time: 0.1
range_min: 0.15
range_max: 12.0
```

**Update Rates:**
- Adjust scan frequency
- Balance performance vs. accuracy
- Consider computational load

---

*For camera operations, see [Camera Operations](camera_operations.md)*
*For IMU operations, see [IMU Operations](imu_operations.md)*
*For troubleshooting, see [Troubleshooting](../troubleshooting/index.md)*
