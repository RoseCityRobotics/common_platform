# LiDAR Operations

This guide covers LiDAR sensor operations, setup, and troubleshooting for the Common Robotics Platform. The robot uses a  2-dimensional LiDAR sensor for obstacle detection, mapping, and navigation. The LiDAR provides 360-degree range measurements around the robot.

## Launch ROS Node

> ⚠️ **Important**: Ensure a ROS discovery server is running (see [Discovery Server Setup](../operations/discovery_server.md))

**Launch LiDAR:**
```bash
cd ~/repos/common_platform/common_platform_ws/
source install/setup.bash
ros2 launch sensors rplidar.launch.py
```

**Verify LiDAR Data:**
```bash
ros2 topic echo /${ROS_NAME}/scan
```
You should see a stream of data points here if your LiDAR is publishing data to ROS.

**Monitor data rate**
```bash
ros2 topic hz /${ROS_NAME}/scan
```

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

**Diagnostic Commands**

```bash
# Check USB devices
lsusb | grep -i lidar

# Check serial devices
ls /dev/tty*

# Test serial communication
ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/ttyUSB0

# Monitor data quality
ros2 topic echo /${ROS_NAME}/scan
ros2 topic hz /${ROS_NAME}/scan
```

## Troubleshooting

**No LiDAR Data:**
- Check USB connection
- Verify Teensy is powered and running
- Ensure a discovery server is running
- Connect Cursor to your Pi via SSH and ask for diagnosis help

## Hardware Connection

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

## Calibration

### Mounting Calibration

**Check Mounting:**
- Verify LiDAR is level
- Ensure no obstructions
- Check for vibration

**Test Rotation:**
```bash
ros2 service call /${ROS_NAME}/stop_motor std_srvs/srv/Empty {}
ros2 service call /${ROS_NAME}/start_motor std_srvs/srv/Empty {}
```

### Range Calibration

**Test Range Accuracy:**
1. Place known objects at known distances
2. Compare LiDAR readings with actual distances
3. Adjust calibration if needed

**Verify Range Limits:**
- Test minimum range (0.15m)
- Test maximum range (12m)
- Check for blind spots

## LiDAR Specifications

- **Model**: RPLidar A1M8
- **Range**: 0.15m - 12m
- **Angular Resolution**: 0.33°
- **Scan Rate**: 5.5 Hz
- **Interface**: Serial (USB)

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

*For camera operations, see [Camera Operations](../camera/camera_operations.md)*
*For IMU operations, see [IMU Operations](imu_operations.md)*
*For troubleshooting, see [Troubleshooting](../troubleshooting/index.md)*
