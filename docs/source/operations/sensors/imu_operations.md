# IMU Operations

This guide covers IMU (Inertial Measurement Unit) operations, setup, and troubleshooting for the PARTS Common Robotics Platform.

## Overview

The robot uses an IMU for orientation sensing, motion detection, and navigation. The IMU provides acceleration, angular velocity, and orientation data.

## IMU Specifications

- **Model**: MPU9250
- **Accelerometer**: ±16g range
- **Gyroscope**: ±2000°/s range
- **Magnetometer**: ±4800μT range
- **Interface**: I2C
- **Update Rate**: 100 Hz

## IMU Setup

### 1. Hardware Connection

1. **Physical Mounting**
   - Mount IMU in stable location
   - Minimize vibration
   - Ensure proper orientation

2. **Power Connection**
   - Connect to robot power system
   - Verify voltage requirements (3.3V)
   - Check for proper grounding

3. **Data Connection**
   - Connect to I2C bus
   - Verify pull-up resistors
   - Test communication

### 2. Software Configuration

**Launch IMU:**
```bash
# Launch IMU node
ros2 launch common_platform imu.launch.py
```

**Verify Data:**
```bash
# Check IMU data
ros2 topic echo /imu/data

# Monitor data rate
ros2 topic hz /imu/data
```

## IMU Calibration

### 1. Static Calibration

**Calibration Process:**
1. Place robot on level surface
2. Ensure no movement
3. Run calibration procedure
4. Save calibration parameters

**Calibration Command:**
```bash
# Run IMU calibration
ros2 run imu_calibration calibrate_imu
```

### 2. Dynamic Calibration

**Motion Calibration:**
- Perform controlled movements
- Calibrate gyroscope bias
- Verify acceleration readings

**Orientation Calibration:**
- Align with known orientation
- Calibrate magnetometer
- Verify heading accuracy

## IMU Data Analysis

### 1. Data Structure

**IMU Message:**
```yaml
std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
```

### 2. Data Quality

**Check Data Quality:**
```bash
# Monitor IMU data
ros2 topic echo /imu/data

# Check for:
# - Stable orientation
# - Reasonable acceleration values
# - Consistent angular velocity
```

## IMU Applications

### 1. Orientation Sensing

**Quaternion Data:**
- Use for robot orientation
- Implement orientation control
- Monitor attitude changes

**Euler Angles:**
- Convert for human readability
- Use for control algorithms
- Monitor pitch, roll, yaw

### 2. Motion Detection

**Acceleration Data:**
- Detect motion changes
- Implement motion control
- Monitor for impacts

**Angular Velocity:**
- Detect rotation
- Implement rotation control
- Monitor for spinning

### 3. Navigation

**Sensor Fusion:**
- Combine with other sensors
- Improve localization
- Enhance navigation accuracy

**Dead Reckoning:**
- Use for short-term navigation
- Complement other sensors
- Handle sensor failures

## Troubleshooting

### Common Issues

**No IMU Data:**
- Check I2C connection
- Verify power supply
- Check device address
- Test with I2C tools

**Poor Data Quality:**
- Check mounting stability
- Verify calibration
- Test in different environments
- Check for interference

**Drift Issues:**
- Recalibrate IMU
- Check for temperature effects
- Verify mounting
- Test with known orientations

### Diagnostic Commands

```bash
# Check I2C devices
i2cdetect -y 1

# Test I2C communication
i2cget -y 1 0x68 0x75

# Monitor data quality
ros2 topic echo /imu/data
ros2 topic hz /imu/data
```

### Error Codes

**Common Error Messages:**
- "Failed to open I2C device" - Check I2C connection
- "No data received" - Check power and connections
- "Invalid data" - Check for interference or damage

## Maintenance

### Regular Maintenance

**Daily:**
- Visual inspection
- Check for vibration
- Verify data quality

**Weekly:**
- Check mounting stability
- Test calibration
- Monitor performance

**Monthly:**
- Full calibration check
- Performance testing
- Connection inspection

### Cleaning Procedures

**IMU Unit:**
1. Power off robot
2. Clean mounting surface
3. Check for loose connections
4. Verify stability

**Mounting Area:**
- Clean mounting surface
- Check for loose connections
- Verify stability

## Performance Optimization

### Data Processing

**Filtering:**
- Remove noise from data
- Implement low-pass filters
- Smooth data for better performance

**Sensor Fusion:**
- Combine with other sensors
- Use Kalman filters
- Improve accuracy

### Parameter Tuning

**IMU Parameters:**
```yaml
# In launch file
update_rate: 100
linear_acceleration_stddev: 0.01
angular_velocity_stddev: 0.01
```

**Filter Parameters:**
- Adjust filter gains
- Balance performance vs. accuracy
- Consider computational load

## Safety Considerations

### Operating Safety

- Handle with care during maintenance
- Follow manufacturer guidelines
- Consider temperature effects
- Monitor for failures

### Data Safety

- Verify data accuracy
- Handle sensor failures gracefully
- Implement appropriate logging
- Consider data validation

---

*For LiDAR operations, see [LiDAR Operations](lidar_operations.md)*
*For camera operations, see [Camera Operations](camera_operations.md)*
*For troubleshooting, see [Troubleshooting](../troubleshooting/index.md)*
