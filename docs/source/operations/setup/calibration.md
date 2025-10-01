# Calibration Procedures

This guide covers the calibration procedures for sensors and systems on the PARTS Common Robotics Platform.

## Prerequisites

- Hardware and software setup completed
- Robot in a stable, level environment
- Calibration tools (ruler, level, etc.)
- Access to robot console/terminal

## IMU Calibration

### 1. Static Calibration

```bash
# Launch robot with IMU
ros2 launch common_platform launch_robot.launch.py

# Monitor IMU data
ros2 topic echo /imu/data
```

### 2. Calibration Steps

1. **Level the Robot**
   - Place robot on level surface
   - Ensure no movement during calibration

2. **Run Calibration**
   ```bash
   # Use calibration tool
   ros2 run imu_calibration calibrate_imu
   ```

3. **Verify Results**
   - Check for zero bias values
   - Verify stable readings

## LiDAR Calibration

### 1. Mounting Verification

- Ensure LiDAR is level
- Check for obstructions
- Verify mounting stability

### 2. Range Calibration

```bash
# Monitor LiDAR data
ros2 topic echo /scan

# Check for consistent readings
ros2 run rqt_plot rqt_plot /scan/ranges
```

### 3. Angular Calibration

- Verify 360° coverage
- Check for blind spots
- Test at different distances

## Camera Calibration

### 1. Intrinsic Calibration

```bash
# Launch camera
ros2 launch common_platform camera.launch.py

# Run calibration
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 image:=/camera/image_raw camera:=/camera
```

### 2. Extrinsic Calibration

- Align camera with robot frame
- Verify coordinate transformations
- Test with known objects

## Motor Calibration

### 1. Encoder Calibration

```bash
# Test encoder counts
ros2 topic echo /wheel_odom

# Verify counts per revolution
ros2 run motor_calibration calibrate_encoders
```

### 2. PID Tuning

```bash
# Launch motor control
ros2 launch common_platform motor_control.launch.py

# Tune PID parameters
ros2 run rqt_reconfigure rqt_reconfigure
```

## Odometry Calibration

### 1. Wheel Diameter

- Measure actual wheel diameter
- Update configuration parameters
- Test with known distances

### 2. Wheelbase

- Measure distance between wheels
- Update robot description
- Verify turning radius

## System Integration

### 1. Coordinate Frames

```bash
# Check transform tree
ros2 run tf2_tools view_frames

# Verify frame relationships
ros2 run tf2_ros tf2_echo base_link laser_link
```

### 2. Sensor Fusion

- Verify sensor data alignment
- Check for timing issues
- Test data consistency

## Calibration Verification

### 1. Static Tests

- Robot should report zero velocity when stationary
- IMU should show level orientation
- LiDAR should show consistent environment

### 2. Dynamic Tests

- Move robot known distances
- Verify odometry accuracy
- Test sensor data during movement

## Troubleshooting

### Common Issues

**IMU Drift**
- Check mounting stability
- Verify calibration parameters
- Consider temperature effects

**LiDAR Inconsistencies**
- Check for obstructions
- Verify mounting alignment
- Test in different environments

**Odometry Errors**
- Verify encoder connections
- Check wheel diameter measurements
- Test on different surfaces

## Maintenance

### Regular Calibration

- Monthly IMU calibration
- Quarterly LiDAR verification
- Annual full system calibration

### Documentation

- Record calibration values
- Note environmental conditions
- Track performance over time

---

*For hardware setup, see [Hardware Setup](hardware_setup.md)*
*For software setup, see [Software Setup](software_setup.md)*
*For troubleshooting, see [Troubleshooting](../troubleshooting/index.md)*
