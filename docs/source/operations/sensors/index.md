# Sensor Operations

This section covers sensor operations and maintenance for the RCR Common Robotics Platform.

## Sensor Types

### LiDAR Operations
- [LiDAR Operations](lidar_operations.md) - LiDAR setup and troubleshooting

### Camera Operations
- [Camera Operations](../../camera/camera_operations.md) - Camera configuration and usage

### IMU Operations
- [IMU Operations](imu_operations.md) - IMU calibration and monitoring

## Sensor Overview

The robot is equipped with several key sensors:

1. **LiDAR** - For obstacle detection and mapping
2. **Camera** - For visual perception and navigation
3. **IMU** - For orientation and motion sensing
4. **Encoders** - For wheel odometry

## Quick Reference

### Check Sensor Status
```bash
# List all sensor topics
ros2 topic list | grep -E "(scan|camera|imu|odom)"

# Check sensor data rates
ros2 topic hz /${ROS_NAME}/scan
ros2 topic hz /${ROS_NAME}/camera/image_raw
ros2 topic hz /${ROS_NAME}/imu/data
```

### Common Commands
```bash
# Launch sensors
ros2 launch common_platform camera.launch.py
ros2 launch common_platform rplidar.launch.py

# Monitor sensor data
ros2 topic echo /${ROS_NAME}/scan
ros2 topic echo /${ROS_NAME}/camera/image_raw
ros2 topic echo /${ROS_NAME}/imu/data
```

## Troubleshooting

Common sensor issues:
- No data from sensors
- Poor data quality
- Calibration problems
- Connection issues

See [Troubleshooting](../troubleshooting/index.md) for solutions.

---

*For setup procedures, see [Setup](../setup/index.md)*
*For troubleshooting, see [Troubleshooting](../troubleshooting/index.md)*
