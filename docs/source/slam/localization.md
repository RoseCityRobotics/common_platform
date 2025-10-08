# Localization Operations

This guide covers robot localization procedures for the PARTS Common Robotics Platform.

## Overview

Localization is the process of determining the robot's position and orientation within a known map. The system uses AMCL (Adaptive Monte Carlo Localization) to track the robot's pose.

## Prerequisites

- Map of the environment available
- LiDAR sensor functioning
- IMU and odometry data available
- Navigation stack configured

## Localization Setup

### 1. Launch Localization

```bash
# Launch localization system
ros2 launch common_platform localization_launch.py
```

### 2. Verify Topics

```bash
# Check required topics
ros2 topic list | grep -E "(scan|odom|map|amcl)"
```

Expected topics:
- `/scan` - LiDAR data
- `/odom` - Odometry data
- `/map` - Map data
- `/amcl_pose` - Localized pose

## Initial Localization

### 1. Set Initial Pose

**Using RViz:**
1. Open RViz
2. Click "2D Pose Estimate"
3. Click and drag on map to set initial pose

**Using Command Line:**
```bash
# Set initial pose
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{}"
```

### 2. Verify Localization

```bash
# Monitor pose estimate
ros2 topic echo /${ROS_NAME}/amcl_pose

# Check pose covariance
ros2 topic echo /${ROS_NAME}/amcl_pose | grep covariance
```

## Localization Parameters

### AMCL Configuration

Key parameters in `nav2_params.yaml`:

```yaml
amcl:
  ros__parameters:
    # Particle filter parameters
    min_particles: 500
    max_particles: 2000

    # Update parameters
    update_min_d: 0.2
    update_min_a: 0.5

    # Laser model parameters
    laser_max_range: 10.0
    laser_min_range: 0.1
```

### Tuning Guidelines

**For Better Accuracy:**
- Increase particle count
- Decrease update thresholds
- Tune laser model parameters

**For Better Performance:**
- Decrease particle count
- Increase update thresholds
- Optimize laser parameters

## Localization Monitoring

### 1. Pose Quality

**Check Covariance:**
```bash
# Monitor pose covariance
ros2 topic echo /amcl_pose | grep covariance
```

Low covariance values indicate good localization.

### 2. Particle Cloud

**Visualize in RViz:**
1. Add "PoseArray" display
2. Set topic to `/particlecloud`
3. Monitor particle distribution

### 3. Localization Status

```bash
# Check localization status
ros2 topic echo /${ROS_NAME}/localization_status
```

## Troubleshooting

### Common Issues

**Poor Localization:**
- Check map quality
- Verify sensor data
- Tune AMCL parameters
- Check for dynamic obstacles

**Localization Failures:**
- Verify initial pose
- Check sensor connections
- Review error logs
- Test in known location

**Drift Issues:**
- Check odometry accuracy
- Verify IMU calibration
- Review sensor fusion
- Test on different surfaces

### Diagnostic Commands

```bash
# Check sensor data quality
ros2 topic hz /${ROS_NAME}/scan
ros2 topic hz /${ROS_NAME}/odom

# Monitor localization performance
ros2 topic echo /${ROS_NAME}/amcl_pose

# Check for errors
ros2 log list
```

## Best Practices

### Environment Setup
- Use high-quality maps
- Ensure good LiDAR coverage
- Minimize dynamic obstacles
- Maintain consistent lighting

### Operation
- Set accurate initial pose
- Monitor localization quality
- Avoid areas with poor features
- Use landmarks for verification

### Maintenance
- Regular sensor cleaning
- Periodic recalibration
- Map updates as needed
- Performance monitoring

## Advanced Localization

### Multi-Hypothesis Tracking
- Handle ambiguous situations
- Maintain multiple hypotheses
- Use additional sensors

### Sensor Fusion
- Combine multiple sensors
- Use IMU for orientation
- Integrate visual features

### Dynamic Environments
- Handle moving obstacles
- Update maps in real-time
- Use adaptive algorithms

---

*For mapping procedures, see [Mapping](mapping.md)*
*For path planning, see [Path Planning](path_planning.md)*
*For troubleshooting, see [Troubleshooting](../troubleshooting/index.md)*
