# Error Codes Reference

This document provides a reference for error codes and messages that may appear in the RCR Common Robotics Platform.

## ROS2 Error Codes

### Node Errors

**Node Not Found**
- **Code**: `NODE_NOT_FOUND`
- **Cause**: Node not running or not found
- **Solution**: Check if node is launched, verify node name

**Node Already Running**
- **Code**: `NODE_ALREADY_RUNNING`
- **Cause**: Multiple instances of same node
- **Solution**: Kill existing node, restart system

### Topic Errors

**Topic Not Found**
- **Code**: `TOPIC_NOT_FOUND`
- **Cause**: Topic not published or wrong name
- **Solution**: Check topic list, verify publisher

**Topic Connection Failed**
- **Code**: `TOPIC_CONNECTION_FAILED`
- **Cause**: Network or communication issue
- **Solution**: Check network, restart nodes

### Service Errors

**Service Not Available**
- **Code**: `SERVICE_NOT_AVAILABLE`
- **Cause**: Service not running or not found
- **Solution**: Check service list, verify service name

**Service Call Failed**
- **Code**: `SERVICE_CALL_FAILED`
- **Cause**: Service error or timeout
- **Solution**: Check service logs, verify parameters

## Hardware Error Codes

### Power System Errors

**Low Battery**
- **Code**: `LOW_BATTERY`
- **Cause**: Battery voltage below threshold
- **Solution**: Charge battery, check battery health

**Power Supply Failure**
- **Code**: `POWER_SUPPLY_FAILURE`
- **Cause**: Power supply malfunction
- **Solution**: Check power connections, replace supply

**Overcurrent Protection**
- **Code**: `OVERCURRENT_PROTECTION`
- **Cause**: Current draw too high
- **Solution**: Check for shorts, reduce load

### Motor System Errors

**Motor Driver Error**
- **Code**: `MOTOR_DRIVER_ERROR`
- **Cause**: Motor driver malfunction
- **Solution**: Check driver connections, replace driver

**Encoder Error**
- **Code**: `ENCODER_ERROR`
- **Cause**: Encoder malfunction or disconnection
- **Solution**: Check encoder wiring, replace encoder

**Motor Overload**
- **Code**: `MOTOR_OVERLOAD`
- **Cause**: Motor current too high
- **Solution**: Reduce load, check for binding

### Sensor Errors

**LiDAR Error**
- **Code**: `LIDAR_ERROR`
- **Cause**: LiDAR malfunction or disconnection
- **Solution**: Check USB connection, restart LiDAR

**Camera Error**
- **Code**: `CAMERA_ERROR`
- **Cause**: Camera malfunction or disconnection
- **Solution**: Check USB connection, restart camera

**IMU Error**
- **Code**: `IMU_ERROR`
- **Cause**: IMU malfunction or disconnection
- **Solution**: Check I2C connection, restart IMU

## Navigation Error Codes

### Localization Errors

**Localization Lost**
- **Code**: `LOCALIZATION_LOST`
- **Cause**: Robot lost position in map
- **Solution**: Set initial pose, check sensor data

**Poor Localization**
- **Code**: `POOR_LOCALIZATION`
- **Cause**: Low localization confidence
- **Solution**: Improve sensor data, tune parameters

### Path Planning Errors

**No Path Found**
- **Code**: `NO_PATH_FOUND`
- **Cause**: No valid path to goal
- **Solution**: Check goal accessibility, adjust parameters

**Path Planning Failed**
- **Code**: `PATH_PLANNING_FAILED`
- **Cause**: Path planning algorithm error
- **Solution**: Check costmap, restart planner

### Navigation Errors

**Navigation Failed**
- **Code**: `NAVIGATION_FAILED`
- **Cause**: Navigation system error
- **Solution**: Check localization, restart navigation

**Goal Unreachable**
- **Code**: `GOAL_UNREACHABLE`
- **Cause**: Goal location not accessible
- **Solution**: Set different goal, check obstacles

## System Error Codes

### Communication Errors

**Serial Communication Error**
- **Code**: `SERIAL_COMM_ERROR`
- **Cause**: Serial port communication failure
- **Solution**: Check serial connection, restart device

**USB Communication Error**
- **Code**: `USB_COMM_ERROR`
- **Cause**: USB communication failure
- **Solution**: Check USB connection, restart device

**Network Communication Error**
- **Code**: `NETWORK_COMM_ERROR`
- **Cause**: Network communication failure
- **Solution**: Check network connection, restart network

### File System Errors

**File Not Found**
- **Code**: `FILE_NOT_FOUND`
- **Cause**: Required file missing
- **Solution**: Check file path, restore file

**Permission Denied**
- **Code**: `PERMISSION_DENIED`
- **Cause**: Insufficient permissions
- **Solution**: Check file permissions, use sudo if needed

**Disk Space Full**
- **Code**: `DISK_SPACE_FULL`
- **Cause**: Insufficient disk space
- **Solution**: Free up disk space, clean logs

## Error Handling

### Error Logging

**System Logs**
```bash
# Check system logs
journalctl -u ros2
journalctl -u robot

# Check ROS2 logs
ros2 log list
ros2 log show
```

**Application Logs**
```bash
# Check application logs
tail -f /var/log/robot.log
tail -f ~/.ros/log/latest/robot.log
```

### Error Recovery

**Automatic Recovery**
- System attempts automatic recovery
- Restart failed components
- Fallback to safe mode

**Manual Recovery**
- Follow recovery procedures
- Restart system components
- Check error logs

### Error Prevention

**Regular Maintenance**
- Perform regular maintenance
- Monitor system health
- Update software regularly

**Monitoring**
- Monitor system performance
- Check for early warning signs
- Implement health checks

## Diagnostic Tools

### System Diagnostics
```bash
# Check system resources
htop
df -h
free -h

# Check network
ip addr show
ping google.com

# Check USB devices
lsusb
ls /dev/tty*
```

### ROS2 Diagnostics
```bash
# Check ROS2 status
ros2 node list
ros2 topic list
ros2 service list

# Monitor topics
ros2 topic hz /${ROS_NAME}/topic_name
ros2 topic echo /${ROS_NAME}/topic_name
```

### Hardware Diagnostics
```bash
# Check I2C devices
i2cdetect -y 1

# Check serial devices
ls /dev/tty*

# Check power
cat /sys/class/power_supply/BAT0/voltage_now
```

## Error Reporting

### Reporting Errors

When reporting errors, include:

1. **Error Code**: Specific error code
2. **Error Message**: Full error message
3. **Context**: What was happening when error occurred
4. **System State**: Current system state
5. **Logs**: Relevant log entries
6. **Steps to Reproduce**: How to reproduce the error

### Error Tracking

- Document all errors
- Track error frequency
- Monitor error trends
- Update error codes as needed

---

*For common issues, see [Common Issues](common_issues.md)*
*For recovery procedures, see [Recovery Procedures](recovery_procedures.md)*
*For troubleshooting, see [Troubleshooting](../troubleshooting/index.md)*
