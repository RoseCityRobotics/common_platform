# Common Issues and Solutions

This document addresses frequently encountered problems with the PARTS Common Robotics Platform and their solutions.

## Communication with Teensy 4.0 Microcontroller
**View debug output**
   ```bash
   SERIAL_TEENSY_DEVICE=`find /dev/serial/by-id/ -name "usb-Teensyduino*if02"|head -1`
   python3 -m serial.tools.miniterm ${SERIAL_TEENSY_DEVICE} 115200
   ```

## Power Issues

### Robot Won't Power On

**Symptoms:**
- No LED indicators
- No response to power switch
- Dead system

**Possible Causes & Solutions:**

1. **Battery Issues**
   - **Cause**: Dead or low battery
   - **Solution**: Check battery voltage, charge if needed
   - **Prevention**: Regular battery maintenance

2. **Power Switch Failure**
   - **Cause**: Faulty power switch
   - **Solution**: Test switch continuity, replace if needed
   - **Prevention**: Handle switch gently

3. **Loose Connections**
   - **Cause**: Poor battery connections
   - **Solution**: Check and tighten all power connections
   - **Prevention**: Regular connection inspection

### Intermittent Power Loss

**Symptoms:**
- Robot shuts down unexpectedly
- System resets during operation
- Unstable power indicators

**Solutions:**
1. Check battery voltage under load
2. Inspect power cables for damage
3. Verify power distribution board connections
4. Test with known good battery

## Communication Issues

### ROS2 Nodes Not Starting

**Symptoms:**
- Missing nodes in `ros2 node list`
- Launch failures
- Communication timeouts

**Solutions:**

1. **Network Issues**
   ```bash
   # Check network connectivity
   ping <robot_ip>

   # Verify ROS2 environment
   echo $ROS_DOMAIN_ID
   ```

2. **Launch File Problems**
   ```bash
   # Test launch file syntax
   ros2 launch --dry-run common_platform launch_robot.launch.py
   ```

3. **Dependency Issues**
   ```bash
   # Check for missing packages
   rosdep check --from-paths src --ignore-src
   ```

### Sensor Communication Errors

**Symptoms:**
- No sensor data on topics
- Error messages about sensor connections
- Inconsistent sensor readings

**Solutions:**

1. **USB Connection Issues**
   - Check USB cable integrity
   - Try different USB ports
   - Verify device permissions

2. **Serial Communication Problems**
   - Check baud rate settings
   - Verify serial port assignments
   - Test with serial terminal

3. **I2C Bus Issues**
   - Check I2C device addresses
   - Verify pull-up resistors
   - Test I2C bus with tools

## Navigation Issues

### Localization Problems

**Symptoms:**
- Robot loses position
- Incorrect pose estimates
- Navigation failures

**Solutions:**

1. **Map Issues**
   - Verify map file integrity
   - Check map resolution and origin
   - Reload map if corrupted

2. **Sensor Data Quality**
   - Check LiDAR data quality
   - Verify IMU calibration
   - Clean sensor surfaces

3. **Algorithm Parameters**
   - Tune localization parameters
   - Adjust particle filter settings
   - Review AMCL configuration

### Path Planning Failures

**Symptoms:**
- No valid paths found
- Robot gets stuck
- Inefficient path planning

**Solutions:**

1. **Costmap Issues**
   - Check obstacle detection
   - Verify inflation parameters
   - Review costmap layers

2. **Goal Issues**
   - Ensure goal is reachable
   - Check goal tolerance settings
   - Verify coordinate frames

## Motor and Movement Issues

### Motors Not Responding

**Symptoms:**
- No movement on command
- Motors make noise but don't move
- Uneven motor performance

**Solutions:**

1. **Motor Driver Issues**
   - Check motor driver connections
   - Verify driver power supply
   - Test driver functionality

2. **Encoder Problems**
   - Check encoder wiring
   - Verify encoder counts
   - Test encoder resolution

3. **Control Loop Issues**
   - Check PID parameters
   - Verify control loop timing
   - Review motor specifications

### Uneven Movement

**Symptoms:**
- Robot drifts to one side
- Uneven wheel speeds
- Curved movement when going straight

**Solutions:**

1. **Calibration Issues**
   - Recalibrate wheel diameters
   - Check wheelbase measurements
   - Verify encoder counts per revolution

2. **Mechanical Issues**
   - Check wheel alignment
   - Verify tire pressure
   - Inspect for mechanical binding

## Diagnostic Commands

### System Health Check
```bash
# Check system resources
htop

# Check disk space
df -h

# Check network connectivity
ip addr show
```

### ROS2 Diagnostics
```bash
# List all nodes
ros2 node list

# Check topic rates
ros2 topic hz /${ROS_NAME}/topic_name

# Monitor system performance
ros2 run rqt_graph rqt_graph
```

### Hardware Diagnostics
```bash
# Check USB devices
lsusb

# Check serial devices
ls /dev/tty*

# Check I2C devices
i2cdetect -y 1
```

## When to Seek Help

Contact the PARTS team if:

- Issues persist after trying solutions
- Hardware appears damaged
- Error messages are unclear
- System behavior is dangerous

## Prevention Tips

1. **Regular Maintenance**
   - Clean sensors regularly
   - Check connections monthly
   - Update software regularly

2. **Proper Operation**
   - Follow startup/shutdown procedures
   - Use appropriate operating environments
   - Monitor system during operation

3. **Documentation**
   - Keep logs of issues and solutions
   - Document any modifications
   - Share solutions with community

---

*For more detailed troubleshooting, see [Recovery Procedures](recovery_procedures.md)*
*For error code reference, see [Error Codes](error_codes.md)*
