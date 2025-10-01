# Robot Startup Procedure

This document outlines the proper procedure for starting up your PARTS Common Robotics Platform.

## Pre-Startup Checklist

Before starting the robot, verify:

- [ ] Battery is charged (check voltage with multimeter)
- [ ] All connections are secure
- [ ] Robot is in a safe operating environment
- [ ] Emergency stop is accessible
- [ ] No obstacles in immediate area

## Startup Sequence

### 1. Power On

1. **Battery Check**
   ```bash
   # Check battery voltage (should be >11V)
   # Connect multimeter to battery terminals
   ```

2. **Power Switch**
   - Turn on main power switch
   - Verify power LED indicators are on
   - Check that all systems receive power

### 2. System Initialization

1. **Controller Boot**
   - Wait for controller to complete boot sequence
   - Verify boot messages on console (if available)
   - Check for any error messages

2. **Sensor Initialization**
   - LiDAR should begin spinning
   - Camera should initialize
   - IMU should calibrate automatically

### 3. Software Launch

1. **ROS2 Launch**
   ```bash
   # Launch the main robot system
   ros2 launch common_platform launch_robot.launch.py
   ```

2. **Verify Topics**
   ```bash
   # Check that all expected topics are publishing
   ros2 topic list
   ```

3. **Check Node Status**
   ```bash
   # Verify all nodes are running
   ros2 node list
   ```

### 4. System Verification

1. **Sensor Data**
   - Verify LiDAR data is being published
   - Check camera feed
   - Confirm IMU data is available

2. **Motor System**
   - Test motor responsiveness
   - Verify encoder data
   - Check for any unusual sounds

3. **Navigation System**
   - Confirm localization is working
   - Check map loading (if applicable)
   - Verify path planning capabilities

## Expected Startup Time

- **Hardware initialization**: 10-15 seconds
- **Software launch**: 30-60 seconds
- **Full system ready**: 1-2 minutes

## Troubleshooting Startup Issues

### Common Problems

**Robot Won't Power On**
- Check battery voltage
- Verify power switch operation
- Check for loose connections

**Software Won't Launch**
- Check network connectivity
- Verify ROS2 installation
- Review launch file parameters

**Sensors Not Responding**
- Check sensor connections
- Verify power to sensors
- Review sensor configuration files

**Motor Issues**
- Check motor driver connections
- Verify encoder wiring
- Test motor drivers individually

## Safety Reminders

- Always perform startup in a controlled environment
- Keep emergency stop accessible
- Monitor system during startup
- Stop immediately if any unusual behavior is observed

## Next Steps

After successful startup:

1. Perform basic movement tests
2. Check sensor data quality
3. Verify navigation capabilities
4. Proceed with intended operations

---

*For shutdown procedures, see [Shutdown Procedure](shutdown_procedure.md)*
*For troubleshooting, see [Troubleshooting](../troubleshooting/index.md)*
