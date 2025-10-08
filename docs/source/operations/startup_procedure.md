# Robot Startup Procedure

This document outlines the proper procedure for starting up your PARTS Common Robotics Platform.

## Pre-Startup Checklist

Before starting the robot, verify:

- [ ] Battery is charged
- [ ] All connections are secure
- [ ] Robot is in a safe operating environment
- [ ] Emergency stop is accessible
- [ ] No obstacles in immediate area


## Power On
- Add batteries to Teensy
- Connect Pi Battery
- Turn on main Teensy power switch
- Verify Teensy and Pi power LED indicators are on

## System Initialization

**Controller Boot**
   - Wait for controller to complete boot sequence
   - Verify boot messages on console (if available)
   - Check for any error messages

**Sensor Initialization**
   - LiDAR should begin spinning
   - Camera should initialize
   - IMU should calibrate automatically

## Software Launch

**ROS2 Launch**
   ```bash
   # Launch the main robot system
   ros2 launch common_platform launch_robot.launch.py
   ```

Open a new terminal window 📟

**Verify Topics**
   ```bash
   # Check that all expected topics are publishing
   ros2 topic list
   ```

**Check Node Status**
   ```bash
   # Verify all nodes are running
   ros2 node list
   ```

## System Verification

 **Sensor Data**
   - Verify LiDAR data is being published
   - Check camera feed
   - Confirm IMU data is available

 **Motor System**
   - Test motor responsiveness
   - Verify encoder data
   - Check for any unusual sounds

 **Navigation System**
   - Confirm localization is working
   - Check map loading (if applicable)
   - Verify path planning capabilities

## Expected Startup Time

- **Hardware initialization**: 10-15 seconds
- **Software launch**: 30-60 seconds
- **Full system ready**: 1-2 minutes

## Troubleshooting Startup Issues

### Common Problems

**Battery Check**
   ```bash
   # Check battery voltage (should be >9V)
   # Connect multimeter to battery terminals
   ```

**Discovery Server Running**
   - Ensure a ROS discovery server is running before starting robot operations
   - See [Discovery Server Setup](discovery_server.md) for detailed setup instructions
   - If at Rose City Robotics Lab, a discovery server is already running on the network

---

*For shutdown procedures, see [Shutdown Procedure](shutdown_procedure.md)*
*For troubleshooting, see [Troubleshooting](../troubleshooting/index.md)*
