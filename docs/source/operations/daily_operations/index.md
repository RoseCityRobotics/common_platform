# Daily Operations

This section covers routine daily operations for the PARTS Common Robotics Platform.

## Daily Procedures

### Startup and Shutdown
- [Startup Procedure](startup_procedure.md) - How to properly start the robot
- [Shutdown Procedure](shutdown_procedure.md) - Safe shutdown procedures

### Robot Control
- [Keyboard Teleoperation](KeyboardTeleop.md) - Control robot using keyboard commands
- [ROS2 Joystick Control](ROS2_Joystick.md) - Control robot using Bluetooth gamepad

### Maintenance
- [Routine Maintenance](maintenance.md) - Regular maintenance tasks

## Daily Checklist

### Before Operation
- [ ] Check battery charge level
- [ ] Verify all connections are secure
- [ ] Ensure operating environment is safe
- [ ] Check for any error indicators

### During Operation
- [ ] Monitor system performance
- [ ] Watch for unusual behavior
- [ ] Keep emergency stop accessible
- [ ] Document any issues

### After Operation
- [ ] Perform proper shutdown
- [ ] Clean sensors if needed
- [ ] Check for any damage
- [ ] Update operation logs

## Safety Reminders

- Always follow startup/shutdown procedures
- Monitor system during operation
- Keep emergency stop accessible
- Report any issues immediately

## Quick Reference

### Common Commands
```bash
# Start robot
ros2 launch common_platform launch_robot.launch.py

# Check system status
ros2 node list
ros2 topic list

# Emergency stop
# Press physical emergency stop button
```

### Troubleshooting
- Check [Common Issues](../troubleshooting/common_issues.md)
- Review system logs
- Contact support if needed

---

*For setup procedures, see [Setup](../setup/index.md)*
*For troubleshooting, see [Troubleshooting](../troubleshooting/index.md)*
