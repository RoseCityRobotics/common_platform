# Robot Shutdown Procedure

This document outlines the proper procedure for safely shutting down your PARTS Common Robotics Platform.

## Pre-Shutdown Checklist

Before shutting down the robot, ensure:

- [ ] Robot is in a safe location
- [ ] No active navigation tasks
- [ ] All data has been saved
- [ ] Emergency stop is accessible

## Shutdown Sequence

### 1. Stop Active Operations

1. **Stop Navigation**
   ```bash
   # Cancel any active navigation goals
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{}"
   ```

2. **Stop Movement**
   ```bash
   # Send zero velocity command
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}"
   ```

3. **Verify Robot is Stationary**
   - Check that robot has stopped moving
   - Ensure no active motor commands
   - Verify emergency stop is not engaged

### 2. Software Shutdown

1. **Stop ROS2 Nodes**
   ```bash
   # Gracefully stop launch file
   # Press Ctrl+C in the terminal running the launch file
   ```

2. **Verify Node Shutdown**
   ```bash
   # Check that nodes have stopped
   ros2 node list
   # Should show minimal or no nodes
   ```

3. **Save Data (if needed)**
   ```bash
   # Save any important data
   # Backup logs if necessary
   ```

### 3. Hardware Shutdown

1. **Power Off Sequence**
   - Turn off main power switch
   - Wait for all LEDs to turn off
   - Verify no power indicators remain

2. **Disconnect External Devices**
   - Remove USB connections
   - Disconnect external power (if applicable)
   - Secure any loose cables

## Post-Shutdown Verification

### 1. System Check
- Verify all systems are off
- Check for any remaining power indicators
- Ensure no unusual sounds or heat

### 2. Environment Check
- Secure robot in designated location
- Check for any damage or issues
- Note any problems for next startup

## Emergency Shutdown

### Immediate Shutdown
If emergency shutdown is needed:

1. **Press Emergency Stop**
   - Locate and press emergency stop button
   - This immediately cuts power to motors

2. **Power Off**
   - Turn off main power switch
   - Disconnect battery if necessary

3. **Assess Situation**
   - Check for any damage
   - Note what caused the emergency
   - Report issues immediately

## Shutdown Troubleshooting

### Common Issues

**Software Won't Stop**
- Force kill processes if necessary
- Check for zombie processes
- Restart terminal if needed

**Hardware Won't Power Off**
- Check power switch operation
- Verify battery connections
- Use emergency stop if available

**Data Loss Concerns**
- Ensure data is saved before shutdown
- Check for unsaved changes
- Backup important files

## Maintenance After Shutdown

### Daily Maintenance
- Clean sensor surfaces
- Check for loose connections
- Inspect for damage

### Weekly Maintenance
- Check battery condition
- Verify all systems
- Update logs

## Safety Reminders

- Always follow proper shutdown sequence
- Never force shutdown unless emergency
- Keep emergency stop accessible
- Document any issues

## Next Steps

After shutdown:

1. Perform any required maintenance
2. Check robot condition
3. Plan for next operation
4. Update operation logs

---

*For startup procedures, see [Startup Procedure](startup_procedure.md)*
*For maintenance, see [Routine Maintenance](maintenance.md)*
*For troubleshooting, see [Troubleshooting](../troubleshooting/index.md)*
