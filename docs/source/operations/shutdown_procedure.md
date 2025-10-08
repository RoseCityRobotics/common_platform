# Robot Shutdown Procedure

Simple shutdown procedure for the Common Robotics Platform.

## Quick Shutdown Steps

### 1. Stop Software
```bash
# Kill all running processes
Ctrl+C
```

### 2. Shutdown Raspberry Pi
```bash
# Shutdown the Pi
sudo shutdown now

# Or reboot if needed
sudo reboot
```

### 3. Turn Off Hardware
- Turn off Teensy power switch
- Remove Pi battery
- Remove Teensy battery

### 4. Recharge Batteries
- Recharge both Pi and Teensy batteries
- Store in designated charging area

## Emergency Shutdown
- Press emergency stop button
- Turn off power switches
- Remove batteries immediately

---

*For startup procedures, see [Startup Procedure](startup_procedure.md)*

---

*For startup procedures, see [Startup Procedure](startup_procedure.md)*
*For maintenance, see [Routine Maintenance](maintenance.md)*
*For troubleshooting, see [Troubleshooting](../troubleshooting/index.md)*
