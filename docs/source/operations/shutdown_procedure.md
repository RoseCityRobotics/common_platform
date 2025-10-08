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

**⚠️ LITHIUM-ION BATTERY SAFETY**

If you notice heat, smoke, or fire from the batteries:

1. **Immediate Actions**
   - Turn off all power switches immediately
   - Remove batteries if safe to do so (wear protective gloves)
   - Move robot to a safe outdoor location if possible

2. **Lithium Fire Safety**
   - **DO NOT** use water on lithium fires
   - Lithium fires need to burn out completely
   - Use a Class D fire extinguisher if available
   - If fire starts, move robot outside and let it burn out safely
   - Call emergency services (911) if needed

3. **After Emergency**
   - Do not attempt to recharge damaged batteries
   - Dispose of damaged batteries properly at a hazardous waste facility
   - Report the incident immediately

---

*For startup procedures, see [Startup Procedure](startup_procedure.md)*

---

*For startup procedures, see [Startup Procedure](startup_procedure.md)*
*For maintenance, see [Routine Maintenance](maintenance.md)*
*For troubleshooting, see [Troubleshooting](../troubleshooting/index.md)*
