# Emergency Shutdown

Emergency procedures for the RCR Common Robotics Platform when immediate shutdown is required.

## When to Use Emergency Shutdown

Use emergency shutdown procedures when:
- Battery overheating or smoking
- Fire or smoke from batteries
- System malfunction requiring immediate power cut
- Safety hazard requiring instant shutdown

## Emergency Shutdown Steps

### 1. Immediate Power Cut
```bash
# Kill all processes immediately
Ctrl+C (in all terminal windows)

# Force shutdown if system is unresponsive
sudo shutdown -h now
```

### 2. Physical Power Disconnect
- Turn off Teensy power switch immediately
- Remove Pi battery if safe to do so
- Remove Teensy battery if safe to do so
- Disconnect any external power sources

## ⚠️ LITHIUM-ION BATTERY SAFETY

**If you notice heat, smoke, or fire from the batteries:**

### Immediate Actions
1. **Turn off all power switches immediately**
2. **Remove batteries if safe to do so (wear protective gloves)**
3. **Move robot to a safe outdoor location if possible**

### Lithium Fire Safety
- **DO NOT** use water on lithium fires
- Lithium fires need to burn out completely
- Use a Class D fire extinguisher if available
- If fire starts, move robot outside and let it burn out safely
- Call emergency services (911) if needed

### After Emergency
- Do not attempt to recharge damaged batteries
- Dispose of damaged batteries properly at a hazardous waste facility
- Report the incident immediately

### Battery Safety Best Practices

**Always follow these safety guidelines:**

- **Keep batteries in chargers when not in use** - Never leave batteries installed in the robot when connected to external power
- **Remove batteries during charging** - Only charge batteries in designated charging stations, not while installed in the robot
- **Monitor charging process** - Never leave batteries charging unattended
- **Use proper storage** - Store batteries in a cool, dry location away from direct sunlight

## Normal Shutdown vs Emergency

**Normal Shutdown**: Use [Shutdown Procedure](shutdown_procedure.md) for routine shutdowns
**Emergency Shutdown**: Use this procedure only when immediate shutdown is required for safety

---

*For normal shutdown procedures, see [Shutdown Procedure](shutdown_procedure.md)*
*For startup procedures, see [Startup Procedure](startup_procedure.md)*
*For troubleshooting, see [Troubleshooting](../troubleshooting/index.md)*
