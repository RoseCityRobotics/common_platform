# Shutdown Procedure

Simple shutdown procedure for the Common Robotics Platform.

## Stop Software
```bash
# Kill all running processes
Ctrl+C
```

## Shutdown Raspberry Pi
```bash
sudo shutdown now
```

This command will safely power down the Raspberry Pi and all connected systems.

> 💡 **Tip:** Always use the proper shutdown command to avoid data corruption and ensure the system starts cleanly next time.

## Turn Off Hardware
- Turn off Teensy power switch
- Remove Pi battery
- Remove Teensy battery

## Recharge Batteries
- Recharge both Pi and Teensy batteries
- Store in designated charging area

# Emergency Shutdown

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

# Safety Best Practices
- Never leave your batteries in your robot when its not in use, leave them in your battery chargers
- Remove batteries before plugging into external power
