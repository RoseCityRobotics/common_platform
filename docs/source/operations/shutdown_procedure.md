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

