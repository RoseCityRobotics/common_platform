# Firmware and Teleoperation

This procedure covers flashing the Teensy firmware and setting up keyboard-based teleoperation for your robot.

## Overview

You'll complete the following steps:
1. Compile and flash the Teensy firmware
2. Power on the robot
3. Monitor Teensy debug output
4. Start the micro-ROS agent
5. Start keyboard teleoperation

---

## Step 1: Flash the Teensy Firmware

```{include} ../modules/flash-teensy.md
:start-after: "# Compile and flash the Teensy firmware"
```

---

## Step 2: Power On the Robot

```{include} ../modules/startup-robot.md
:start-after: "# Power on the robot"
```

---

## Step 3: Monitor Teensy Serial Output

```{include} ../modules/monitor-teensy-serial.md
:start-after: "# Monitor Teensy debug output"
```

---

## Step 4: Start the micro-ROS Agent

```{include} ../modules/start-microros-agent.md
:start-after: "# Start the micro-ROS agent"
```

---

## Step 5: Start Keyboard Teleoperation

```{include} ../modules/start-keyboard-teleop.md
:start-after: "# Start keyboard teleoperation"
```

---

## Next Steps

Now that your robot is running:
- [Create a map with SLAM](slam-mapping.md)
- [Test navigation](navigation.md)
- [Configure sensors](../operations/sensors/index.md)

