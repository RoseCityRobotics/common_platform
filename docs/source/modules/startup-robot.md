---
type: module
slug: startup-robot
title: Power on the robot
description: Proper procedure for powering on the robot hardware.
tags: [startup, power, hardware]
device: hardware
---

# Power on the robot

## Pre-Startup Checklist

Before starting the robot, verify:

- [ ] Battery is charged
- [ ] All connections are secure
- [ ] Robot is in a safe operating environment
- [ ] Emergency stop is accessible
- [ ] No obstacles in immediate area

## Power On Sequence

1. **Add batteries to Teensy**
2. **Connect Pi Battery**
3. **Turn on main Teensy power switch**
4. **Verify Teensy and Pi power LED indicators are on**

## System Initialization

**Controller Boot**
- Wait for controller to complete boot sequence (approximately 10-15 seconds)
- Verify boot messages on console (if available)
- Check for any error messages

**Sensor Initialization**
- LiDAR should begin spinning
- Camera should initialize
- IMU should calibrate automatically

:::{note}
**Expected startup time:**
- Hardware initialization: 10-15 seconds
- Software launch: 30-60 seconds
- Full system ready: 1-2 minutes
:::

:::{tip}
If the Pi doesn't boot, check battery voltage. It should be greater than 9V.
:::

