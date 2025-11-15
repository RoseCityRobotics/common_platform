---
type: module
slug: start-keyboard-teleop
title: Start keyboard teleoperation
description: Launch the keyboard teleoperation node to control the robot with arrow keys.
tags: [teleop, keyboard, control]
device: pi
---

# Start keyboard teleoperation

{{pi}} **On the Raspberry Pi**

Open a new terminal window {{terminal}}

This node allows you to control your robot using keyboard inputs.

```bash
cd ~/repos/common_platform/common_platform_ws
source install/setup.bash
ros2 launch evdev_teleop evdev_teleop.launch.py
```

This command launches the `evdev_teleop` package, which sets up a ROS2 node to read keyboard input and publish corresponding control commands.

## Keyboard Commands 🎮

:::{warning}
**Important:** Put your robot on the ground before starting teleoperation!
:::

| Command | Key | Action |
|---------|-----|--------|
| ⬆️ | **Forward Arrow** | Move forward |
| ⬇️ | **Backward Arrow** | Move backward |
| ⬅️ | **Left Arrow** | Turn left |
| ➡️ | **Right Arrow** | Turn right |
| ⏸️ | **Spacebar** | Stop/Emergency brake |

