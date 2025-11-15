---
type: module
slug: set-ros-namespacing
title: Set ROS namespace for your robot
description: Configure ROS namespace to allow multiple robots to operate on the same network.
tags: [setup, ros, namespace]
device: pi
---

# Set ROS namespace for your robot

{{pi}} **On the Raspberry Pi**

Since we are running a swarm of many robots, ROS nodes and topics need to be namespaced. To ensure your robot runs under its unique namespace (e.g., `/rcr001`, `/rcr002`, etc.), follow these steps:

## Set your namespace in `.profile`

Edit your shell profile to define your robot's namespace:

```bash
sudo nano ~/.profile
```

Add or update this line (replace `n` with your robot number):

```bash
export ROS_NAME=rcr00n
```

Save and close the file, then reload your profile:

```bash
source ~/.profile
```

This ensures that your environment variables are set on login and immediately available in your current session.

## Set namespace in `env.list`

Update the environment variable file used for Docker and ROS:

```bash
sudo nano ~/env.list
```

Add or update the line:

```bash
ROS_NAMESPACE=/rcr00n
```

## Set firmware namespace

In your firmware directory, update the namespace inside the `RosInterface.cpp` file:

```bash
cd ~/repos/common_platform/firmware/closed_loop/
sudo nano RosInterface.cpp
```

Find this line where the node is initialized with a namespace (default should either be empty `""` or `rcr001`). Change it to `rcr00n`:

```cpp
rclc_node_init_default(&node, "micro_ros_arduino_node", "rcr00n", &support));
```

This sets the micro-ROS node namespace properly for communication with ROS2.

:::{warning}
**Important:** After updating the firmware, you must re-flash the Teensy. See the [flash-teensy](flash-teensy.md) module for instructions.
:::

