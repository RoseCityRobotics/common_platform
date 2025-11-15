---
type: module
slug: copy-cartographer-launch
title: Set up Cartographer launch files
description: Copy and build cartographer launch files for SLAM operations.
tags: [slam, cartographer, setup]
device: pi
---

# Set up Cartographer launch files

{{pi}} **On the Raspberry Pi**

This is a one-time setup to prepare Cartographer for SLAM mapping.

## Copy launch file to cartographer_ros

```bash
cp ~/repos/common_platform/launch/cartographer_simple.launch.py ~/ros2_ws/src/cartographer_ros/cartographer_ros/launch/
```

## Build the cartographer_ros package

```bash
cd ~/ros2_ws
colcon build --packages-select cartographer_ros --symlink-install
```

:::{note}
You only need to run this setup once. The `--symlink-install` flag creates symbolic links instead of copying files, making future updates easier.
:::

