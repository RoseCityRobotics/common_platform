---
type: module
slug: view-robot-slam
title: Visualize SLAM with RViz
description: Launch RViz to visualize the robot and SLAM mapping process.
tags: [slam, rviz, visualization]
device: pc
---

# Visualize SLAM with RViz

{{pc}} **On the development computer**

Use RViz to visualize the robot model, sensor data, and the map being built during SLAM.

## Launch RViz with robot visualization

```bash
ros2 launch common_platform view_robot.launch.py
```

This opens RViz configured to show:
- Robot model (URDF)
- LiDAR scan data
- Camera feed (if available)
- Occupancy grid map
- TF transforms

## What to monitor in RViz

1. **Robot Model** - Ensure it's displayed correctly
2. **LaserScan** - Check that LiDAR data is being received
3. **Map** - Watch the map being built in real-time
4. **TF Tree** - Verify all transforms are connected

:::{tip}
You can save your RViz configuration for future use via `File > Save Config As...`
:::

