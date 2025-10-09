# SLAM Operations

This section covers Simultaneous Localization and Mapping (SLAM) operations for the common platform robot. Learn how to create maps of your environment using Cartographer, localize your robot within existing maps, then plan and execute navigation paths.

## Prerequisites

- **Discovery Server Running**: Ensure a ROS discovery server is running (see [Discovery Server Setup](../operations/discovery_server.md))
- Odometry system working (wheel encoders and/or IMU)
- Sufficient battery charge for mapping session
- Robot control setup (see [Keyboard Teleoperation Setup](../operations/keyboard_teleoperation.md))
- LIDAR node is started (see [Lidar Operations](../lidar_operations.md))
- Make sure the cartographer launch file is installed (see [Install Cartographer Launcher](install_carto_launch.md))

## Topics

```{toctree}
:maxdepth: 1

mapping
localization
path_planning
```
