# Navigation Operations

This section covers navigation-related operations for the PARTS Common Robotics Platform.

## Navigation Procedures

### Mapping and Localization
- [Mapping](mapping.md) - Creating and updating maps
- [Localization](localization.md) - Robot positioning and localization
- [Path Planning](path_planning.md) - Navigation and path planning

## Navigation Overview

The navigation system consists of several key components:

1. **Mapping** - Creating maps of the environment
2. **Localization** - Determining robot position
3. **Path Planning** - Planning routes to destinations
4. **Obstacle Avoidance** - Avoiding obstacles during navigation

## Quick Start

### Basic Navigation
```bash
# Launch navigation system
ros2 launch common_platform navigation_launch.py

# Set navigation goal
ros2 run nav2_util navigation_goal --x 2.0 --y 1.0 --yaw 0.0
```

### Mapping
```bash
# Launch SLAM
ros2 launch common_platform cartographer_2d.launch.py

# Save map
ros2 run nav2_map_server map_saver_cli -f my_map
```

## Navigation Components

### Sensors
- LiDAR for obstacle detection
- IMU for orientation
- Odometry for position tracking

### Software
- ROS2 Navigation2 stack
- Cartographer for SLAM
- AMCL for localization

## Troubleshooting

Common navigation issues:
- Localization failures
- Path planning errors
- Obstacle detection problems

See [Troubleshooting](../troubleshooting/index.md) for solutions.

---

*For setup procedures, see [Setup](../setup/index.md)*
*For troubleshooting, see [Troubleshooting](../troubleshooting/index.md)*
