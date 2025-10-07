# Path Planning Operations

This guide covers path planning and navigation procedures for the PARTS Common Robotics Platform.

## Overview

Path planning involves finding a safe and efficient route from the robot's current position to a desired goal location while avoiding obstacles.

## Prerequisites

- Localization system running
- Map of the environment available
- Navigation stack configured
- Goal location specified

## Path Planning Setup

### 1. Launch Navigation

```bash
# Launch navigation system
ros2 launch common_platform navigation_launch.py
```

### 2. Verify Components

```bash
# Check navigation topics
ros2 topic list | grep -E "(goal|plan|path|cmd_vel)"
```

Expected topics:
- `/goal_pose` - Navigation goals
- `/plan` - Planned path
- `/cmd_vel` - Velocity commands

## Setting Navigation Goals

### 1. Using RViz

1. Open RViz
2. Click "2D Nav Goal"
3. Click and drag on map to set goal
4. Robot will plan and execute path

### 2. Using Command Line

```bash
# Set navigation goal
ros2 run nav2_util navigation_goal --x 2.0 --y 1.0 --yaw 0.0
```

### 3. Programmatic Goals

```python
# Python example
import rclpy
from geometry_msgs.msg import PoseStamped

# Create goal message
goal = PoseStamped()
goal.header.frame_id = "map"
goal.pose.position.x = 2.0
goal.pose.position.y = 1.0
goal.pose.orientation.w = 1.0

# Send goal
goal_pub.publish(goal)
```

## Path Planning Parameters

### Global Planner

Key parameters in `nav2_params.yaml`:

```yaml
global_costmap:
  ros__parameters:
    # Costmap parameters
    inflation_radius: 0.2
    cost_scaling_factor: 10.0
    
    # Obstacle parameters
    obstacle_range: 2.5
    raytrace_range: 3.0
```

### Local Planner

```yaml
local_costmap:
  ros__parameters:
    # Update parameters
    update_frequency: 5.0
    publish_frequency: 2.0
    
    # Inflation parameters
    inflation_radius: 0.2
    cost_scaling_factor: 10.0
```

## Path Planning Algorithms

### Global Planning

**A* Algorithm:**
- Optimal path finding
- Good for static environments
- Computationally intensive

**Dijkstra Algorithm:**
- Guaranteed shortest path
- Slower than A*
- Good for complex environments

### Local Planning

**DWA (Dynamic Window Approach):**
- Real-time obstacle avoidance
- Good for dynamic environments
- Responsive to changes

**TEB (Timed Elastic Band):**
- Smooth trajectories
- Good for non-holonomic robots
- Computationally efficient

## Monitoring Path Planning

### 1. Path Visualization

**In RViz:**
1. Add "Path" display
2. Set topic to `/plan`
3. Monitor planned path

### 2. Costmap Visualization

**Global Costmap:**
1. Add "Map" display
2. Set topic to `/global_costmap/costmap`
3. Monitor global planning

**Local Costmap:**
1. Add "Map" display
2. Set topic to `/local_costmap/costmap`
3. Monitor local planning

### 3. Performance Monitoring

```bash
# Monitor planning frequency
ros2 topic hz /plan

# Check goal status
ros2 topic echo /navigate_to_pose/_action/status
```

## Troubleshooting

### Common Issues

**No Path Found:**
- Check goal accessibility
- Verify map quality
- Adjust costmap parameters
- Check for obstacles

**Poor Path Quality:**
- Tune planner parameters
- Check costmap inflation
- Verify sensor data
- Review algorithm choice

**Navigation Failures:**
- Check localization
- Verify sensor data
- Review error logs
- Test in simple environment

### Diagnostic Commands

```bash
# Check planning status
ros2 topic echo /navigate_to_pose/_action/status

# Monitor path planning
ros2 topic echo /plan

# Check costmaps
ros2 topic echo /global_costmap/costmap
ros2 topic echo /local_costmap/costmap
```

## Path Planning Optimization

### Parameter Tuning

**For Better Performance:**
- Increase update frequencies
- Optimize costmap resolution
- Tune inflation parameters
- Adjust planning horizons

**For Better Quality:**
- Use higher resolution maps
- Increase planning time
- Tune cost functions
- Optimize algorithms

### Algorithm Selection

**Static Environments:**
- Use A* for global planning
- Use DWA for local planning
- Optimize for speed

**Dynamic Environments:**
- Use faster global planners
- Use reactive local planners
- Optimize for responsiveness

## Advanced Path Planning

### Multi-Robot Planning
- Coordinate multiple robots
- Avoid robot-robot collisions
- Optimize overall efficiency

### Dynamic Replanning
- Handle moving obstacles
- Update paths in real-time
- Maintain safety margins

### Hierarchical Planning
- Use different planners for different scales
- Combine global and local planning
- Optimize for different objectives

## Best Practices

### Environment Setup
- Use high-quality maps
- Ensure good sensor coverage
- Minimize dynamic obstacles
- Plan for safety margins

### Operation
- Set reasonable goals
- Monitor planning performance
- Handle planning failures gracefully
- Use appropriate algorithms

### Maintenance
- Regular parameter tuning
- Performance monitoring
- Algorithm updates
- Environment updates

---

*For mapping procedures, see [Mapping](mapping.md)*
*For localization, see [Localization](localization.md)*
*For troubleshooting, see [Troubleshooting](../troubleshooting/index.md)*
