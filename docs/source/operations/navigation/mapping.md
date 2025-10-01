# Mapping Operations

This guide covers creating, updating, and managing maps for the PARTS Common Robotics Platform.

## Overview

Mapping is the process of creating a representation of the robot's environment using sensor data, primarily from the LiDAR sensor. This map is essential for navigation and localization.

## Prerequisites

- Robot hardware properly assembled and calibrated
- LiDAR sensor functioning correctly
- ROS2 navigation stack installed
- Sufficient battery charge for mapping session

## Mapping Methods

### 1. SLAM (Simultaneous Localization and Mapping)

SLAM allows the robot to build a map while simultaneously tracking its position within that map.

#### Using Cartographer

1. **Launch SLAM**
   ```bash
   ros2 launch common_platform cartographer_2d.launch.py
   ```

2. **Teleop Control**
   ```bash
   # In another terminal
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

3. **Drive the Robot**
   - Use keyboard controls to drive the robot
   - Cover the entire area you want to map
   - Move slowly for better map quality
   - Ensure good LiDAR coverage

4. **Save the Map**
   ```bash
   # Save the map
   ros2 run nav2_map_server map_saver_cli -f my_map
   ```

#### Using Online Async SLAM

1. **Launch Online Async SLAM**
   ```bash
   ros2 launch common_platform online_async_launch.py
   ```

2. **Follow similar teleop procedure**

### 2. Manual Mapping

For areas where autonomous mapping is difficult:

1. **Use RViz for Visualization**
   ```bash
   ros2 launch common_platform view_robot.launch.py
   ```

2. **Manual Control**
   - Use joystick or keyboard control
   - Monitor map building in RViz
   - Adjust speed based on map quality

## Mapping Best Practices

### Environment Preparation

1. **Lighting**
   - Ensure adequate lighting
   - Avoid direct sunlight on LiDAR
   - Minimize shadows and reflections

2. **Obstacles**
   - Remove temporary obstacles
   - Ensure clear pathways
   - Mark permanent obstacles clearly

3. **Area Coverage**
   - Plan mapping route in advance
   - Ensure complete area coverage
   - Overlap paths for better accuracy

### Robot Operation

1. **Speed Control**
   - Move slowly (0.1-0.3 m/s)
   - Avoid sudden direction changes
   - Maintain consistent speed

2. **Path Planning**
   - Use systematic coverage patterns
   - Ensure good LiDAR visibility
   - Avoid tight spaces initially

3. **Quality Monitoring**
   - Monitor map quality in RViz
   - Watch for mapping errors
   - Adjust parameters if needed

## Map Quality Assessment

### Visual Inspection

1. **Check Map Completeness**
   - All areas should be covered
   - No large gaps in walls
   - Obstacles clearly defined

2. **Verify Accuracy**
   - Compare with actual environment
   - Check wall straightness
   - Verify obstacle positions

3. **Look for Artifacts**
   - Ghost walls or obstacles
   - Inconsistent wall thickness
   - Mapping errors or loops

### Quantitative Metrics

1. **Coverage Percentage**
   - Calculate mapped vs. total area
   - Identify unmapped regions
   - Plan additional mapping if needed

2. **Map Resolution**
   - Verify appropriate resolution
   - Check for over/under-sampling
   - Adjust parameters if necessary

## Map Management

### File Organization

```
maps/
├── office_floor1/
│   ├── office_floor1.yaml
│   └── office_floor1.pgm
├── warehouse/
│   ├── warehouse.yaml
│   └── warehouse.pgm
└── test_area/
    ├── test_area.yaml
    └── test_area.pgm
```

### Map Metadata

Each map includes a `.yaml` file with metadata:

```yaml
image: warehouse.pgm
resolution: 0.05
origin: [-10.0, -10.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

### Map Updates

1. **Incremental Updates**
   - Use SLAM for map updates
   - Merge new data with existing map
   - Maintain map consistency

2. **Complete Remapping**
   - Delete old map files
   - Create new map from scratch
   - Update navigation parameters

## Troubleshooting

### Common Mapping Issues

**Poor Map Quality**
- Check LiDAR calibration
- Verify sensor mounting
- Adjust mapping parameters

**Incomplete Coverage**
- Plan better mapping route
- Increase mapping time
- Use systematic coverage patterns

**Mapping Errors**
- Check for sensor obstructions
- Verify robot odometry
- Review SLAM parameters

### Parameter Tuning

1. **Resolution Settings**
   ```yaml
   # In mapping configuration
   resolution: 0.05  # 5cm per pixel
   ```

2. **SLAM Parameters**
   ```yaml
   # Cartographer parameters
   num_laser_scans: 1
   num_multi_echo_laser_scans: 0
   num_subdivisions_per_laser_scan: 1
   ```

3. **Map Server Settings**
   ```yaml
   # Map server configuration
   occupied_thresh: 0.65
   free_thresh: 0.196
   ```

## Advanced Mapping

### Multi-Floor Mapping

1. **Separate Maps**
   - Create individual maps per floor
   - Use elevator/stairs as transition points
   - Maintain separate navigation stacks

2. **Map Switching**
   - Implement map switching logic
   - Update localization parameters
   - Handle coordinate frame changes

### Dynamic Environment Mapping

1. **Obstacle Tracking**
   - Use dynamic obstacle detection
   - Update costmaps in real-time
   - Maintain static map base

2. **Temporary Obstacles**
   - Mark temporary obstacles
   - Remove from map when cleared
   - Update navigation accordingly

## Map Validation

### Testing Procedures

1. **Localization Test**
   - Test robot localization accuracy
   - Verify pose estimation quality
   - Check for localization failures

2. **Navigation Test**
   - Test path planning capabilities
   - Verify obstacle avoidance
   - Check navigation performance

3. **Edge Case Testing**
   - Test in narrow passages
   - Verify performance near walls
   - Check behavior in open areas

---

*For localization procedures, see [Localization](localization.md)*
*For path planning, see [Path Planning](path_planning.md)*
*For troubleshooting, see [Troubleshooting](../troubleshooting/index.md)*
