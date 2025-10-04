# Mapping Operations

This guide covers creating, updating, and managing maps for the PARTS Common Robotics Platform.

## Overview

Mapping is the process of creating a representation of the robot's environment using sensor data, primarily from the LiDAR sensor. This map is essential for navigation and localization.

## Prerequisites

- Robot hardware properly assembled and calibrated
- LiDAR sensor functioning correctly
- Odometry system working (wheel encoders and/or IMU)
- Sufficient battery charge for mapping session

## Mapping Methods

### 1. SLAM (Simultaneous Localization and Mapping) Using Cartographer

SLAM allows the robot to build a map while simultaneously tracking its position within that map.

1. **Launch SLAM**
   ```bash
      cd ros2_ws/

      # 1. Launch the robot state publisher
      ros2 launch common_platform pub_robot_state.launch.py use_sim_time:=false

      # 2. Run Cartographer node with remapped scan topic
      ros2 run cartographer_ros cartographer_node \
      -configuration_directory . \
      -configuration_basename common_platform.lua \
      --ros-args --remap scan:=/${ROS_NAMESPACE}/scan -r __ns:=${ROS_NAMESPACE}

      # 3. Run occupancy grid node
      ros2 run cartographer_ros cartographer_occupancy_grid_node \
      --ros-args -p resolution:=0.05 -p publish_period_sec:=1.0 \
      -r __ns:=${ROS_NAMESPACE}
   ```

   **Nodes Created:**
   - `robot_state_publisher` - Publishes robot transforms
   - `joint_state_publisher` - Publishes robot joint status
   - `cartographer_node` - Main SLAM processing node
   - `cartographer_occupancy_grid_node` - Occupancy grid publisher

   **Expected Topics:**

   **Subscriptions (cartographer_node):**
   - `/scan` (sensor_msgs/LaserScan) - LiDAR scan data
   - `/tf` (tf2_msgs/TFMessage) - Transform data
   - `/tf_static` (tf2_msgs/TFMessage) - Static transform data
   - `/odom` (nav_msgs/Odometry) - Odometry data

   **Publications (cartographer_node):**
   - `/tf` (tf2_msgs/TFMessage) - Transform from map to odom
   - `/constraint_list` (cartographer_ros_msgs/ConstraintList) - Loop closure constraints
   - `/trajectory_node_list` (cartographer_ros_msgs/TrajectoryNodeList) - Trajectory nodes

   **Publications (cartographer_occupancy_grid_node):**
   - `/map` (nav_msgs/OccupancyGrid) - Final occupancy grid map
   - `/submap_list` (cartographer_ros_msgs/SubmapList) - Submap information

   **Verify Topics:**
   ```bash
   # Check active topics
   ros2 topic list

   # Monitor map updates
   ros2 topic echo /map

   # Check transform tree
   ros2 run tf2_tools view_frames
   ```

2. **Teleop Control**

   See [Keyboard Teleoperation Setup](../../daily_operations/KeyboardTeleop.md) for robot control setup.

3. **Drive the Robot**
   - Use keyboard controls to drive the robot
   - Cover the entire area you want to map
   - Move slowly for better map quality
   - Ensure good LiDAR coverage

4. **Save the Map**
   ```bash
   # Save the map using Cartographer's built-in functionality
   ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
   ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: 'my_map.pbstream'}"

   # Convert to standard map format (optional)
   ros2 run cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename my_map.pbstream -map_filename my_map
   ```

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

#### Cartographer Parameters

Cartographer uses Lua configuration files located in `config/cartographer/`. The most important parameters for tuning:

1. **Trajectory Builder Parameters**
   ```lua
   -- In cartographer_2d.lua
   TRAJECTORY_BUILDER_2D = {
     min_range = 0.3,                    -- Minimum LiDAR range (m)
     max_range = 8.0,                    -- Maximum LiDAR range (m)
     min_z = 0.1,                        -- Minimum height for points
     max_z = 2.0,                        -- Maximum height for points
     missing_data_ray_length = 1.0,      -- Length for missing data rays
     num_accumulated_range_data = 1,     -- Number of scans to accumulate
     voxel_filter_size = 0.025,          -- Voxel filter size (m)
     adaptive_voxel_filter_max_length = 0.5,
     adaptive_voxel_filter_min_num_points = 200,
     adaptive_voxel_filter_max_range = 50.0,
     use_intensities = false,            -- Use LiDAR intensity data
   }
   ```

   **Why Voxels in 2D SLAM?**

   Even though Cartographer creates 2D maps, it processes 3D point cloud data from the LiDAR sensor. The voxel filtering serves several purposes:

   - **3D Point Cloud Processing**: LiDAR sensors provide 3D points (x, y, z coordinates)
   - **Height Filtering**: `min_z` and `max_z` parameters filter out points outside the robot's height range
   - **Noise Reduction**: Voxel filtering reduces noise and duplicate points in 3D space
   - **Computational Efficiency**: Downsampling 3D points before projecting to 2D map
   - **Data Quality**: Removes outliers and inconsistent measurements

   The final 2D map is created by projecting the filtered 3D points onto the ground plane.

2. **Pose Graph Parameters**
   ```lua
   POSE_GRAPH = {
     optimize_every_n_nodes = 90,        -- Optimize every N nodes
     constraint_builder = {
       sampling_ratio = 0.03,            -- Loop closure sampling ratio
       max_constraint_distance = 15.0,   -- Max distance for constraints
       min_score = 0.55,                 -- Minimum score for loop closure
       global_localization_min_score = 0.6,
     },
   }
   ```

3. **Launch File Parameters**
   ```bash
   # Adjustable via launch arguments
   ros2 launch common_platform cartographer_2d.launch.py \
     resolution:=0.05 \
     publish_period_sec:=1.0
   ```


#### Tuning Procedure

1. **Start with Default Parameters**
   ```bash
   # Use default configuration
   ros2 launch common_platform cartographer_2d.launch.py
   ```

2. **Monitor Mapping Quality**
   - Watch for loop closure detection
   - Check for consistent wall thickness
   - Verify accurate pose estimation

#### Detecting Loop Closures

**Visual Indicators in RViz:**
- **Pose Graph Visualization**: Look for constraint lines connecting distant poses
- **Map Alignment**: Watch for sudden map corrections when returning to previously visited areas
- **Trajectory Smoothing**: Notice when the robot's path becomes more consistent

**Console Output Indicators:**
```bash
# Look for these messages in the terminal:
[INFO] [cartographer_node]: Adding loop closure constraint
[INFO] [cartographer_node]: Optimization completed
[INFO] [cartographer_node]: Pose graph optimization took X ms
```

**RViz Display Settings:**
```bash
# Enable these displays in RViz:
- Map (occupancy grid)
- Pose Graph (constraints and nodes)
- Trajectory (robot path)
- LaserScan (current scan data)
```

**Monitoring Commands:**
```bash
# Monitor Cartographer topics
ros2 topic echo /constraint_list
ros2 topic echo /trajectory_node_list

# Check for optimization events
ros2 topic hz /constraint_list
```

3. **Adjust Based on Environment**

   **For Large Open Spaces:**
   ```lua
   -- Increase loop closure search distance
   max_constraint_distance = 25.0
   loop_search_maximum_distance = 5.0
   ```

   **For Cluttered Environments:**
   ```lua
   -- Reduce minimum range, increase sampling
   min_range = 0.2
   sampling_ratio = 0.05
   ```

   **For High-Speed Mapping:**
   ```lua
   -- Reduce optimization frequency
   optimize_every_n_nodes = 120
   num_accumulated_range_data = 2
   ```

4. **Fine-tune Loop Closure**
   ```lua
   -- Adjust loop closure sensitivity
   min_score = 0.5              # Lower = more sensitive
   global_localization_min_score = 0.55
   ```

5. **Optimize Performance**
   ```lua
   -- Balance accuracy vs. performance
   optimize_every_n_nodes = 60   # Lower = more frequent optimization
   num_accumulated_range_data = 1  # Higher = more data per scan
   ```

#### Common Tuning Scenarios

**Poor Loop Closure:**
- Increase `max_constraint_distance`
- Decrease `min_score`
- Increase `sampling_ratio`

**Inconsistent Wall Thickness:**
- Adjust `voxel_filter_size`
- Tune `adaptive_voxel_filter_min_num_points`
- Check `min_range` and `max_range`

**Slow Performance:**
- Increase `optimize_every_n_nodes`
- Reduce `num_accumulated_range_data`
- Decrease `sampling_ratio`

**Memory Issues:**
- Increase `optimize_every_n_nodes`
- Reduce `max_constraint_distance`
- Lower `adaptive_voxel_filter_max_range`

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
