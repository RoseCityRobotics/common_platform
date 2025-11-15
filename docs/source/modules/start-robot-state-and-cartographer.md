---
type: module
slug: start-robot-state-and-cartographer
title: Start robot state publisher and Cartographer SLAM
description: Launch the robot state publisher and Cartographer nodes for SLAM mapping.
tags: [slam, cartographer, mapping]
device: pi
---

# Start robot state publisher and Cartographer SLAM

{{pi}} **On the Raspberry Pi**

## Launch robot state publisher

Open a new terminal window {{terminal}}

```bash
cd ~/repos/common_platform/common_platform_ws/
source install/setup.bash
ros2 launch common_platform pub_robot_state.launch.py use_sim_time:=false
```

This publishes the robot's URDF model and joint states.

## Launch Cartographer SLAM node

Open a new terminal window {{terminal}}

```bash
cd ~/ros2_ws/
ros2 launch cartographer_ros cartographer_simple.launch.py
```

This starts the main SLAM processing node.

## Launch Cartographer occupancy grid node

Open a new terminal window {{terminal}}

```bash
cd ~/ros2_ws/
ros2 run cartographer_ros cartographer_occupancy_grid_node --ros-args -p resolution:=0.05 -p publish_period_sec:=1.0 -r __ns:=${ROS_NAMESPACE}
```

This node publishes the occupancy grid map that can be visualized in RViz.

## Nodes created

- **`robot_state_publisher`** - Publishes robot transforms
- **`joint_state_publisher`** - Publishes robot joint status
- **`cartographer_node`** - Main SLAM processing node
- **`cartographer_occupancy_grid_node`** - Occupancy grid publisher

## Expected topics

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

## Verify topics

```bash
# Check active topics
ros2 topic list

# Monitor map updates
ros2 topic echo /map

# Check transform tree
ros2 run tf2_tools view_frames
```

