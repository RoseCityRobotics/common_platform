---
type: module
slug: save-map
title: Save a Cartographer SLAM map
description: Finish the trajectory and save the map created during SLAM.
tags: [slam, cartographer, mapping]
device: pi
---

# Save a Cartographer SLAM map

{{pi}} **On the Raspberry Pi**

After you've finished mapping your environment, save the map for future use.

## Finish the trajectory

```bash
ros2 service call ${ROS_NAMESPACE}/finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
```

This tells Cartographer to finish recording the current trajectory.

## Write the state to a file

```bash
ros2 service call ${ROS_NAMESPACE}/write_state cartographer_ros_msgs/srv/WriteState "{filename: 'my_map.pbstream'}"
```

This saves the map in Cartographer's `.pbstream` format.

## Convert to standard map format (optional)

```bash
ros2 run cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename my_map.pbstream -map_filename my_map
```

:::{note}
This creates standard `.pgm` and `.yaml` files that can be used with other ROS 2 navigation tools.
:::

:::{tip}
Name your map files descriptively (e.g., `lab_floor1.pbstream` or `office_2024.pbstream`) to keep track of different environments.
:::

