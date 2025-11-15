# SLAM Mapping

This procedure guides you through creating a map of your environment using SLAM (Simultaneous Localization and Mapping) with Cartographer.

## Overview

You'll complete the following steps:
1. Set up Cartographer launch files (one-time setup)
2. Start robot state publisher and Cartographer
3. Drive the robot to map the environment
4. Save the map

## Prerequisites

Before starting SLAM mapping, ensure:
- [Raspberry Pi is set up](raspberry-pi-setup.md)
- [Firmware is flashed and teleoperation is working](firmware-and-teleop.md)
- LiDAR is powered and spinning
- You have a clear area to map

---

## Step 1: Set Up Cartographer (One-Time)

```{include} ../modules/copy-cartographer-launch.md
:start-after: "# Set up Cartographer launch files"
```

---

## Step 2: Launch SLAM Nodes

```{include} ../modules/start-robot-state-and-cartographer.md
:start-after: "# Start robot state publisher and Cartographer SLAM"
```

---

## Step 3: Visualize in RViz

```{include} ../modules/view-robot-slam.md
:start-after: "# Visualize SLAM with RViz"
```

---

## Step 4: Drive the Robot

Use keyboard teleoperation (see [Firmware and Teleoperation](firmware-and-teleop.md)) to drive the robot around the environment.

**Mapping Best Practices:**

- Move slowly (0.1-0.3 m/s)
- Cover the entire area systematically
- Avoid sudden direction changes
- Maintain consistent speed
- Ensure good LiDAR visibility
- Watch the map quality in RViz

:::{tip}
Plan your mapping route in advance. Use systematic coverage patterns like parallel lines or spirals for complete coverage.
:::

---

## Step 5: Save the Map

```{include} ../modules/save-map.md
:start-after: "# Save a Cartographer SLAM map"
```

---

## Map Quality Assessment

### Visual Inspection

Check your map for:
- **Completeness**: All areas covered, no large gaps in walls
- **Accuracy**: Obstacles clearly defined, walls straight
- **Consistency**: No duplicate features or misalignments

### Troubleshooting Poor Maps

If your map quality is poor:

1. **Slow down** - Move more slowly during mapping
2. **Better lighting** - Ensure adequate lighting, avoid direct sunlight on LiDAR
3. **Check LiDAR** - Verify LiDAR is spinning and publishing data
4. **Re-map problem areas** - Drive through poorly mapped sections again
5. **Adjust parameters** - See [Cartographer tuning](../slam/tuning.md)

---

## Next Steps

After creating your map:
- [Set up localization](slam-localization.md) to use the map for navigation
- [Configure navigation](navigation.md) for autonomous driving
- [Refine your map](../slam/map-editing.md) if needed

