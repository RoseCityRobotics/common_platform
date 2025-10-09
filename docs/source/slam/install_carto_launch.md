# Install Cartographer Launch

This guide explains how to install and configure Cartographer launch files for SLAM.

## Prerequisites

Before installing Cartographer, ensure you have the latest version of the common platform repository and a ROS2 workspace set up.

## Steps

### 1. Navigate to the Repository

Change to the common platform directory:

```bash
cd ~/repos/common_platform
```

### 2. Stash Current Changes

Save any uncommitted changes:

```bash
git stash
```

### 3. Switch to Main Branch

Ensure you're on the main branch to get the latest updates:

```bash
git checkout main
```

### 4. Pull Latest Changes

Get the most recent updates from the repository:

```bash
git pull
```

### 5. Restore Stashed Changes

Apply your previously stashed changes back:

```bash
git stash pop
```

### 6. Copy Launch File

Copy the Cartographer launch file to your ROS2 workspace:

```bash
cp launch/cartographer_simple.launch.py ~/ros2_ws/src/cartographer_ros/cartographer_ros/launch/
```

### 7. Navigate to ROS2 Workspace

Change to your ROS2 workspace directory:

```bash
cd ~/ros2_ws/
```

### 8. Build Cartographer Package

Build the Cartographer ROS package with symlink installation:

```bash
colcon build --packages-select cartographer_ros --symlink-install
```

## Next Steps

After completing these steps, Cartographer should be ready to use. You can now launch SLAM mapping with the configured launch file.
