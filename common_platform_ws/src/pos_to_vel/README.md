# Odometry Calibration Node

This package provides a ROS2 node for calibrating robot odometry using discrete keyboard-controlled movements.

## Features

- **Discrete Actions**: Arrow keys trigger discrete movements (forward, turn+forward, etc.)
- **Action Calibration**: Adjust movement parameters in real-time using keyboard keys
- **Odometry Comparison**: Displays how well actual odometry readings match expected values after each action
- **Statistics Tracking**: Tracks position and orientation errors across multiple actions

## Usage

### Launch the Node

```bash
cd ~/code/common_platform/common_platform_ws
source install/setup.bash
ros2 launch pos_to_vel odom_calibration.launch.py
```

### With Namespace

```bash
ROS_NAME=robot1 ros2 launch pos_to_vel odom_calibration.launch.py
```

### With Custom Parameters

```bash
ros2 launch pos_to_vel odom_calibration.launch.py \
  forward_distance:=0.05 \
  turn_angle:=1.5708 \
  linear_speed:=0.1 \
  angular_speed:=0.5
```

## Keyboard Controls

### Arrow Keys (Actions)
- **↑ (Up)**: Move forward by the set distance (default: 5 cm)
- **← (Left)**: Turn left 90° then move forward
- **→ (Right)**: Turn right 90° then move forward
- **↓ (Down)**: Turn 180° then move forward

### Calibration Adjustments
- **w/s**: Increase/decrease forward distance by 1 mm
- **a/d**: Increase/decrease turn angle by ~0.57°
- **q/e**: Increase/decrease linear speed by 1 cm/s
- **z/x**: Increase/decrease angular speed by 0.05 rad/s

### Other Controls
- **r**: Reset statistics
- **p**: Print current calibration values
- **h**: Show help/instructions
- **Ctrl+C**: Exit

## How It Works

1. When you press an arrow key, the node:
   - Records the current odometry position and orientation
   - Calculates the expected final position/orientation based on the action
   - Generates a sequence of `cmd_vel` commands to execute the action
   - Publishes commands to move the robot

2. After the action completes:
   - The robot stops
   - The node waits 0.5 seconds for odometry to settle
   - It compares actual vs expected position/orientation
   - Displays calibration results showing errors

3. You can adjust parameters in real-time:
   - Adjustments are applied to the next action
   - Use the adjustment keys to fine-tune movements
   - View current values with 'p' key

## Topics

### Subscribed
- `/scan` (sensor_msgs/LaserScan): LiDAR scan data
- `/odom` (nav_msgs/Odometry): Odometry data

### Published
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands for robot movement

## Parameters

- `forward_distance` (default: 0.05 m): Distance to move forward in each action
- `turn_angle` (default: 1.5708 rad = 90°): Angle to turn for left/right actions
- `linear_speed` (default: 0.1 m/s): Speed for forward movement
- `angular_speed` (default: 0.5 rad/s): Speed for turning
- `scan_topic` (default: 'scan'): LiDAR scan topic name
- `odom_topic` (default: 'odom'): Odometry topic name
- `cmd_vel_topic` (default: 'cmd_vel'): Command velocity topic name
- `namespace` (default: ''): ROS namespace (can also use ROS_NAME env var)

## Calibration Process

1. Start the node and ensure odometry is publishing
2. Use arrow keys to execute discrete movements
3. Observe the calibration results after each action
4. If errors are consistent, use adjustment keys to fine-tune:
   - If robot moves too far/not far enough: adjust forward distance (w/s)
   - If turns are off: adjust turn angle (a/d)
   - If movements are too fast/slow: adjust speeds (q/e, z/x)
5. Repeat actions and check if errors decrease
6. Use 'r' to reset statistics and start fresh measurements

## Notes

- The node requires a terminal with proper keyboard input support
- Arrow keys send escape sequences that are detected automatically
- Each action completes fully before the next can be executed
- The robot stops after each discrete movement
- Calibration adjustments are cumulative and persist until reset

