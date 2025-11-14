# Imitation Learning ROS2 Package

This ROS2 package provides imitation learning inference capabilities using trained models on the Raspberry Pi 5 AI Hat (Hailo-8L AI accelerator).

## Features

- Subscribes to `camera/image_raw` topic for input images
- Runs inference using HailoRT API with trained imitation learning model (HEF format)
- Publishes velocity commands to `cmd_vel` topic at configurable rate (default: 30 Hz)
- Supports ROS2 namespaces via `ROS_NAMESPACE` environment variable
- Configurable model parameters, input dimensions, and velocity limits

## Dependencies

- ROS2 (Kilted)
- rclcpp
- sensor_msgs
- geometry_msgs
- std_msgs
- OpenCV4
- HailoRT API
- ament_cmake

## Building

```bash
cd /home/rcr/repos/common_platform/common_platform_ws
colcon build --packages-select imitation_learning --symlink-install
source install/setup.bash
```

## Running

### Using launch file (recommended)

```bash
ros2 launch imitation_learning imitation_learning.launch.py
```

### With custom parameters

```bash
ros2 launch imitation_learning imitation_learning.launch.py \
  model_path:=/path/to/your/model.hef \
  input_width:=224 \
  input_height:=224 \
  sequence_length:=10 \
  max_linear_velocity:=0.5 \
  max_angular_velocity:=1.5 \
  publish_rate:=30.0
```

### With namespace

```bash
ROS_NAMESPACE=/robot1 ros2 launch imitation_learning imitation_learning.launch.py
```

### Running the node directly

```bash
ros2 run imitation_learning imitation_learning_node
```

## Parameters

- `model_path`: Path to the HEF model file (default: `/home/rcr/repos/common_platform/models/imitation_learning.hef`)
- `input_width`: Input image width (default: 224)
- `input_height`: Input image height (default: 224)
- `sequence_length`: Number of consecutive frames for temporal model (default: 10)
- `max_linear_velocity`: Maximum linear velocity in m/s (default: 0.5)
- `max_angular_velocity`: Maximum angular velocity in rad/s (default: 1.5)
- `publish_rate`: Publishing rate for cmd_vel in Hz (default: 30.0)
- `hailort_log_path`: Directory for HailoRT log file - `hailort.log` will be created here (default: `/tmp`)

## Topics

### Subscribed Topics

- `/camera/image_raw` (sensor_msgs/Image): Input images for inference

### Published Topics

- `/cmd_vel` (geometry_msgs/Twist): Velocity commands with linear.x and angular.z

## Model Compilation

See [MODEL_COMPILATION.md](MODEL_COMPILATION.md) for detailed instructions on converting PyTorch models to HEF format for the Raspberry Pi 5 AI Hat.

## Usage Example

1. Ensure you have a compiled HEF model (see MODEL_COMPILATION.md)

2. Start the imitation learning node:
```bash
ros2 launch imitation_learning imitation_learning.launch.py \
  model_path:=/path/to/your/model.hef
```

3. In another terminal, check the topics:
```bash
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic hz /cmd_vel
```

4. The node will automatically:
   - Subscribe to camera images
   - Run inference when enough frames are available
   - Publish cmd_vel commands at the specified rate (default 30 Hz)

## Architecture

The node maintains an image buffer to support temporal models that require sequences of images. When enough images are collected (based on `sequence_length`), inference is run and the predicted velocities are stored. A timer publishes these predictions at the configured rate (default 30 Hz) to ensure smooth control.

## Logging

HailoRT logs are written to a configurable directory (default: `/tmp`). The log file `hailort.log` will be created in this directory. The log directory is printed at node startup.

To use a custom log directory:
```bash
ros2 launch imitation_learning imitation_learning.launch.py \
  hailort_log_path:=/your/custom/directory
```

