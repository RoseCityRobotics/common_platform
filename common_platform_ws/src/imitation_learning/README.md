# Imitation Learning ROS2 Package

This ROS2 package provides imitation learning inference capabilities using trained models on Raspberry Pi 5, running inference on CPU using ONNX Runtime.

## Features

- Subscribes to `camera/image_raw` topic for input images
- Runs inference using ONNX Runtime with trained imitation learning model (ONNX format)
- Publishes velocity commands to `cmd_vel` topic at configurable rate (default: 30 Hz)
- Supports ROS2 namespaces via `ROS_NAMESPACE` environment variable
- Configurable model parameters, input dimensions, and velocity limits
- **Performance monitoring**: Tracks inference time, dropped frames, and reports statistics
- **Rate monitoring**: Warns if inference can't keep up with camera frame rate

## Dependencies

- ROS2 (Kilted)
- rclcpp
- sensor_msgs
- geometry_msgs
- std_msgs
- OpenCV4
- **ONNX Runtime C++ library** (see [ONNXRUNTIME_INSTALL.md](ONNXRUNTIME_INSTALL.md) for installation)
- ament_cmake

## Installation

### 1. Install ONNX Runtime

See [ONNXRUNTIME_INSTALL.md](ONNXRUNTIME_INSTALL.md) for detailed instructions on installing ONNX Runtime on Raspberry Pi 5 (Ubuntu 24 Server).

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
  model_path:=/path/to/your/model.onnx \
  input_width:=224 \
  input_height:=224 \
  sequence_length:=10 \
  max_linear_velocity:=0.5 \
  max_angular_velocity:=1.5 \
  publish_rate:=30.0 \
  stats_report_interval:=10.0 \
  max_inference_time_ms:=33.0
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

- `model_path`: Path to the ONNX model file (default: `/home/rcr/repos/common_platform/models/imitation_learning.onnx`)
- `input_width`: Input image width (default: 224)
- `input_height`: Input image height (default: 224)
- `sequence_length`: Number of consecutive frames for temporal model (default: 10)
- `max_linear_velocity`: Maximum linear velocity in m/s (default: 0.5)
- `max_angular_velocity`: Maximum angular velocity in rad/s (default: 1.5)
- `publish_rate`: Publishing rate for cmd_vel in Hz (default: 30.0)
- `stats_report_interval`: Interval for reporting inference statistics in seconds (default: 10.0)
- `max_inference_time_ms`: Maximum expected inference time in milliseconds (default: 33.0, for 30 Hz camera)

## Topics

### Subscribed Topics

- `/camera/image_raw` (sensor_msgs/Image): Input images for inference

### Published Topics

- `/cmd_vel` (geometry_msgs/Twist): Velocity commands with linear.x and angular.z

## Model Compilation

See [MODEL_COMPILATION.md](MODEL_COMPILATION.md) for detailed instructions on converting PyTorch models to ONNX format.

## Usage Example

1. Ensure you have an ONNX model (see MODEL_COMPILATION.md)

2. Start the imitation learning node:
```bash
ros2 launch imitation_learning imitation_learning.launch.py \
  model_path:=/path/to/your/model.onnx
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

## Performance Monitoring

The node includes comprehensive performance monitoring:

- **Inference time tracking**: Measures and tracks min/max/average inference times
- **Dropped frame detection**: Monitors if frames arrive slower than expected
- **Rate monitoring**: Checks if inference rate matches camera rate
- **Periodic statistics**: Reports performance metrics every 10 seconds (configurable)

Example statistics output:
```
=== Inference Performance Stats (last 10 seconds) ===
  Total inferences: 300 (30.00 Hz)
  Avg inference time: 25.50 ms
  Min inference time: 22.10 ms
  Max inference time: 35.20 ms
  Dropped frames: 2
  ✓ Inference rate is sufficient for 30.0 Hz camera
```

If inference is too slow, you'll see warnings:
```
⚠ WARNING: Inference rate (25.00 Hz) is below expected rate (30.00 Hz)
```

## Troubleshooting

### ONNX Runtime Not Found

If the build fails to find ONNX Runtime, see [ONNXRUNTIME_INSTALL.md](ONNXRUNTIME_INSTALL.md) for installation instructions.

### External Data File Error

If you see an error like:
```
Exception during initialization: filesystem error: cannot get file size: No such file or directory [../models/best_model.onnx.data]
```

This means your ONNX model references an external data file (weights stored separately) but the file is missing or has a different name. Solutions:

1. **Re-export the model without external data** (recommended):
   ```python
   # In export_to_onnx.py, ensure external_data=False (default)
   torch.onnx.export(..., external_data=False)
   ```

2. **Copy the external data file** to match the name referenced in the ONNX file:
   - The error message shows which file is expected (e.g., `best_model.onnx.data`)
   - Copy your `.data` file to match that name in the same directory as the model

3. **Use a tool to fix the ONNX file** to point to the correct external data file name

### Inference Too Slow

If inference is too slow to keep up with the camera rate:
1. Check CPU usage and temperature
2. Consider reducing model complexity
3. Try quantizing the model to INT8
4. Reduce `sequence_length` if possible
5. Adjust thread counts in the code (currently 4 intra-op, 2 inter-op)

