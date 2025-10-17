# Object Detection ROS2 Package

This ROS2 package provides object detection capabilities using the HailoRT API and YOLOv11n model.

## Features

- Subscribes to `image_raw` topic for input images
- Runs inference using HailoRT API with YOLOv11n_2cls.hef model
- Publishes detection results to `detect` topic
- Custom message interface for detection results
- Detects Purple ball and Green ball objects
- Configurable parameters for confidence threshold, input dimensions, and log path

## Message Interface

### Detection.msg
```
string class_name
float32 confidence
float32 x
float32 y
float32 width
float32 height
```

### DetectionArray.msg
```
std_msgs/Header header
Detection[] detections
```

## Dependencies

- ROS2 (Kilted)
- rclcpp
- sensor_msgs
- geometry_msgs
- std_msgs
- OpenCV4
- HailoRT API
- rosidl_default_generators

## Building

```bash
cd /home/rcr/repos/common_platform/common_platform_ws
colcon build --packages-select obj_detect --symlink-install
source install/setup.bash
```

## Running

### Using launch file (recommended)
```bash
ros2 launch obj_detect object_detector.launch.py
```

### With custom parameters
```bash
ros2 launch obj_detect object_detector.launch.py \
  model_path:=/path/to/your/model.hef \
  confidence_threshold:=0.6 \
  hailort_log_path:=/path/to/hailort.log
```

### Running the node directly
```bash
ros2 run obj_detect object_detector
```

## Parameters

- `model_path`: Path to the HEF model file (default: `/home/rcr/repos/common_platform/models/yolov11n_2cls.hef`)
- `confidence_threshold`: Confidence threshold for detections (default: 0.5)
- `input_width`: Input image width (default: 640)
- `input_height`: Input image height (default: 640)
- `hailort_log_path`: Path for HailoRT log file (default: `/tmp/hailort.log`)

## Topics

### Subscribed Topics
- `/image_raw` (sensor_msgs/Image): Input images for detection

### Published Topics
- `/detect` (obj_detect/DetectionArray): Detection results with bounding boxes and class names

## Usage Example

1. Start the object detector:
```bash
ros2 launch obj_detect object_detector.launch.py
```

2. In another terminal, check the topics:
```bash
ros2 topic list
ros2 topic echo /detect
```

3. To visualize detections, you can use RViz2 or create a visualization node.

## Class Names

The model detects 2 classes:
- **Purple ball** (class 0)
- **Green ball** (class 1)

## Logging

HailoRT logs are written to a configurable location (default: `/tmp/hailort.log`). This prevents log files from cluttering the repository. The log path is printed at node startup.

To use a custom log location:
```bash
ros2 launch obj_detect object_detector.launch.py hailort_log_path:=/your/custom/path/hailort.log
```