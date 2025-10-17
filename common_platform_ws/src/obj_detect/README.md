# Object Detection ROS2 Package

This ROS2 package provides object detection capabilities using the HailoRT API and YOLOv11n model.

## Features

- Subscribes to `image_raw` topic for input images
- Runs inference using HailoRT API with YOLOv11n_2cls.hef model
- Publishes detection results to `detect` topic
- Custom message interface for detection results
- Configurable parameters for confidence threshold, NMS threshold, and input dimensions

## Message Interface

### Detection.msg
```
string class_name
float64 confidence
geometry_msgs/Point top_left
geometry_msgs/Point bottom_right
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
- cv_bridge
- OpenCV
- HailoRT API
- rosidl_default_generators

## Building

```bash
cd /home/rcr/repos/common_platform/common_platform_ws
colcon build --packages-select obj_detect
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
  nms_threshold:=0.5
```

### Running the node directly
```bash
ros2 run obj_detect object_detector
```

## Parameters

- `model_path`: Path to the HEF model file (default: `/home/rcr/repos/common_platform/models/yolov11n_2cls.hef`)
- `confidence_threshold`: Confidence threshold for detections (default: 0.5)
- `nms_threshold`: NMS threshold for post-processing (default: 0.4)
- `input_width`: Input image width (default: 640)
- `input_height`: Input image height (default: 640)

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

## Notes

- The current implementation includes a framework for HailoRT integration but may need specific adjustments based on your HEF model format
- Class names are currently set to generic values ("class1", "class2") - update these in the code to match your model's classes
- The inference implementation is a template that needs to be completed with actual HailoRT API calls for your specific model
