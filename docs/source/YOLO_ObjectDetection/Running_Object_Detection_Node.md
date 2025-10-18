# Running the Object Detection Node

This guide explains how to run the object detection node from the `obj_detect` package, which provides real-time object detection capabilities using the HailoRT API and YOLOv11n model.

## Overview

The object detection node (`object_detector`) is designed to:
- Subscribe to camera image data from the `/camera/image_raw` topic
- Perform real-time object detection using a YOLOv11n model optimized for Hailo hardware
- Publish detection results to the `/detect` topic with bounding boxes and class information
- Detect two specific classes: Purple ball and Green ball

## Prerequisites

Before running the object detection node, ensure you have:

1. **ROS2 Environment**: The workspace must be built and sourced
2. **HailoRT API**: Required for running the YOLOv11n model on Hailo hardware
3. **Model File**: The `yolov11n_2cls.hef` model file should be available
4. **Camera Data**: A camera node publishing to `/camera/image_raw` topic (See Camera Operations)[../camera/camera_operations.md]

## Building the Package

First, build the `obj_detect` package (you only need to do this once):

```bash
cd ~/repos/common_platform/common_platform_ws
colcon build --packages-select obj_detect --symlink-install
source install/setup.bash
```

## Running the Object Detection Node

### Method 1: Using Launch File (Recommended)

The easiest way to run the object detection node is using the provided launch file:

```bash
ros2 launch obj_detect object_detector.launch.py
```

This will start the node with default parameters:
- Model path: `/home/rcr/repos/common_platform/models/yolov11n_2cls.hef`
- Confidence threshold: 0.5
- Input dimensions: 640x640
- Log directory: `/tmp`

### Method 2: Custom Parameters

You can override default parameters when launching:

```bash
ros2 launch obj_detect object_detector.launch.py \
  model_path:=/path/to/your/model.hef \
  confidence_threshold:=0.6 \
  input_width:=640 \
  input_height:=640 \
  hailort_log_path:=/path/to/logs
```

### Method 3: Direct Node Execution

To run the node directly without the launch file:

```bash
ros2 run obj_detect object_detector
```

Note: When running directly, you'll need to set parameters manually or use the launch file for proper configuration.

## Configuration Parameters

| Parameter | Default Value | Description |
|-----------|---------------|-------------|
| `model_path` | `/home/rcr/repos/common_platform/models/yolov11n_2cls.hef` | Path to the HEF model file |
| `confidence_threshold` | 0.5 | Minimum confidence for detections (0.0-1.0) |
| `input_width` | 640 | Input image width in pixels |
| `input_height` | 640 | Input image height in pixels |
| `hailort_log_path` | `/tmp` | Directory for HailoRT log files |

## Topics

### Subscribed Topics
- **`/camera/image_raw`** (`sensor_msgs/Image`): Input camera images for object detection

### Published Topics
- **`/detect`** (`obj_detect/DetectionArray`): Detection results containing:
  - Bounding box coordinates (x, y, width, height)
  - Class names ("Purple ball" or "Green ball")
  - Confidence scores

## Message Format

The detection results are published using custom message types:

**Detection.msg:**
```
string class_name    # "Purple ball" or "Green ball"
float32 confidence   # Detection confidence (0.0-1.0)
float32 x           # Bounding box center x-coordinate
float32 y           # Bounding box center y-coordinate
float32 width       # Bounding box width
float32 height      # Bounding box height
```

**DetectionArray.msg:**
```
std_msgs/Header header
Detection[] detections
```

## Verification and Monitoring

### Check Node Status
```bash
ros2 node list
ros2 node info /rcr0nn/object_detector
```

### Monitor Detection Results
```bash
# View detection messages
ros2 topic echo /rcr0nn/detect

# Check topic information
ros2 topic info /rcr0nn/detect
ros2 topic hz /rcr0nn/detect
```

### Verify Camera Input
```bash
# Check if camera is publishing
ros2 topic list | grep camera
ros2 topic hz /rcr0nn/camera/image_raw
```

## Troubleshooting

### Common Issues

1. **Model File Not Found**
   - Ensure the model file exists at the specified path
   - Check file permissions

2. **No Camera Data**
   - Verify camera node is running and publishing to `/camera/image_raw`
   - Check camera topic with `ros2 topic list`

3. **HailoRT Errors**
   - Check HailoRT installation and hardware connection
   - Review log files in the specified `hailort_log_path`

4. **No Detections**
   - Lower the confidence threshold
   - Ensure objects are within the camera's field of view
   - Check lighting conditions

### Logging

HailoRT logs are written to the specified directory (default: `/tmp`). The log file `hailort.log` contains detailed information about the inference process.

To use a custom log directory:
```bash
ros2 launch obj_detect object_detector.launch.py hailort_log_path:=/your/custom/directory
```

## Integration with Other Nodes

The object detection node is designed to work seamlessly with other ROS2 nodes:

- **Camera nodes**: Publishes to `/camera/image_raw`
- **Navigation nodes**: Can subscribe to `/detect` for obstacle avoidance
- **Visualization nodes**: Can display bounding boxes on camera feed
- **Control nodes**: Can use detection data for robot control decisions

## Performance Considerations

- The node processes images at the camera's publishing rate
- Higher confidence thresholds reduce false positives but may miss valid detections
- Input image dimensions affect processing speed and accuracy
- HailoRT hardware acceleration provides real-time performance

This completes the setup for running the object detection node. The system is now ready to detect Purple and Green balls in real-time camera feeds.
