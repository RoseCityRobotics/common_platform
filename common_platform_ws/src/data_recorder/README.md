# Data Recorder Package

A ROS2 package for recording synchronized camera images and motor velocity data during teleoperation for neural network training.

## Features

- **Synchronized Data Capture**: Records camera images and motor velocities at 30 Hz with precise timestamps
- **Multiple Output Formats**: Supports HDF5, NumPy, and TFRecord formats for machine learning
- **Real-time Control**: Start/stop recording via ROS2 topics or launch parameters
- **Data Validation**: Built-in validation to ensure data integrity
- **Flexible Configuration**: Configurable topics, recording rates, and output directories

## Quick Start

### 1. Build the Package

```bash
cd /home/rcr/repos/common_platform/common_platform_ws
colcon build --packages-select data_recorder
source install/setup.bash
```

### 2. Launch the Data Recorder

```bash
# Basic launch (manual start/stop) - uses libcamera by default
ros2 launch data_recorder data_recorder.launch.py

# Auto-start recording
ros2 launch data_recorder data_recorder.launch.py auto_start:=true

# Custom output directory
ros2 launch data_recorder data_recorder.launch.py output_dir:=/path/to/your/data

# Use v4l2 camera instead of libcamera
ros2 launch data_recorder data_recorder.launch.py camera_type:=v4l2

# Launch with camera and data recorder together (recommended)
ros2 launch data_recorder data_recorder_with_camera.launch.py

# Launch with libcamera (Pi HQ Camera)
ros2 launch data_recorder data_recorder_with_camera.launch.py camera_type:=libcamera

# Launch with v4l2 camera
ros2 launch data_recorder data_recorder_with_camera.launch.py camera_type:=v4l2

# Launch with namespace support (for multi-robot setups)
ROS_NAMESPACE=robot1 ros2 launch data_recorder data_recorder_with_camera.launch.py
```

### 3. Control Recording

```bash
# Start recording
python3 /path/to/data_recorder/scripts/record_control.py start

# Stop recording
python3 /path/to/data_recorder/scripts/record_control.py stop

# Check status
python3 /path/to/data_recorder/scripts/record_control.py status
```

### 4. Process Data for Training

```bash
# Process a recording session
python3 /path/to/data_recorder/scripts/process_training_data.py /path/to/session_directory

# Validate data only
python3 /path/to/data_recorder/scripts/process_training_data.py /path/to/session_directory --validate-only

# Process with custom output format
python3 /path/to/data_recorder/scripts/process_training_data.py /path/to/session_directory --output-format hdf5
```

## Data Format

### Recorded Data Structure

Each recording session creates a timestamped directory with:

```
session_YYYYMMDD_HHMMSS/
├── images/
│   ├── 00000000.jpg
│   ├── 00000001.jpg
│   └── ...
└── metadata/
    └── data_log.csv
```

### CSV Data Format

The `data_log.csv` contains:

| Column | Description | Units |
|--------|-------------|-------|
| timestamp | Microsecond timestamp | μs |
| image_file | Image filename | - |
| linear_vel | Linear velocity | m/s |
| angular_vel | Angular velocity | rad/s |
| left_wheel_vel | Left wheel velocity | rad/s |
| right_wheel_vel | Right wheel velocity | rad/s |

### Processed Data Formats

After processing, you get:

- **HDF5**: `training_data.h5` with 'images' and 'labels' datasets
- **NumPy**: `images.npy` and `labels.npy` arrays
- **TFRecord**: Conversion script for TensorFlow training
- **Statistics**: `statistics.json` with dataset statistics

## Configuration

### Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `output_dir` | `/home/rcr/teleop_data` | Output directory for recordings |
| `camera_topic` | `/camera/image_raw` | Camera topic to record |
| `cmd_vel_topic` | `/cmd_vel` | Command velocity topic |
| `record_rate` | `30.0` | Recording rate in Hz |
| `auto_start` | `false` | Auto-start recording on launch |
| `use_sim_time` | `false` | Use simulation time |
| `camera_type` | `libcamera` | Camera type: `libcamera` or `v4l2` |
| `camera_id` | `0` | Camera ID for libcamera |
| `pixel_format` | `""` | Pixel format (empty for default) |

### Namespace Support

The data recorder supports ROS2 namespaces for multi-robot setups:

- **Environment Variable**: Set `ROS_NAMESPACE` to enable namespace support
- **Topic Namespacing**: Topics are automatically prefixed with the namespace
- **Node Namespacing**: Nodes are placed in the specified namespace

**Example:**
```bash
# Single robot (no namespace)
ros2 launch data_recorder data_recorder_with_camera.launch.py

# Multi-robot setup
ROS_NAMESPACE=robot1 ros2 launch data_recorder data_recorder_with_camera.launch.py
ROS_NAMESPACE=robot2 ros2 launch data_recorder data_recorder_with_camera.launch.py
```

**With namespace `robot1`, topics become:**
- `/robot1/camera/image_raw`
- `/robot1/cmd_vel`
- `/robot1/data_recorder/start_stop`

### Robot Parameters

The recorder uses these robot parameters (from your config):

- **Wheel Separation**: 0.297 meters
- **Wheel Radius**: 0.033 meters

These are used to convert linear/angular velocities to individual wheel velocities.

### Camera Support

The data recorder supports both camera types:

#### libcamera (Recommended for Pi HQ Camera)
- **Default**: Uses libcamera for better image quality and control
- **Compatible**: Pi HQ Camera (IMX477), Pi Camera v2/v3
- **Benefits**: Better image quality, hardware ISP, automatic exposure control
- **Usage**: `camera_type:=libcamera`

#### v4l2 (Legacy support)
- **Compatible**: USB cameras, older Pi cameras
- **Usage**: `camera_type:=v4l2`

## Topics

### Subscribed Topics

- `/camera/image_raw` (sensor_msgs/Image): Camera images
- `/cmd_vel` (geometry_msgs/Twist): Motor commands
- `/data_recorder/start_stop` (std_msgs/Bool): Recording control

### Published Topics

- `/data_recorder/status` (std_msgs/Bool): Recording status

## Usage Examples

### Basic Teleoperation Recording

1. Start your robot system:
```bash
ros2 launch common_platform launch_robot.launch.py
```

2. Start the data recorder:
```bash
ros2 launch data_recorder data_recorder.launch.py
```

3. Start teleoperation (joystick, keyboard, etc.):
```bash
ros2 launch common_platform joystick.launch.py
```

4. Control recording:
```bash
# Start recording
python3 /path/to/record_control.py start

# Drive your robot around...

# Stop recording
python3 /path/to/record_control.py stop
```

### Batch Processing Multiple Sessions

```bash
# Process all sessions in a directory
for session in /path/to/data/session_*; do
  echo "Processing $session"
  python3 /path/to/process_training_data.py "$session"
done
```

### Custom Training Pipeline

```python
import h5py
import numpy as np

# Load processed data
with h5py.File('training_data.h5', 'r') as f:
    images = f['images'][:]
    labels = f['labels'][:]

# Split into train/validation
split_idx = int(0.8 * len(images))
train_images = images[:split_idx]
train_labels = labels[:split_idx]
val_images = images[split_idx:]
val_labels = labels[split_idx:]

# Your neural network training code here...
```

## Troubleshooting

### Common Issues

1. **No images being recorded**: Check that the camera topic is publishing
2. **No velocity data**: Verify the cmd_vel topic is active
3. **Permission errors**: Ensure write permissions to output directory
4. **High CPU usage**: Reduce recording rate or image resolution

### Debug Commands

```bash
# Check if topics are publishing
ros2 topic list
ros2 topic hz /camera/image_raw
ros2 topic hz /cmd_vel

# Monitor recording status
ros2 topic echo /data_recorder/status

# Check node status
ros2 node list
ros2 node info /data_recorder
```

## Performance Considerations

- **Storage**: Each session can generate 100MB+ per minute (640x480 images at 30Hz)
- **CPU**: Image processing and file I/O can be CPU intensive
- **Memory**: Large datasets may require significant RAM for processing

## Integration with Your Robot

The data recorder is designed to work with your existing robot setup:

- Uses your camera topic: `/camera/image_raw`
- Records direct motor commands: `/cmd_vel`
- Compatible with your joystick teleoperation
- Respects your robot's physical parameters

## YOLO Training with Label Studio

The data recorder is perfect for YOLO object detection training! Images are already saved in JPG format, which is exactly what Label Studio expects.

### Complete YOLO Workflow:

1. **Record Data**:
   ```bash
   ros2 launch data_recorder data_recorder_with_camera.launch.py camera_type:=libcamera
   python3 install/data_recorder/lib/data_recorder/record_control.py start
   # Drive around and collect data
   python3 install/data_recorder/lib/data_recorder/record_control.py stop
   ```

2. **Prepare for Label Studio**:
   ```bash
   python3 install/data_recorder/lib/data_recorder/prepare_for_labelstudio.py /path/to/session_directory
   ```

3. **Annotate in Label Studio**:
   - Import the generated `manifest.json` file
   - Draw bounding boxes around objects
   - Export in YOLO format

4. **Combine with Motor Data**:
   ```bash
   python3 install/data_recorder/lib/data_recorder/combine_yolo_with_motor_data.py /path/to/session_directory /path/to/yolo_annotations
   ```

5. **Train YOLO Model**:
   ```bash
   cd /path/to/combined_dataset
   python3 train_yolo.py
   ```

### Benefits for YOLO Training:

- ✅ **Perfect Image Format**: JPG files ready for Label Studio
- ✅ **Motion Filtering**: Only annotate images with robot movement
- ✅ **Motor Commands**: Combine object detection with navigation data
- ✅ **End-to-End Learning**: Train models that see objects and predict actions
- ✅ **High Quality**: libcamera provides excellent image quality for detection

## Future Enhancements

Potential improvements:

- Real-time data visualization
- Automatic data quality assessment
- Integration with popular ML frameworks
- Support for additional sensor data (IMU, LiDAR, etc.)
- Data augmentation during recording
- Automatic object detection during recording
