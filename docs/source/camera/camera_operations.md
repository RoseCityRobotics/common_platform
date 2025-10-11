# Camera Operations

This guide covers camera operations, setup, and troubleshooting for the RCR Common Robotics Platform.

## Overview

The robot uses a camera for visual perception, navigation, and object detection. The camera provides real-time image data for various applications.

## Camera Specifications

- **Model**: USB Camera
- **Resolution**: 640x480 (configurable)
- **Frame Rate**: 30 FPS
- **Interface**: USB 2.0
- **Field of View**: 60° (approximate)

## Prerequisites

- **Discovery Server Running**: Ensure a ROS discovery server is running (see [Discovery Server Setup](../operations/discovery_server.md))
- Camera hardware properly connected
- ROS2 environment sourced
- Sufficient battery charge

## Camera Setup

### 1. Hardware Connection

1. **Physical Mounting**
   - Mount camera on designated mounting point
   - Ensure clear field of view
   - Secure mounting to prevent vibration

2. **Power Connection**
   - Connect USB cable to robot controller
   - Verify power requirements
   - Check for proper grounding

3. **Data Connection**
   - Connect USB cable to robot controller
   - Verify USB communication
   - Test connection stability

### 2. Software Configuration

Open a new terminal window 📟

**Launch Camera:**
```bash
# Launch camera node
cd ~/repos/common_platform/common_platform_ws
source install/setup.bash
ros2 launch sensors camera.launch.py
```

You will need to leave this window open for as long as you want the camera running. If you want to verify that the camera data is streaming you can echo the topic, to do so open a new terminal window 📟

**Check camera data stream:**
```bash
ros2 topic echo /${ROS_NAME}/camera/image_raw
```
**Monitor data rate (in hertz):**
```bash
ros2 topic hz /${ROS_NAME}/camera/image_raw
```

Once you have checked the data stream, you can close this window as this is just for testing and diagnosis. You want to get in the habit of looking for the ros topic to verify data is streaming. You can do this with:

**View all ROS topics**
 ```bash
 ros2 topic list
 ```

## Camera Applications

### Computer Vision
In order to collect images to annotate in Label Studio, use our custom data_recorder node. This is the first step in fine tuning an existing neural network. Check the [Data Annotation](../data_annotation/label_studio.md) document for information on labeling.


## Recording Image Files

Open a new terminal window 📟. Launch the image recorder program with:

```bash
cd ~/repos/common_platform/common_platform_ws/
source install/setup.bash
ros2 launch data_recorder ramdisk_recorder.launch.py
```

The recorder needs to be stopped manually using `Ctrl+C`. It should automatically copy the recorded frames from the RAM disk to the following location on your micro SD card: ~/teleop_data/ . Check for a folder called session_<DATE>_<TIME> .

You should see it with:
```bash
cd ~/teleop_data/
ls
```

---

## Recording Video Files

Open a new terminal window 📟

```bash
ros2 run image_view video_recorder --ros-args -p filename:=/tmp/robot_video.avi
```

The video file will be saved to /tmp/robot_video.avi on your Raspberry Pi.

### Visual Navigation

**Feature Detection:**
- Detect visual features
- Use for localization
- Implement visual SLAM

**Object Recognition:**
- Identify objects
- Navigate to targets
- Avoid obstacles

### Real-time Monitoring
```bash
# View camera feed - you need a virtual environment running to do this - more details soon but in the meantime run this on Joe's computer.
ros2 run rqt_image_view rqt_image_view
```

## Camera Calibration

### 1. Intrinsic Calibration

**Calibration Process:**
1. Print calibration pattern
2. Launch camera
3. Run calibration tool
4. Save calibration parameters

**Calibration Command:**
```bash
# Run camera calibration
# Command to be added after verification
```

### 2. Extrinsic Calibration

**Coordinate Frame Alignment:**
- Align camera with robot frame
- Verify coordinate transformations
- Test with known objects

**Transform Setup:**
```bash
# Check transform tree
ros2 run tf2_tools view_frames

# Verify camera transform
ros2 run tf2_ros tf2_echo base_link camera_link
```

## Camera Data Analysis

### 1. Image Data

**Image Message:**
```yaml
std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
```

### 2. Data Quality

**Check Image Quality:**
```bash
# View camera feed
ros2 run rqt_image_view rqt_image_view

# Check for:
# - Clear image
# - Proper exposure
# - No distortion
# - Consistent frame rate
```

## Troubleshooting

### Common Issues

Check the camera hardware is present
```bash
cam -l
```

**No Camera Data:**
- Check USB connection
- Verify Teensy is powered and running
- Ensure a discovery server is running
- Connect Cursor to your Pi via SSH and ask for diagnosis help

**Poor Image Quality:**
- Clean camera lens
- Check for obstructions
- Verify focus
- Test in different lighting

**Inconsistent Frame Rate:**
- Check USB bandwidth
- Verify power supply
- Test with different resolution
- Check for interference

### Diagnostic Commands

```bash
# Check USB devices
lsusb | grep -i camera

# Check video devices
ls /dev/video*

# Test camera
ros2 run usb_cam usb_cam_node --ros-args -p video_device:=/dev/video0

# Monitor data quality
ros2 topic echo /${ROS_NAME}/camera/image_raw
ros2 topic hz /${ROS_NAME}/camera/image_raw
```

### Error Codes

**Common Error Messages:**
- "Failed to open video device" - Check USB connection
- "No frames received" - Check power and connections
- "Invalid image data" - Check for interference or damage

## Maintenance

### Regular Maintenance
- Visual inspection
- Check for obstructions
- Verify image quality
- Clean camera lens
- Check mounting stability
- Test focus

### Cleaning Procedures

**Camera Lens:**
1. Power off robot
2. Use soft, lint-free cloth
3. Clean gently to avoid scratches
4. Remove dust and debris
5. Verify clear view

**Mounting Area:**
- Clean mounting surface
- Check for loose connections
- Verify stability

## Performance Optimization

### Image Processing

**Resolution:**
- Adjust resolution for performance
- Balance quality vs. speed
- Consider bandwidth requirements

**Frame Rate:**
- Adjust frame rate as needed
- Balance performance vs. accuracy
- Consider computational load

### Parameter Tuning

**Camera Parameters:**
```yaml
# In launch file
image_width: 640
image_height: 480
framerate: 30
```

**Processing Parameters:**
- Adjust processing algorithms
- Optimize for real-time performance
- Balance accuracy vs. speed
---

*For LiDAR operations, see [LiDAR Operations](../lidar_operations.md)*
*For IMU operations, see [IMU Operations](../operations/sensors/imu_operations.md)*
*For troubleshooting, see [Troubleshooting](../operations/troubleshooting/index.md)*
