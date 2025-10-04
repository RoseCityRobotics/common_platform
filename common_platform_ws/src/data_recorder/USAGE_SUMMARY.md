# Data Recorder Usage Summary

## 🚀 **Quick Start Commands**

### **Standard Recording (SD Card)**
```bash
# Start recording
ros2 launch data_recorder data_recorder_with_camera.launch.py

# Control recording
python3 install/data_recorder/lib/data_recorder/record_control.py start
python3 install/data_recorder/lib/data_recorder/record_control.py stop
python3 install/data_recorder/lib/data_recorder/record_control.py status
```

### **High-Speed RAM Disk Recording**
```bash
# Start recording with RAM disk (requires sudo)
sudo python3 install/data_recorder/lib/data_recorder/ramdisk_control.py start

# Stop recording and backup to SD card
sudo python3 install/data_recorder/lib/data_recorder/ramdisk_control.py stop

# Check status
python3 install/data_recorder/lib/data_recorder/ramdisk_control.py status
```

## 📊 **Performance Comparison**

| Method | Write Speed | Use Case | Pros | Cons |
|--------|-------------|----------|------|------|
| **SD Card** | ~20 MB/s | Standard recording | Simple, persistent | Slower, wear |
| **RAM Disk** | ~500 MB/s | High-speed recording | Very fast, no wear | Requires sudo, temporary |

## 🎯 **When to Use Each Method**

### **Use SD Card Recording When:**
- Simple data collection
- Long recording sessions
- No sudo access available
- Standard performance is sufficient

### **Use RAM Disk Recording When:**
- High-speed data collection needed
- Short recording sessions
- Sudo access available
- Maximum performance required

## 🔧 **Configuration Options**

### **Output Location**
```bash
# Custom output directory
ros2 launch data_recorder data_recorder_with_camera.launch.py output_dir:=/path/to/your/data

# RAM disk with custom backup location
sudo python3 install/data_recorder/lib/data_recorder/ramdisk_recorder.py \
  --backup-path /path/to/backup --start
```

### **Multi-Robot Setup**
```bash
# Robot 1
ROS_NAMESPACE=robot1 ros2 launch data_recorder data_recorder_with_camera.launch.py

# Robot 2
ROS_NAMESPACE=robot2 ros2 launch data_recorder data_recorder_with_camera.launch.py
```

### **Camera Configuration**
```bash
# Use libcamera (Pi HQ Camera)
ros2 launch data_recorder data_recorder_with_camera.launch.py camera_type:=libcamera

# Use v4l2 (USB cameras)
ros2 launch data_recorder data_recorder_with_camera.launch.py camera_type:=v4l2
```

## 📁 **Data Structure**

Both methods create the same data structure:

```
output_directory/
└── session_YYYYMMDD_HHMMSS/
    ├── images/
    │   ├── 00000000.jpg
    │   ├── 00000001.jpg
    │   └── ...
    └── metadata/
        └── data_log.csv
```

## 🎮 **Complete Workflow Examples**

### **1. Basic Data Collection**
```bash
# Start recording
ros2 launch data_recorder data_recorder_with_camera.launch.py

# Do teleoperation
# ... drive your robot ...

# Stop recording
python3 install/data_recorder/lib/data_recorder/record_control.py stop
```

### **2. High-Speed Data Collection**
```bash
# Start RAM disk recording
sudo python3 install/data_recorder/lib/data_recorder/ramdisk_control.py start

# Do teleoperation
# ... drive your robot ...

# Stop and backup
sudo python3 install/data_recorder/lib/data_recorder/ramdisk_control.py stop
```

### **3. Multi-Robot Data Collection**
```bash
# Terminal 1 - Robot 1
ROS_NAMESPACE=robot1 ros2 launch data_recorder data_recorder_with_camera.launch.py

# Terminal 2 - Robot 2
ROS_NAMESPACE=robot2 ros2 launch data_recorder data_recorder_with_camera.launch.py

# Control each robot separately
ROS_NAMESPACE=robot1 python3 install/data_recorder/lib/data_recorder/record_control.py start
ROS_NAMESPACE=robot2 python3 install/data_recorder/lib/data_recorder/record_control.py start
```

### **4. YOLO Training Data Preparation**
```bash
# Record data
ros2 launch data_recorder data_recorder_with_camera.launch.py

# Prepare for Label Studio
python3 install/data_recorder/lib/data_recorder/prepare_for_labelstudio.py \
  /home/rcr/teleop_data/session_*/images

# After annotation, combine with motor data
python3 install/data_recorder/lib/data_recorder/combine_yolo_with_motor_data.py \
  /path/to/labelstudio/export \
  /home/rcr/teleop_data/session_*/metadata/data_log.csv
```

## 🚨 **Important Notes**

### **RAM Disk Recording**
- **Requires sudo privileges**
- **Data is lost on reboot** (unless backed up)
- **Uses system RAM** (may affect performance)
- **Always use `--stop` to backup data**

### **SD Card Recording**
- **No special privileges needed**
- **Data persists on reboot**
- **Slower write speeds**
- **May wear out SD card over time**

### **General**
- **Check available space** before recording
- **Monitor system resources** during recording
- **Use appropriate method** for your use case

## 🔍 **Troubleshooting**

### **Common Issues**

#### **"Permission denied" (RAM disk)**
```bash
# Solution: Use sudo
sudo python3 install/data_recorder/lib/data_recorder/ramdisk_control.py start
```

#### **"No space left on device"**
```bash
# Check available space
df -h

# Free up space or use different location
ros2 launch data_recorder data_recorder_with_camera.launch.py output_dir:=/path/with/space
```

#### **"Data recorder not starting"**
```bash
# Check ROS2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Check if camera is available
ros2 topic list | grep camera
```

#### **"Camera not found"**
```bash
# Check camera devices
ls /dev/video*
ls /dev/media*

# Test camera
python3 install/data_recorder/lib/data_recorder/test_libcamera.py
```

## 📚 **Additional Resources**

- **RAMDISK_README.md** - Detailed RAM disk documentation
- **README.md** - Complete package documentation
- **test_ramdisk.py** - Test RAM disk functionality
- **test_libcamera.py** - Test camera functionality
- **test_namespace.py** - Test namespace functionality

## 🎯 **Next Steps**

1. **Choose your recording method** (SD card or RAM disk)
2. **Test the setup** with the test scripts
3. **Start recording** your training data
4. **Process data** for neural network training
5. **Annotate data** with Label Studio (if needed)
6. **Train your models** with the collected data

The data recorder is now ready for high-quality data collection for your neural network training!
