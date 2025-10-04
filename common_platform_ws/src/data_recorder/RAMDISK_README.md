# RAM Disk Data Recording

This document describes the advanced RAM disk recording functionality that provides high-speed data recording with automatic backup to SD card.

## 🚀 **Features**

- **Automatic RAM disk mounting** - Creates a dedicated RAM disk for fast recording
- **High-speed recording** - Records directly to RAM for maximum performance
- **Automatic backup** - Copies data to SD card with progress bar on stop
- **Graceful shutdown** - Handles Ctrl+C and system signals properly
- **Error recovery** - Robust error handling and cleanup
- **Progress monitoring** - Real-time progress bar during backup
- **Namespace support** - Works with multi-robot setups

## 📋 **Prerequisites**

- **Sudo privileges** - Required for RAM disk operations
- **Available RAM** - At least 2GB free RAM recommended
- **SD card space** - Sufficient space for data backup

## 🛠 **Installation**

The RAM disk scripts are automatically installed with the data recorder package:

```bash
cd /home/rcr/repos/common_platform/common_platform_ws
colcon build --packages-select data_recorder
source install/setup.bash
```

## 🎯 **Quick Start**

### **Simple Control (Recommended)**

```bash
# Start recording with RAM disk
python3 install/data_recorder/lib/data_recorder/ramdisk_control.py start

# Stop recording and backup to SD card
python3 install/data_recorder/lib/data_recorder/ramdisk_control.py stop

# Check recording status
python3 install/data_recorder/lib/data_recorder/ramdisk_control.py status
```

### **Advanced Control**

```bash
# Start with custom settings
python3 install/data_recorder/lib/data_recorder/ramdisk_recorder.py \
  --ramdisk-size 4G \
  --backup-path /home/rcr/robot_data \
  --namespace robot1 \
  --start

# Stop and backup
python3 install/data_recorder/lib/data_recorder/ramdisk_recorder.py --stop
```

## 📊 **How It Works**

### **1. Recording Start**
```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Mount RAM     │ -> │  Start Data      │ -> │   Record to     │
│   Disk (2GB)    │    │  Recorder        │    │   RAM Disk      │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

### **2. Recording Process**
- Data recorder writes directly to RAM disk
- Images saved as JPG files
- Metadata logged to CSV
- Real-time monitoring available

### **3. Recording Stop**
```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Stop Data     │ -> │  Copy to SD      │ -> │   Unmount RAM   │
│   Recorder      │    │  Card (Progress) │    │   Disk          │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## ⚙️ **Configuration**

### **RAM Disk Settings**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--ramdisk-size` | `2G` | Size of RAM disk (e.g., 2G, 4G) |
| `--ramdisk-path` | `/mnt/recording_ramdisk` | Mount point for RAM disk |
| `--backup-path` | `/home/rcr/robot_data` | SD card backup location |

### **Recording Settings**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--namespace` | None | ROS namespace for multi-robot |
| `camera_type` | `libcamera` | Camera type (libcamera/v4l2) |
| `record_rate` | `30.0` | Recording rate in Hz |

## 📁 **Data Structure**

### **RAM Disk (During Recording)**
```
/mnt/recording_ramdisk/
└── teleop_data/
    └── session_20241001_143022/
        ├── images/
        │   ├── 00000000.jpg
        │   ├── 00000001.jpg
        │   └── ...
        └── metadata/
            └── data_log.csv
```

### **SD Card (After Backup)**
```
/home/rcr/robot_data/
└── session_20241001_143022_backup_20241001_143045/
    ├── images/
    │   ├── 00000000.jpg
    │   ├── 00000001.jpg
    │   └── ...
    └── metadata/
        └── data_log.csv
```

## 🎮 **Usage Examples**

### **Basic Recording**
```bash
# Start recording
python3 install/data_recorder/lib/data_recorder/ramdisk_control.py start

# ... do your teleoperation ...

# Stop and backup
python3 install/data_recorder/lib/data_recorder/ramdisk_control.py stop
```

### **Multi-Robot Setup**
```bash
# Robot 1
ROS_NAMESPACE=robot1 python3 install/data_recorder/lib/data_recorder/ramdisk_control.py start

# Robot 2
ROS_NAMESPACE=robot2 python3 install/data_recorder/lib/data_recorder/ramdisk_control.py start
```

### **Custom Configuration**
```bash
# Large RAM disk for long recording
python3 install/data_recorder/lib/data_recorder/ramdisk_recorder.py \
  --ramdisk-size 8G \
  --backup-path /media/usb/recordings \
  --start
```

## 📈 **Performance Benefits**

### **Speed Comparison**
| Method | Write Speed | Read Speed | Notes |
|--------|-------------|------------|-------|
| SD Card | ~20 MB/s | ~30 MB/s | Limited by SD card |
| RAM Disk | ~500 MB/s | ~1000 MB/s | Limited by RAM speed |
| **Improvement** | **25x faster** | **33x faster** | Significant boost |

### **Real-World Impact**
- **30Hz recording**: No dropped frames
- **Large datasets**: Handle hours of recording
- **System stability**: No I/O bottlenecks
- **Battery life**: Reduced SD card wear

## 🔧 **Troubleshooting**

### **Common Issues**

#### **"Permission denied" errors**
```bash
# Solution: Run with sudo
sudo python3 install/data_recorder/lib/data_recorder/ramdisk_control.py start
```

#### **"No space left on device"**
```bash
# Solution: Increase RAM disk size or free up RAM
python3 install/data_recorder/lib/data_recorder/ramdisk_recorder.py \
  --ramdisk-size 4G --start
```

#### **"Failed to mount RAM disk"**
```bash
# Solution: Check if already mounted
mountpoint /mnt/recording_ramdisk
sudo umount /mnt/recording_ramdisk  # If needed
```

#### **"Data recorder not starting"**
```bash
# Solution: Check ROS2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 node list
```

### **Status Checking**
```bash
# Check recording status
python3 install/data_recorder/lib/data_recorder/ramdisk_control.py status

# Check RAM disk
df -h /mnt/recording_ramdisk

# Check data recorder
ros2 node list | grep data_recorder
```

## 🚨 **Important Notes**

### **Memory Requirements**
- **Minimum**: 2GB free RAM
- **Recommended**: 4GB free RAM
- **Maximum**: 50% of total RAM

### **Data Safety**
- **RAM disk data is lost on reboot**
- **Always backup before shutdown**
- **Use `--stop` command to backup safely**

### **System Impact**
- **Reduces available RAM for other processes**
- **May affect system performance if RAM is low**
- **Monitor system resources during recording**

## 🔄 **Workflow Integration**

### **With Label Studio**
```bash
# 1. Record data with RAM disk
python3 install/data_recorder/lib/data_recorder/ramdisk_control.py start
# ... teleoperation ...
python3 install/data_recorder/lib/data_recorder/ramdisk_control.py stop

# 2. Prepare for Label Studio
python3 install/data_recorder/lib/data_recorder/prepare_for_labelstudio.py \
  /home/rcr/robot_data/session_*/images

# 3. Annotate in Label Studio
# ... annotation work ...

# 4. Combine with motor data
python3 install/data_recorder/lib/data_recorder/combine_yolo_with_motor_data.py \
  /path/to/labelstudio/export \
  /home/rcr/robot_data/session_*/metadata/data_log.csv
```

### **With Neural Network Training**
```bash
# 1. Record training data
python3 install/data_recorder/lib/data_recorder/ramdisk_control.py start
# ... collect training data ...
python3 install/data_recorder/lib/data_recorder/ramdisk_control.py stop

# 2. Process for training
python3 install/data_recorder/lib/data_recorder/process_training_data.py \
  /home/rcr/robot_data/session_* \
  --format hdf5 \
  --output /home/rcr/training_data.h5
```

## 📚 **Advanced Usage**

### **Script Integration**
```python
# In your own Python scripts
from ramdisk_recorder import RamdiskRecorder

recorder = RamdiskRecorder(
  ramdisk_size="4G",
  backup_path="/home/rcr/robot_data"
)

# Start recording
if recorder.start_recording():
  # Do your work
  time.sleep(60)  # Record for 1 minute
  
  # Stop and backup
  recorder.stop_recording()
```

### **Custom Progress Monitoring**
```python
# Monitor recording progress
import time
from pathlib import Path

ramdisk_path = Path("/mnt/recording_ramdisk/teleop_data")
while True:
  if ramdisk_path.exists():
    sessions = list(ramdisk_path.glob("session_*"))
    if sessions:
      latest = max(sessions, key=lambda x: x.stat().st_mtime)
      images = list((latest / "images").glob("*.jpg"))
      print(f"Recording: {len(images)} images")
  time.sleep(5)
```

This RAM disk recording system provides a professional-grade solution for high-speed data collection with automatic persistence, making it perfect for neural network training and research applications.
