#!/usr/bin/env python3
"""
Example script demonstrating how to use the data recorder package.

This script shows how to:
1. Start the data recorder
2. Control recording via topics
3. Process recorded data
4. Load data for training

Run this script after building the data_recorder package.
"""

import subprocess
import time
import os
import sys
from pathlib import Path

def run_command(cmd, background=False):
  """Run a shell command."""
  print(f"Running: {cmd}")
  if background:
    return subprocess.Popen(cmd, shell=True)
  else:
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if result.returncode != 0:
      print(f"Error: {result.stderr}")
    return result

def main():
  print("=== Data Recorder Example Usage ===\n")
  
  # Check if we're in the right directory
  if not Path("install/setup.bash").exists():
    print("Please run this script from the workspace root directory")
    print("Expected: /home/rcr/repos/common_platform/common_platform_ws")
    return 1
  
  # Source the workspace
  print("1. Sourcing workspace...")
  run_command("source install/setup.bash")
  
  # Check if data recorder is built
  print("\n2. Checking if data recorder is built...")
  result = run_command("ros2 pkg list | grep data_recorder")
  if "data_recorder" not in result.stdout:
    print("Data recorder package not found. Building...")
    run_command("colcon build --packages-select data_recorder")
    run_command("source install/setup.bash")
  
  # Show available topics
  print("\n3. Available ROS2 topics:")
  run_command("ros2 topic list")
  
  # Show data recorder launch options
  print("\n4. Data recorder launch options:")
  print("   Basic launch:")
  print("   ros2 launch data_recorder data_recorder.launch.py")
  print("")
  print("   With auto-start:")
  print("   ros2 launch data_recorder data_recorder.launch.py auto_start:=true")
  print("")
  print("   Custom output directory:")
  print("   ros2 launch data_recorder data_recorder.launch.py output_dir:=/path/to/data")
  
  # Show control commands
  print("\n5. Recording control commands:")
  print("   Start recording:")
  print("   python3 lib/data_recorder/record_control.py start")
  print("")
  print("   Stop recording:")
  print("   python3 lib/data_recorder/record_control.py stop")
  print("")
  print("   Check status:")
  print("   python3 lib/data_recorder/record_control.py status")
  
  # Show data processing commands
  print("\n6. Data processing commands:")
  print("   Process recorded data:")
  print("   python3 lib/data_recorder/process_training_data.py /path/to/session_directory")
  print("")
  print("   Validate data only:")
  print("   python3 lib/data_recorder/process_training_data.py /path/to/session_directory --validate-only")
  print("")
  print("   Process with custom format:")
  print("   python3 lib/data_recorder/process_training_data.py /path/to/session_directory --output-format hdf5")
  
  # Show example workflow
  print("\n7. Example workflow:")
  print("   # Terminal 1: Start your robot with micro-ROS")
  print("   ros2 launch common_platform micro_ros_teleop_lidar.launch.py")
  print("")
  print("   # Terminal 2: Start data recorder with camera")
  print("   ros2 launch data_recorder data_recorder_with_camera.launch.py camera_type:=libcamera")
  print("")
  print("   # Terminal 3: Control recording")
  print("   python3 lib/data_recorder/record_control.py start")
  print("   # ... drive your robot around with keyboard/joystick ...")
  print("   python3 lib/data_recorder/record_control.py stop")
  print("")
  print("   # Process the data")
  print("   python3 lib/data_recorder/process_training_data.py ~/teleop_data/session_YYYYMMDD_HHMMSS")
  
  # Show data structure
  print("\n8. Recorded data structure:")
  print("   session_YYYYMMDD_HHMMSS/")
  print("   ├── images/")
  print("   │   ├── 00000000.jpg")
  print("   │   ├── 00000001.jpg")
  print("   │   └── ...")
  print("   └── metadata/")
  print("       └── data_log.csv")
  
  # Show processed data structure
  print("\n9. Processed data structure:")
  print("   session_YYYYMMDD_HHMMSS/processed/")
  print("   ├── training_data.h5      # HDF5 format")
  print("   ├── images.npy            # NumPy arrays")
  print("   ├── labels.npy")
  print("   ├── metadata.json")
  print("   ├── statistics.json")
  print("   └── create_tfrecord.py    # TFRecord conversion script")
  
  # Show Python loading example
  print("\n10. Python code to load processed data:")
  print("    import h5py")
  print("    import numpy as np")
  print("    ")
  print("    # Load HDF5 data")
  print("    with h5py.File('training_data.h5', 'r') as f:")
  print("        images = f['images'][:]")
  print("        labels = f['labels'][:]")
  print("    ")
  print("    # Load NumPy data")
  print("    images = np.load('images.npy')")
  print("    labels = np.load('labels.npy')")
  print("    ")
  print("    # Labels format: [linear_vel, angular_vel, left_wheel_vel, right_wheel_vel]")
  
  print("\n=== Example completed ===")
  print("See README.md for detailed documentation")
  
  return 0

if __name__ == "__main__":
  sys.exit(main())
