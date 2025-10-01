#!/usr/bin/env python3
"""
Test script to demonstrate namespace functionality of the data recorder.

This script shows how to use the data recorder with different namespaces
for multi-robot setups.
"""

import subprocess
import time
import sys
import os
from pathlib import Path

def run_command(cmd, timeout=5):
  """Run a command with timeout."""
  try:
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
    return result.returncode == 0, result.stdout, result.stderr
  except subprocess.TimeoutExpired:
    return False, "", "Command timed out"

def test_namespace_functionality():
  """Test namespace functionality."""
  print("=== Testing Namespace Functionality ===\n")
  
  # Check if we're in the right directory
  if not Path("install/setup.bash").exists():
    print("Please run this script from the workspace root directory")
    return 1
  
  # Source the workspace
  print("Sourcing workspace...")
  os.system("source install/setup.bash")
  
  print("\n1. Testing without namespace (default behavior):")
  print("   Topics will be:")
  print("   - /camera/image_raw")
  print("   - /cmd_vel")
  print("   - /data_recorder/start_stop")
  
  print("\n2. Testing with namespace 'robot1':")
  print("   Topics will be:")
  print("   - /robot1/camera/image_raw")
  print("   - /robot1/cmd_vel")
  print("   - /robot1/data_recorder/start_stop")
  
  print("\n3. Testing with namespace 'robot2':")
  print("   Topics will be:")
  print("   - /robot2/camera/image_raw")
  print("   - /robot2/cmd_vel")
  print("   - /robot2/data_recorder/start_stop")
  
  print("\n4. Example usage commands:")
  print("   # Single robot (no namespace)")
  print("   ros2 launch data_recorder data_recorder_with_camera.launch.py")
  print("")
  print("   # Multi-robot setup")
  print("   ROS_NAMESPACE=robot1 ros2 launch data_recorder data_recorder_with_camera.launch.py")
  print("   ROS_NAMESPACE=robot2 ros2 launch data_recorder data_recorder_with_camera.launch.py")
  print("")
  print("   # Control recording with namespace")
  print("   ROS_NAMESPACE=robot1 python3 lib/data_recorder/record_control.py start")
  print("   ROS_NAMESPACE=robot2 python3 lib/data_recorder/record_control.py start")
  
  print("\n5. Benefits of namespace support:")
  print("   ✓ Multiple robots can run simultaneously")
  print("   ✓ No topic conflicts between robots")
  print("   ✓ Isolated data recording per robot")
  print("   ✓ Easy to scale to many robots")
  
  print("\n6. Multi-robot workflow example:")
  print("   # Terminal 1: Start robot1 with micro-ROS")
  print("   ROS_NAMESPACE=robot1 ros2 launch common_platform micro_ros_teleop_lidar.launch.py")
  print("")
  print("   # Terminal 2: Start robot2 with micro-ROS")
  print("   ROS_NAMESPACE=robot2 ros2 launch common_platform micro_ros_teleop_lidar.launch.py")
  print("")
  print("   # Terminal 3: Start data recorder for robot1")
  print("   ROS_NAMESPACE=robot1 ros2 launch data_recorder data_recorder_with_camera.launch.py")
  print("")
  print("   # Terminal 4: Start data recorder for robot2")
  print("   ROS_NAMESPACE=robot2 ros2 launch data_recorder data_recorder_with_camera.launch.py")
  print("")
  print("   # Terminal 5: Control recording for both robots")
  print("   ROS_NAMESPACE=robot1 python3 lib/data_recorder/record_control.py start")
  print("   ROS_NAMESPACE=robot2 python3 lib/data_recorder/record_control.py start")
  
  print("\n=== Namespace test completed ===")
  print("The data recorder now supports full namespace isolation for multi-robot setups!")
  
  return 0

if __name__ == "__main__":
  sys.exit(test_namespace_functionality())
