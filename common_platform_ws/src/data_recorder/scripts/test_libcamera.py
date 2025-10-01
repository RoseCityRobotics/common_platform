#!/usr/bin/env python3
"""
Test script to verify libcamera integration with the data recorder.

This script tests:
1. Camera detection with libcamera
2. ROS2 camera_ros node functionality
3. Data recorder integration
"""

import subprocess
import time
import sys
import os
from pathlib import Path

def run_command(cmd, timeout=10):
  """Run a command with timeout."""
  try:
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
    return result.returncode == 0, result.stdout, result.stderr
  except subprocess.TimeoutExpired:
    return False, "", "Command timed out"

def test_camera_detection():
  """Test if camera is detected by libcamera."""
  print("1. Testing camera detection...")
  success, stdout, stderr = run_command("cam -l")
  if success and "imx477" in stdout:
    print("   ✓ IMX477 camera detected")
    return True
  else:
    print("   ✗ Camera detection failed")
    print(f"   Error: {stderr}")
    return False

def test_camera_ros_node():
  """Test if camera_ros node can start."""
  print("\n2. Testing camera_ros node...")
  
  # Start camera node in background
  proc = subprocess.Popen(
    "ros2 run camera_ros camera_node --ros-args -p camera:=0 -p width:=640 -p height:=480",
    shell=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE
  )
  
  # Wait a bit for node to start
  time.sleep(3)
  
  # Check if node is running
  success, stdout, stderr = run_command("ros2 node list | grep camera")
  if success and "camera" in stdout:
    print("   ✓ camera_ros node started successfully")
    proc.terminate()
    proc.wait()
    return True
  else:
    print("   ✗ camera_ros node failed to start")
    print(f"   Error: {stderr}")
    proc.terminate()
    proc.wait()
    return False

def test_topic_publishing():
  """Test if camera topic is publishing."""
  print("\n3. Testing topic publishing...")
  
  # Start camera node
  proc = subprocess.Popen(
    "ros2 run camera_ros camera_node --ros-args -p camera:=0 -p width:=640 -p height:=480",
    shell=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE
  )
  
  time.sleep(3)
  
  # Check if topic exists
  success, stdout, stderr = run_command("ros2 topic list | grep image_raw")
  if success and "image_raw" in stdout:
    print("   ✓ Camera topic found")
    
    # Check topic info
    success, stdout, stderr = run_command("ros2 topic info /camera/image_raw")
    if success:
      print("   ✓ Topic info retrieved")
      proc.terminate()
      proc.wait()
      return True
  
  print("   ✗ Topic publishing failed")
  proc.terminate()
  proc.wait()
  return False

def test_data_recorder_launch():
  """Test if data recorder can be launched with libcamera."""
  print("\n4. Testing data recorder launch...")
  
  # Check if launch file exists
  launch_file = Path("/home/rcr/repos/common_platform/common_platform_ws/install/data_recorder/share/data_recorder/launch/data_recorder_with_camera.launch.py")
  if not launch_file.exists():
    print("   ✗ Launch file not found")
    return False
  
  print("   ✓ Launch file exists")
  
  # Test launch file syntax (dry run)
  success, stdout, stderr = run_command("ros2 launch data_recorder data_recorder_with_camera.launch.py --show-args")
  if success:
    print("   ✓ Launch file syntax is valid")
    return True
  else:
    print("   ✗ Launch file syntax error")
    print(f"   Error: {stderr}")
    return False

def main():
  print("=== libcamera Integration Test ===\n")
  
  # Check if we're in the right environment
  if not Path("install/setup.bash").exists():
    print("Please run this script from the workspace root directory")
    return 1
  
  # Source the workspace
  print("Sourcing workspace...")
  os.system("source install/setup.bash")
  
  tests = [
    test_camera_detection,
    test_camera_ros_node,
    test_topic_publishing,
    test_data_recorder_launch,
  ]
  
  passed = 0
  total = len(tests)
  
  for test in tests:
    if test():
      passed += 1
  
  print(f"\n=== Test Results ===")
  print(f"Passed: {passed}/{total}")
  
  if passed == total:
    print("✓ All tests passed! libcamera integration is working.")
    print("\nYou can now use:")
    print("  ros2 launch data_recorder data_recorder_with_camera.launch.py camera_type:=libcamera")
    return 0
  else:
    print("✗ Some tests failed. Check the errors above.")
    return 1

if __name__ == "__main__":
  sys.exit(main())
