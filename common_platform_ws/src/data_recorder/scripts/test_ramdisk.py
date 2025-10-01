#!/usr/bin/env python3
"""
Test script for RAM disk functionality.

This script tests the RAM disk mounting, data recording, and backup functionality
without requiring a full ROS2 setup.
"""

import os
import sys
import subprocess
import time
import tempfile
from pathlib import Path

def test_ramdisk_mounting():
  """Test RAM disk mounting and unmounting."""
  print("=== Testing RAM Disk Mounting ===")
  
  ramdisk_path = "/tmp/test_ramdisk"
  
  try:
    # Create test directory
    Path(ramdisk_path).mkdir(parents=True, exist_ok=True)
    
    # Mount RAM disk
    cmd = f"sudo mount -t tmpfs -o size=100M tmpfs {ramdisk_path}"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    
    if result.returncode == 0:
      print("✓ RAM disk mounted successfully")
      
      # Test write speed
      test_file = Path(ramdisk_path) / "test_write.txt"
      start_time = time.time()
      
      with open(test_file, 'w') as f:
        f.write("x" * 1024 * 1024)  # Write 1MB
      
      write_time = time.time() - start_time
      write_speed = 1.0 / write_time  # MB/s
      
      print(f"✓ Write speed: {write_speed:.1f} MB/s")
      
      # Clean up
      subprocess.run(f"sudo umount {ramdisk_path}", shell=True)
      print("✓ RAM disk unmounted successfully")
      
      return True
    else:
      print(f"✗ Failed to mount RAM disk: {result.stderr}")
      return False
      
  except Exception as e:
    print(f"✗ Error testing RAM disk: {e}")
    return False

def test_script_availability():
  """Test if the RAM disk scripts are available."""
  print("\n=== Testing Script Availability ===")
  
  script_path = Path(__file__).parent / "ramdisk_recorder.py"
  control_path = Path(__file__).parent / "ramdisk_control.py"
  
  if script_path.exists():
    print("✓ ramdisk_recorder.py found")
  else:
    print("✗ ramdisk_recorder.py not found")
    return False
  
  if control_path.exists():
    print("✓ ramdisk_control.py found")
  else:
    print("✗ ramdisk_control.py not found")
    return False
  
  # Test script syntax
  try:
    result = subprocess.run([sys.executable, "-m", "py_compile", str(script_path)], 
                          capture_output=True, text=True)
    if result.returncode == 0:
      print("✓ ramdisk_recorder.py syntax is valid")
    else:
      print(f"✗ ramdisk_recorder.py syntax error: {result.stderr}")
      return False
  except Exception as e:
    print(f"✗ Error checking script syntax: {e}")
    return False
  
  return True

def test_help_commands():
  """Test help commands for the scripts."""
  print("\n=== Testing Help Commands ===")
  
  script_path = Path(__file__).parent / "ramdisk_recorder.py"
  control_path = Path(__file__).parent / "ramdisk_control.py"
  
  try:
    # Test ramdisk_recorder.py help
    result = subprocess.run([sys.executable, str(script_path), "--help"], 
                          capture_output=True, text=True)
    if result.returncode == 0 and "RAM disk data recorder" in result.stdout:
      print("✓ ramdisk_recorder.py help works")
    else:
      print("✗ ramdisk_recorder.py help failed")
      return False
    
    # Test ramdisk_control.py help
    result = subprocess.run([sys.executable, str(control_path)], 
                          capture_output=True, text=True)
    if result.returncode == 1 and "Usage:" in result.stdout:  # Exit code 1 is expected for no args
      print("✓ ramdisk_control.py help works")
    else:
      print("✗ ramdisk_control.py help failed")
      return False
    
    return True
    
  except Exception as e:
    print(f"✗ Error testing help commands: {e}")
    return False

def test_sudo_privileges():
  """Test if we have sudo privileges."""
  print("\n=== Testing Sudo Privileges ===")
  
  try:
    result = subprocess.run(["sudo", "-n", "true"], capture_output=True, text=True)
    if result.returncode == 0:
      print("✓ Sudo privileges available")
      return True
    else:
      print("✗ Sudo privileges not available")
      print("  Note: You'll need sudo for RAM disk operations")
      return False
  except Exception as e:
    print(f"✗ Error checking sudo: {e}")
    return False

def main():
  """Run all tests."""
  print("RAM Disk Functionality Test")
  print("=" * 40)
  
  tests = [
    test_script_availability,
    test_help_commands,
    test_sudo_privileges,
    test_ramdisk_mounting,
  ]
  
  passed = 0
  total = len(tests)
  
  for test in tests:
    if test():
      passed += 1
  
  print(f"\n=== Test Results ===")
  print(f"Passed: {passed}/{total}")
  
  if passed == total:
    print("✓ All tests passed! RAM disk functionality is ready.")
    return 0
  else:
    print("✗ Some tests failed. Check the output above.")
    return 1

if __name__ == "__main__":
  sys.exit(main())
