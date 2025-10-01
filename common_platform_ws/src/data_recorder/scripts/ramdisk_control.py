#!/usr/bin/env python3
"""
Simple control script for RAM disk recording.

Usage:
  python3 ramdisk_control.py start    # Start recording with RAM disk
  python3 ramdisk_control.py stop     # Stop recording and backup to SD card
  python3 ramdisk_control.py status   # Check recording status
"""

import sys
import os
import subprocess
import time
from pathlib import Path

def run_ramdisk_recorder(args):
  """Run the ramdisk_recorder.py script with given arguments."""
  script_path = Path(__file__).parent / "ramdisk_recorder.py"
  cmd = ["python3", str(script_path)] + args
  
  # Set environment for ROS
  env = os.environ.copy()
  env["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "0")
  
  try:
    result = subprocess.run(cmd, env=env, check=True)
    return result.returncode == 0
  except subprocess.CalledProcessError as e:
    print(f"Error running ramdisk recorder: {e}")
    return False
  except FileNotFoundError:
    print("Error: ramdisk_recorder.py not found")
    return False

def check_status():
  """Check if recording is active."""
  try:
    # Check if data recorder is running
    result = subprocess.run(
      ["ros2", "node", "list"], 
      capture_output=True, text=True, check=True
    )
    
    if "data_recorder" in result.stdout:
      print("✓ Data recorder is running")
      
      # Check if RAM disk is mounted
      ramdisk_path = "/mnt/recording_ramdisk"
      result = subprocess.run(
        ["mountpoint", ramdisk_path], 
        capture_output=True, text=True, check=False
      )
      
      if result.returncode == 0:
        print("✓ RAM disk is mounted")
        
        # Check for data
        data_path = Path(ramdisk_path) / "teleop_data"
        if data_path.exists():
          session_dirs = list(data_path.glob("session_*"))
          if session_dirs:
            latest = max(session_dirs, key=os.path.getctime)
            image_count = len(list((latest / "images").glob("*.jpg")))
            print(f"✓ Recording active: {image_count} images in {latest.name}")
          else:
            print("✓ Recording active: No data yet")
        else:
          print("✓ Recording active: No data directory yet")
      else:
        print("⚠ RAM disk not mounted")
    else:
      print("✗ Data recorder not running")
      
  except subprocess.CalledProcessError:
    print("✗ ROS2 not available or no nodes running")
  except Exception as e:
    print(f"Error checking status: {e}")

def main():
  if len(sys.argv) != 2:
    print("Usage: python3 ramdisk_control.py [start|stop|status]")
    print()
    print("Commands:")
    print("  start  - Start recording with RAM disk")
    print("  stop   - Stop recording and backup to SD card")
    print("  status - Check recording status")
    return 1
  
  command = sys.argv[1].lower()
  
  if command == "start":
    print("Starting RAM disk recording...")
    success = run_ramdisk_recorder(["--start"])
    if success:
      print("✓ Recording started successfully!")
      print("Press Ctrl+C in the recording terminal to stop and backup")
    else:
      print("✗ Failed to start recording")
    return 0 if success else 1
    
  elif command == "stop":
    print("Stopping recording and backing up to SD card...")
    success = run_ramdisk_recorder(["--stop"])
    if success:
      print("✓ Recording stopped and data backed up successfully!")
    else:
      print("✗ Failed to stop recording or backup data")
    return 0 if success else 1
    
  elif command == "status":
    check_status()
    return 0
    
  else:
    print(f"Invalid command: {command}")
    print("Use: start, stop, or status")
    return 1

if __name__ == "__main__":
  sys.exit(main())
