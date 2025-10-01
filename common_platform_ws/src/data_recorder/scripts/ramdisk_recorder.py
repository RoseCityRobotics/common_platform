#!/usr/bin/env python3
"""
Advanced data recorder with automatic RAM disk mounting and SD card backup.

This script:
1. Automatically mounts a RAM disk for fast recording
2. Starts the data recorder
3. Monitors recording progress
4. On stop, copies data to SD card with progress bar
5. Unmounts the RAM disk
6. Handles errors gracefully
"""

import os
import sys
import subprocess
import time
import shutil
import threading
import signal
from pathlib import Path
from datetime import datetime
import argparse
import logging

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class RamdiskRecorder:
  def __init__(self, ramdisk_size="2G", ramdisk_path="/mnt/recording_ramdisk", 
               backup_path="/home/rcr/robot_data", namespace=None):
    self.ramdisk_size = ramdisk_size
    self.ramdisk_path = Path(ramdisk_path)
    self.backup_path = Path(backup_path)
    self.namespace = namespace
    self.recorder_process = None
    self.ramdisk_mounted = False
    self.recording_active = False
    
    # Set up signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, self._signal_handler)
    signal.signal(signal.SIGTERM, self._signal_handler)
  
  def _signal_handler(self, signum, frame):
    """Handle shutdown signals gracefully."""
    logger.info("Received shutdown signal, stopping recording...")
    self.stop_recording()
    sys.exit(0)
  
  def _run_command(self, cmd, check=True, capture_output=True):
    """Run a shell command with error handling."""
    try:
      result = subprocess.run(cmd, shell=True, check=check, capture_output=capture_output, text=True)
      return result.returncode == 0, result.stdout, result.stderr
    except subprocess.CalledProcessError as e:
      return False, e.stdout, e.stderr
  
  def _check_sudo(self):
    """Check if we have sudo privileges."""
    success, _, _ = self._run_command("sudo -n true", check=False)
    if not success:
      logger.error("This script requires sudo privileges for RAM disk operations")
      logger.error("Please run with: sudo python3 ramdisk_recorder.py")
      return False
    return True
  
  def _mount_ramdisk(self):
    """Mount a RAM disk for fast recording."""
    logger.info(f"Mounting {self.ramdisk_size} RAM disk at {self.ramdisk_path}")
    
    # Create mount point
    self.ramdisk_path.mkdir(parents=True, exist_ok=True)
    
    # Check if already mounted
    success, stdout, _ = self._run_command(f"mountpoint {self.ramdisk_path}", check=False)
    if success:
      logger.info("RAM disk already mounted")
      self.ramdisk_mounted = True
      return True
    
    # Mount the RAM disk
    cmd = f"sudo mount -t tmpfs -o size={self.ramdisk_size} tmpfs {self.ramdisk_path}"
    success, stdout, stderr = self._run_command(cmd)
    
    if success:
      logger.info("✓ RAM disk mounted successfully")
      self.ramdisk_mounted = True
      return True
    else:
      logger.error(f"Failed to mount RAM disk: {stderr}")
      return False
  
  def _unmount_ramdisk(self):
    """Unmount the RAM disk."""
    if not self.ramdisk_mounted:
      return True
    
    logger.info("Unmounting RAM disk...")
    success, stdout, stderr = self._run_command(f"sudo umount {self.ramdisk_path}")
    
    if success:
      logger.info("✓ RAM disk unmounted successfully")
      self.ramdisk_mounted = False
      return True
    else:
      logger.error(f"Failed to unmount RAM disk: {stderr}")
      return False
  
  def _start_data_recorder(self):
    """Start the data recorder with RAM disk output."""
    logger.info("Starting data recorder...")
    
    # Build the launch command
    cmd_parts = ["ros2", "launch", "data_recorder", "data_recorder_with_camera.launch.py"]
    cmd_parts.append(f"output_dir:={self.ramdisk_path}/teleop_data")
    cmd_parts.append("camera_type:=libcamera")
    cmd_parts.append("auto_start:=true")
    
    if self.namespace:
      cmd_parts.insert(0, f"ROS_NAMESPACE={self.namespace}")
    
    cmd = " ".join(cmd_parts)
    
    # Start the recorder process
    self.recorder_process = subprocess.Popen(
      cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    
    # Wait a moment for startup
    time.sleep(2)
    
    if self.recorder_process.poll() is None:
      logger.info("✓ Data recorder started successfully")
      self.recording_active = True
      return True
    else:
      logger.error("Failed to start data recorder")
      return False
  
  def _stop_data_recorder(self):
    """Stop the data recorder."""
    if not self.recording_active or not self.recorder_process:
      return True
    
    logger.info("Stopping data recorder...")
    
    # Send stop command via topic
    if self.namespace:
      topic = f"/{self.namespace}/data_recorder/start_stop"
    else:
      topic = "/data_recorder/start_stop"
    
    cmd = f"ros2 topic pub --once {topic} std_msgs/msg/Bool \"data: false\""
    self._run_command(cmd, check=False)
    
    # Wait for graceful shutdown
    time.sleep(2)
    
    # Force terminate if still running
    if self.recorder_process.poll() is None:
      self.recorder_process.terminate()
      self.recorder_process.wait(timeout=5)
    
    self.recording_active = False
    logger.info("✓ Data recorder stopped")
    return True
  
  def _copy_with_progress(self, src, dst):
    """Copy files with a text-based progress bar."""
    src_path = Path(src)
    dst_path = Path(dst)
    
    if not src_path.exists():
      logger.error(f"Source path does not exist: {src_path}")
      return False
    
    # Create destination directory
    dst_path.mkdir(parents=True, exist_ok=True)
    
    # Get list of files to copy
    files_to_copy = []
    total_size = 0
    
    for root, dirs, files in os.walk(src_path):
      for file in files:
        file_path = Path(root) / file
        file_size = file_path.stat().st_size
        files_to_copy.append((file_path, file_size))
        total_size += file_size
    
    if not files_to_copy:
      logger.warning("No files to copy")
      return True
    
    logger.info(f"Copying {len(files_to_copy)} files ({total_size / (1024*1024):.1f} MB)...")
    
    # Copy files with progress
    copied_size = 0
    copied_files = 0
    
    for src_file, file_size in files_to_copy:
      # Calculate relative path
      rel_path = src_file.relative_to(src_path)
      dst_file = dst_path / rel_path
      
      # Create destination directory
      dst_file.parent.mkdir(parents=True, exist_ok=True)
      
      # Copy file
      shutil.copy2(src_file, dst_file)
      
      # Update progress
      copied_size += file_size
      copied_files += 1
      
      # Show progress bar
      progress = copied_size / total_size
      bar_length = 50
      filled_length = int(bar_length * progress)
      bar = '█' * filled_length + '-' * (bar_length - filled_length)
      
      print(f"\r[{bar}] {progress*100:.1f}% ({copied_files}/{len(files_to_copy)} files) {copied_size/(1024*1024):.1f}MB", end='', flush=True)
    
    print()  # New line after progress bar
    logger.info("✓ Copy completed successfully")
    return True
  
  def _backup_to_sd_card(self):
    """Copy recorded data from RAM disk to SD card."""
    ramdisk_data_path = self.ramdisk_path / "teleop_data"
    
    if not ramdisk_data_path.exists():
      logger.warning("No data found in RAM disk")
      return True
    
    # Find the session directory
    session_dirs = list(ramdisk_data_path.glob("session_*"))
    if not session_dirs:
      logger.warning("No session directories found")
      return True
    
    # Use the most recent session
    latest_session = max(session_dirs, key=os.path.getctime)
    
    logger.info(f"Backing up session: {latest_session.name}")
    
    # Create backup directory with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_session_path = self.backup_path / f"{latest_session.name}_backup_{timestamp}"
    
    # Copy with progress bar
    return self._copy_with_progress(latest_session, backup_session_path)
  
  def start_recording(self):
    """Start recording with RAM disk."""
    logger.info("=== Starting RAM Disk Recording ===")
    
    # Check sudo privileges
    if not self._check_sudo():
      return False
    
    # Mount RAM disk
    if not self._mount_ramdisk():
      return False
    
    # Start data recorder
    if not self._start_data_recorder():
      self._unmount_ramdisk()
      return False
    
    logger.info("✓ Recording started successfully!")
    logger.info(f"Data is being recorded to: {self.ramdisk_path}/teleop_data")
    logger.info("Press Ctrl+C to stop recording and backup to SD card")
    
    return True
  
  def stop_recording(self):
    """Stop recording and backup to SD card."""
    logger.info("=== Stopping Recording and Backing Up ===")
    
    # Stop data recorder
    self._stop_data_recorder()
    
    # Backup to SD card
    if not self._backup_to_sd_card():
      logger.error("Failed to backup data to SD card")
      return False
    
    # Unmount RAM disk
    if not self._unmount_ramdisk():
      logger.error("Failed to unmount RAM disk")
      return False
    
    logger.info("✓ Recording stopped and data backed up successfully!")
    return True
  
  def run_interactive(self):
    """Run in interactive mode."""
    if not self.start_recording():
      return False
    
    try:
      # Wait for user to stop recording
      while self.recording_active:
        time.sleep(1)
    except KeyboardInterrupt:
      pass
    
    return self.stop_recording()

def main():
  parser = argparse.ArgumentParser(description='RAM disk data recorder with automatic backup')
  parser.add_argument('--ramdisk-size', default='2G', help='RAM disk size (default: 2G)')
  parser.add_argument('--ramdisk-path', default='/mnt/recording_ramdisk', help='RAM disk mount path')
  parser.add_argument('--backup-path', default='/home/rcr/robot_data', help='SD card backup path')
  parser.add_argument('--namespace', help='ROS namespace for multi-robot setups')
  parser.add_argument('--start', action='store_true', help='Start recording')
  parser.add_argument('--stop', action='store_true', help='Stop recording and backup')
  
  args = parser.parse_args()
  
  # Create recorder instance
  recorder = RamdiskRecorder(
    ramdisk_size=args.ramdisk_size,
    ramdisk_path=args.ramdisk_path,
    backup_path=args.backup_path,
    namespace=args.namespace
  )
  
  if args.start:
    return 0 if recorder.start_recording() else 1
  elif args.stop:
    return 0 if recorder.stop_recording() else 1
  else:
    # Interactive mode
    return 0 if recorder.run_interactive() else 1

if __name__ == "__main__":
  sys.exit(main())
