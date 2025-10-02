#!/usr/bin/env python3
"""
Camera calibration script for creating camera_info and calibration data.
"""

import os
import sys
import subprocess
import time
from pathlib import Path

def create_calibration_directory():
  """Create directory for calibration files."""
  calib_dir = Path.home() / "camera_calibration"
  calib_dir.mkdir(exist_ok=True)
  return calib_dir

def download_calibration_pattern():
  """Download or create calibration pattern."""
  print("=== Camera Calibration Setup ===")
  print()
  
  calib_dir = create_calibration_directory()
  pattern_file = calib_dir / "calibration_pattern.pdf"
  
  print("1. Calibration Pattern:")
  print("   You need a checkerboard pattern for calibration.")
  print("   Recommended: 9x6 checkerboard with 25mm squares")
  print()
  print("   Download from:")
  print("   https://github.com/opencv/opencv/blob/master/doc/pattern.png")
  print()
  print("   Or create your own:")
  print("   - 9x6 checkerboard")
  print("   - 25mm square size")
  print("   - Print on A4 paper")
  print("   - Ensure flat surface")
  
  return calib_dir

def run_calibration():
  """Run camera calibration process."""
  print()
  print("2. Calibration Process:")
  print("   a) Start camera:")
  print("      ros2 launch sensors camera_libcamera.launch.py")
  print()
  print("   b) Start calibration node:")
  print("      ros2 run camera_calibration cameracalibrator \\")
  print("        --size 9x6 \\")
  print("        --square 0.025 \\")
  print("        image:=/camera/image_raw \\")
  print("        camera:=/camera")
  print()
  print("   c) Move checkerboard around:")
  print("      - Cover entire image")
  print("      - Different angles")
  print("      - Different distances")
  print("      - Wait for 'Calibrated' message")
  print()
  print("   d) Save calibration:")
  print("      - Click 'Save' button")
  print("      - Choose save location")
  print("      - File will be saved as YAML")

def create_sample_calibration():
  """Create a sample calibration file."""
  calib_dir = create_calibration_directory()
  sample_file = calib_dir / "sample_camera_calibration.yaml"
  
  sample_content = """# Sample camera calibration file
# Replace with your actual calibration data

image_width: 1280
image_height: 720
camera_name: camera
camera_matrix:
  rows: 3
  cols: 3
  data: [1000.0, 0.0, 640.0, 0.0, 1000.0, 360.0, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0, 0.0, 0.0, 0.0, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [1000.0, 0.0, 640.0, 0.0, 0.0, 1000.0, 360.0, 0.0, 0.0, 0.0, 1.0, 0.0]
"""
  
  with open(sample_file, 'w') as f:
    f.write(sample_content)
  
  print()
  print("3. Sample calibration file created:")
  print(f"   {sample_file}")
  print("   (Replace with your actual calibration data)")

def show_usage():
  """Show how to use calibration files."""
  print()
  print("4. Using Calibration Files:")
  print("   # With calibration file:")
  print("   ros2 launch sensors camera_calibration.launch.py \\")
  print("     calibration_file:=/path/to/calibration.yaml")
  print()
  print("   # With rectification:")
  print("   ros2 launch sensors camera_rectified.launch.py \\")
  print("     calibration_file:=/path/to/calibration.yaml")
  print()
  print("   # Topics available:")
  print("   /camera/image_raw          # Original image")
  print("   /camera/camera_info        # Camera parameters")
  print("   /camera/image_rect         # Rectified image")
  print("   /camera/image_rect/compressed  # Compressed rectified")

def main():
  """Run camera calibration setup."""
  print("Camera Calibration Setup")
  print("=" * 40)
  
  calib_dir = download_calibration_pattern()
  run_calibration()
  create_sample_calibration()
  show_usage()
  
  print()
  print("=== Setup Complete ===")
  print(f"Calibration files will be saved to: {calib_dir}")
  print("Follow the steps above to calibrate your camera.")
  
  return 0

if __name__ == "__main__":
  sys.exit(main())
