#!/usr/bin/env python3
"""
Manual backup script for RAM disk data.

This script copies data from the RAM disk to the SD card.
Use this if the automatic backup didn't work.
"""

import os
import sys
import shutil
from pathlib import Path
from datetime import datetime

def copy_with_progress(src, dst):
    """Copy files with progress bar."""
    src_path = Path(src)
    dst_path = Path(dst)
    
    if not src_path.exists():
        print(f"No data found at {src_path}")
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
        print("No files to copy")
        return True
    
    print(f"Copying {len(files_to_copy)} files ({total_size / (1024*1024):.1f} MB)...")
    
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
    print("✓ Copy completed successfully")
    return True

def main():
    ramdisk_path = "/mnt/recording_ramdisk/teleop_data"
    backup_path = "/home/rcr/robot_data"
    
    if not os.path.exists(ramdisk_path):
        print(f"RAM disk path not found: {ramdisk_path}")
        print("Make sure the RAM disk is mounted and contains data.")
        return 1
    
    # Find the session directory
    session_dirs = []
    for item in os.listdir(ramdisk_path):
        item_path = os.path.join(ramdisk_path, item)
        if os.path.isdir(item_path) and item.startswith("session_"):
            session_dirs.append(item_path)
    
    if not session_dirs:
        print("No session directories found")
        return 1
    
    # Use the most recent session
    latest_session = max(session_dirs, key=os.path.getctime)
    session_name = os.path.basename(latest_session)
    
    print(f"Found session: {session_name}")
    
    # Create backup directory with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_session_path = os.path.join(backup_path, f"{session_name}_backup_{timestamp}")
    
    print(f"Backing up to: {backup_session_path}")
    
    # Copy with progress bar
    success = copy_with_progress(latest_session, backup_session_path)
    
    if success:
        print(f"✓ Backup completed: {backup_session_path}")
        return 0
    else:
        print("✗ Backup failed")
        return 1

if __name__ == "__main__":
    sys.exit(main())

