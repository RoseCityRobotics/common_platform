#!/usr/bin/env python3
"""
Script to prepare data recorder output for Label Studio annotation.

This script:
1. Copies images to a Label Studio-friendly directory structure
2. Creates a manifest file for easy import
3. Optionally filters images based on motion criteria
4. Prepares data for YOLO annotation workflow
"""

import os
import sys
import argparse
import pandas as pd
import shutil
from pathlib import Path
import json
import logging

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class LabelStudioPreparer:
  def __init__(self, session_dir: str):
    self.session_dir = Path(session_dir)
    self.images_dir = self.session_dir / "images"
    self.metadata_file = self.session_dir / "metadata" / "data_log.csv"
    
    if not self.session_dir.exists():
      raise ValueError(f"Session directory does not exist: {session_dir}")
    if not self.images_dir.exists():
      raise ValueError(f"Images directory does not exist: {self.images_dir}")
    if not self.metadata_file.exists():
      raise ValueError(f"Metadata file does not exist: {self.metadata_file}")
    
    # Load metadata
    self.df = pd.read_csv(self.metadata_file)
    logger.info(f"Loaded {len(self.df)} data points from {self.metadata_file}")
  
  def filter_by_motion(self, min_linear_vel: float = 0.01, min_angular_vel: float = 0.01):
    """Filter images to only include those with significant motion."""
    logger.info(f"Filtering images with motion > {min_linear_vel} m/s linear, {min_angular_vel} rad/s angular")
    
    # Filter for significant motion
    motion_mask = (
      (self.df['linear_vel'].abs() > min_linear_vel) |
      (self.df['angular_vel'].abs() > min_angular_vel)
    )
    
    filtered_df = self.df[motion_mask]
    logger.info(f"Filtered to {len(filtered_df)} images with significant motion")
    
    return filtered_df
  
  def prepare_for_labelstudio(self, output_dir: str, filter_motion: bool = True, 
                            min_linear_vel: float = 0.01, min_angular_vel: float = 0.01,
                            max_images: int = None):
    """Prepare images for Label Studio import."""
    
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Create subdirectories
    images_output = output_path / "images"
    images_output.mkdir(exist_ok=True)
    
    # Filter data if requested
    if filter_motion:
      df_to_process = self.filter_by_motion(min_linear_vel, min_angular_vel)
    else:
      df_to_process = self.df
    
    # Limit number of images if specified
    if max_images and len(df_to_process) > max_images:
      df_to_process = df_to_process.head(max_images)
      logger.info(f"Limited to {max_images} images")
    
    # Copy images and create manifest
    manifest_data = []
    
    for idx, row in df_to_process.iterrows():
      # Source and destination paths
      src_image = self.images_dir / row['image_file']
      dst_image = images_output / row['image_file']
      
      if src_image.exists():
        # Copy image
        shutil.copy2(src_image, dst_image)
        
        # Add to manifest
        manifest_entry = {
          'id': idx,
          'image_file': row['image_file'],
          'timestamp': row['timestamp'],
          'linear_vel': float(row['linear_vel']),
          'angular_vel': float(row['angular_vel']),
          'left_wheel_vel': float(row['left_wheel_vel']),
          'right_wheel_vel': float(row['right_wheel_vel']),
          'data': {
            'image': f'/data/local-files/?d=images/{row["image_file"]}'
          }
        }
        manifest_data.append(manifest_entry)
        
        if (idx + 1) % 100 == 0:
          logger.info(f"Processed {idx + 1} images...")
      else:
        logger.warning(f"Image not found: {src_image}")
    
    # Save manifest file
    manifest_file = output_path / "manifest.json"
    with open(manifest_file, 'w') as f:
      json.dump(manifest_data, f, indent=2)
    
    # Save filtered metadata
    metadata_file = output_path / "filtered_metadata.csv"
    df_to_process.to_csv(metadata_file, index=False)
    
    # Create Label Studio import instructions
    instructions_file = output_path / "labelstudio_instructions.md"
    with open(instructions_file, 'w') as f:
      f.write(f"""# Label Studio Import Instructions

## Dataset Information
- **Total Images**: {len(manifest_data)}
- **Source Session**: {self.session_dir.name}
- **Filtered for Motion**: {filter_motion}
- **Min Linear Velocity**: {min_linear_vel} m/s
- **Min Angular Velocity**: {min_angular_vel} rad/s

## Import Steps

1. **Open Label Studio** and create a new project
2. **Choose Template**: Select "Object Detection with Bounding Boxes" or "Custom"
3. **Import Data**: 
   - Go to Data Import
   - Upload the `manifest.json` file
   - Or upload all images from the `images/` directory

## Labeling Configuration

For YOLO format, use this Label Studio configuration:

```xml
<View>
  <Image name="image" value="$image" zoom="true"/>
  <RectangleLabels name="label" toName="image">
    <Label value="object" background="red"/>
    <Label value="obstacle" background="blue"/>
    <Label value="target" background="green"/>
  </RectangleLabels>
</View>
```

## Export for YOLO

After labeling:
1. Go to Data Manager
2. Select all tasks
3. Click "Export"
4. Choose "YOLO" format
5. Download the annotations

## Next Steps

After getting YOLO annotations:
1. Use `combine_yolo_with_motor_data.py` to merge annotations with motor commands
2. Train your YOLO model
3. Use the trained model for autonomous navigation

## Files in this directory:
- `images/` - All images for annotation
- `manifest.json` - Label Studio import file
- `filtered_metadata.csv` - Motor command data for each image
- `labelstudio_instructions.md` - This file
""")
    
    logger.info(f"Prepared {len(manifest_data)} images for Label Studio")
    logger.info(f"Output directory: {output_path}")
    logger.info(f"Manifest file: {manifest_file}")
    logger.info(f"Instructions: {instructions_file}")
    
    return output_path

def main():
  parser = argparse.ArgumentParser(description='Prepare data recorder output for Label Studio')
  parser.add_argument('session_dir', help='Path to session directory containing recorded data')
  parser.add_argument('--output-dir', help='Output directory for Label Studio data (default: session_dir/labelstudio)')
  parser.add_argument('--no-filter', action='store_true', help='Do not filter by motion')
  parser.add_argument('--min-linear-vel', type=float, default=0.01, help='Minimum linear velocity for motion filter (m/s)')
  parser.add_argument('--min-angular-vel', type=float, default=0.01, help='Minimum angular velocity for motion filter (rad/s)')
  parser.add_argument('--max-images', type=int, help='Maximum number of images to process')
  
  args = parser.parse_args()
  
  try:
    # Initialize preparer
    preparer = LabelStudioPreparer(args.session_dir)
    
    # Determine output directory
    if args.output_dir:
      output_dir = args.output_dir
    else:
      output_dir = str(Path(args.session_dir) / "labelstudio")
    
    # Prepare for Label Studio
    output_path = preparer.prepare_for_labelstudio(
      output_dir=output_dir,
      filter_motion=not args.no_filter,
      min_linear_vel=args.min_linear_vel,
      min_angular_vel=args.min_angular_vel,
      max_images=args.max_images
    )
    
    print(f"\n✅ Successfully prepared data for Label Studio!")
    print(f"📁 Output directory: {output_path}")
    print(f"📋 Next steps:")
    print(f"   1. Open Label Studio")
    print(f"   2. Import the manifest.json file")
    print(f"   3. Start annotating your images")
    print(f"   4. Export in YOLO format when done")
    
    return 0
    
  except Exception as e:
    logger.error(f"Error preparing data: {e}")
    return 1

if __name__ == "__main__":
  sys.exit(main())
