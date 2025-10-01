#!/usr/bin/env python3
"""
Script to combine YOLO annotations from Label Studio with motor command data.

This script:
1. Loads YOLO annotations from Label Studio export
2. Matches them with motor command data from data recorder
3. Creates training datasets for end-to-end learning
4. Supports both YOLO detection and motor command prediction
"""

import os
import sys
import argparse
import pandas as pd
import numpy as np
import json
import cv2
from pathlib import Path
import logging
from typing import Dict, List, Tuple

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class YOLOMotorCombiner:
  def __init__(self, session_dir: str, yolo_annotations_dir: str):
    self.session_dir = Path(session_dir)
    self.yolo_dir = Path(yolo_annotations_dir)
    
    # Paths
    self.images_dir = self.session_dir / "images"
    self.metadata_file = self.session_dir / "metadata" / "data_log.csv"
    
    # Validate paths
    if not self.session_dir.exists():
      raise ValueError(f"Session directory does not exist: {session_dir}")
    if not self.yolo_dir.exists():
      raise ValueError(f"YOLO annotations directory does not exist: {yolo_annotations_dir}")
    if not self.images_dir.exists():
      raise ValueError(f"Images directory does not exist: {self.images_dir}")
    if not self.metadata_file.exists():
      raise ValueError(f"Metadata file does not exist: {self.metadata_file}")
    
    # Load metadata
    self.df = pd.read_csv(self.metadata_file)
    logger.info(f"Loaded {len(self.df)} data points from {self.metadata_file}")
  
  def load_yolo_annotations(self) -> Dict[str, List[Dict]]:
    """Load YOLO annotations from Label Studio export."""
    annotations = {}
    
    # Look for YOLO format files
    yolo_files = list(self.yolo_dir.glob("*.txt"))
    if not yolo_files:
      # Look for Label Studio export files
      yolo_files = list(self.yolo_dir.glob("*.json"))
    
    if not yolo_files:
      raise ValueError(f"No YOLO annotation files found in {self.yolo_dir}")
    
    logger.info(f"Found {len(yolo_files)} annotation files")
    
    for yolo_file in yolo_files:
      if yolo_file.suffix == '.txt':
        # YOLO format: class_id center_x center_y width height
        image_name = yolo_file.stem + '.jpg'
        annotations[image_name] = []
        
        with open(yolo_file, 'r') as f:
          for line in f:
            parts = line.strip().split()
            if len(parts) >= 5:
              annotation = {
                'class_id': int(parts[0]),
                'center_x': float(parts[1]),
                'center_y': float(parts[2]),
                'width': float(parts[3]),
                'height': float(parts[4]),
                'confidence': float(parts[5]) if len(parts) > 5 else 1.0
              }
              annotations[image_name].append(annotation)
      
      elif yolo_file.suffix == '.json':
        # Label Studio export format
        with open(yolo_file, 'r') as f:
          data = json.load(f)
        
        for item in data:
          if 'data' in item and 'image' in item['data']:
            # Extract image filename
            image_path = item['data']['image']
            image_name = Path(image_path).name
            
            annotations[image_name] = []
            
            # Extract annotations
            if 'annotations' in item and item['annotations']:
              for annotation in item['annotations']:
                if 'result' in annotation:
                  for result in annotation['result']:
                    if result['type'] == 'rectanglelabels':
                      # Convert to YOLO format
                      x = result['value']['x'] / 100.0  # Convert percentage to decimal
                      y = result['value']['y'] / 100.0
                      w = result['value']['width'] / 100.0
                      h = result['value']['height'] / 100.0
                      
                      # Convert to center format
                      center_x = x + w / 2
                      center_y = y + h / 2
                      
                      # Get class ID (you may need to map class names to IDs)
                      class_name = result['value']['rectanglelabels'][0]
                      class_id = self._get_class_id(class_name)
                      
                      annotations[image_name].append({
                        'class_id': class_id,
                        'center_x': center_x,
                        'center_y': center_y,
                        'width': w,
                        'height': h,
                        'confidence': 1.0
                      })
    
    logger.info(f"Loaded annotations for {len(annotations)} images")
    return annotations
  
  def _get_class_id(self, class_name: str) -> int:
    """Map class names to YOLO class IDs."""
    class_mapping = {
      'object': 0,
      'obstacle': 1,
      'target': 2,
      'person': 3,
      'vehicle': 4,
    }
    return class_mapping.get(class_name.lower(), 0)
  
  def combine_data(self, output_dir: str) -> str:
    """Combine YOLO annotations with motor data."""
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Load YOLO annotations
    yolo_annotations = self.load_yolo_annotations()
    
    # Create combined dataset
    combined_data = []
    
    for _, row in self.df.iterrows():
      image_name = row['image_file']
      
      if image_name in yolo_annotations:
        # Get image dimensions
        image_path = self.images_dir / image_name
        if image_path.exists():
          image = cv2.imread(str(image_path))
          if image is not None:
            img_height, img_width = image.shape[:2]
            
            # Create data entry
            entry = {
              'image_file': image_name,
              'timestamp': row['timestamp'],
              'image_width': img_width,
              'image_height': img_height,
              'linear_vel': row['linear_vel'],
              'angular_vel': row['angular_vel'],
              'left_wheel_vel': row['left_wheel_vel'],
              'right_wheel_vel': row['right_wheel_vel'],
              'num_objects': len(yolo_annotations[image_name]),
              'annotations': yolo_annotations[image_name]
            }
            combined_data.append(entry)
    
    logger.info(f"Combined data for {len(combined_data)} images")
    
    # Save combined dataset
    combined_file = output_path / "combined_dataset.json"
    with open(combined_file, 'w') as f:
      json.dump(combined_data, f, indent=2)
    
    # Create YOLO format files
    yolo_output_dir = output_path / "yolo_format"
    yolo_output_dir.mkdir(exist_ok=True)
    
    # Copy images and create annotation files
    images_output_dir = yolo_output_dir / "images"
    images_output_dir.mkdir(exist_ok=True)
    
    labels_output_dir = yolo_output_dir / "labels"
    labels_output_dir.mkdir(exist_ok=True)
    
    for entry in combined_data:
      # Copy image
      src_image = self.images_dir / entry['image_file']
      dst_image = images_output_dir / entry['image_file']
      if src_image.exists():
        import shutil
        shutil.copy2(src_image, dst_image)
      
      # Create YOLO annotation file
      annotation_file = labels_output_dir / (Path(entry['image_file']).stem + '.txt')
      with open(annotation_file, 'w') as f:
        for ann in entry['annotations']:
          f.write(f"{ann['class_id']} {ann['center_x']} {ann['center_y']} {ann['width']} {ann['height']}\n")
    
    # Create dataset configuration
    config_file = output_path / "dataset_config.yaml"
    with open(config_file, 'w') as f:
      f.write(f"""# YOLO Dataset Configuration
# Generated from data recorder session: {self.session_dir.name}

# Dataset paths
train: {yolo_output_dir}/images
val: {yolo_output_dir}/images  # You may want to split this

# Number of classes
nc: 5

# Class names
names:
  0: object
  1: obstacle
  2: target
  3: person
  4: vehicle

# Motor command data
motor_commands:
  linear_vel: true
  angular_vel: true
  left_wheel_vel: true
  right_wheel_vel: true
""")
    
    # Create training script template
    training_script = output_path / "train_yolo.py"
    with open(training_script, 'w') as f:
      f.write(f'''#!/usr/bin/env python3
"""
YOLO training script for combined detection and motor command prediction.
"""

import torch
from ultralytics import YOLO
import json
import cv2
import numpy as np
from pathlib import Path

def train_yolo_model():
    # Load YOLO model
    model = YOLO('yolov8n.pt')  # or yolov8s.pt, yolov8m.pt, etc.
    
    # Train the model
    results = model.train(
        data='{config_file}',  # Path to dataset config
        epochs=100,
        imgsz=640,
        batch=16,
        device='cuda' if torch.cuda.is_available() else 'cpu'
    )
    
    # Save the trained model
    model.save('trained_yolo_model.pt')
    
    return model

def predict_with_motor_commands(model, image_path):
    """Predict objects and motor commands from an image."""
    # Run YOLO detection
    results = model(image_path)
    
    # Extract detections
    detections = results[0].boxes
    
    # You can add motor command prediction here
    # based on the detected objects
    
    return results

if __name__ == "__main__":
    model = train_yolo_model()
    print("Training completed!")
''')
    
    # Create summary
    summary_file = output_path / "dataset_summary.txt"
    with open(summary_file, 'w') as f:
      f.write(f"""Dataset Summary
==============

Source Session: {self.session_dir.name}
Total Images: {len(combined_data)}
Images with Annotations: {len([e for e in combined_data if e['num_objects'] > 0])}
Total Objects: {sum(e['num_objects'] for e in combined_data)}

Motor Command Statistics:
- Linear Velocity: {np.mean([e['linear_vel'] for e in combined_data]):.3f} ± {np.std([e['linear_vel'] for e in combined_data]):.3f} m/s
- Angular Velocity: {np.mean([e['angular_vel'] for e in combined_data]):.3f} ± {np.std([e['angular_vel'] for e in combined_data]):.3f} rad/s

Files Created:
- combined_dataset.json: Complete dataset with annotations and motor data
- yolo_format/: YOLO-compatible directory structure
- dataset_config.yaml: YOLO training configuration
- train_yolo.py: Training script template
- dataset_summary.txt: This summary

Next Steps:
1. Review the dataset_config.yaml file
2. Split data into train/validation sets if needed
3. Run the training script: python train_yolo.py
4. Test the trained model on new images
""")
    
    logger.info(f"Combined dataset created in: {output_path}")
    logger.info(f"YOLO format files in: {yolo_output_dir}")
    logger.info(f"Dataset config: {config_file}")
    
    return str(output_path)

def main():
  parser = argparse.ArgumentParser(description='Combine YOLO annotations with motor data')
  parser.add_argument('session_dir', help='Path to session directory containing recorded data')
  parser.add_argument('yolo_annotations_dir', help='Path to directory containing YOLO annotations from Label Studio')
  parser.add_argument('--output-dir', help='Output directory for combined dataset (default: session_dir/combined_yolo)')
  
  args = parser.parse_args()
  
  try:
    # Initialize combiner
    combiner = YOLOMotorCombiner(args.session_dir, args.yolo_annotations_dir)
    
    # Determine output directory
    if args.output_dir:
      output_dir = args.output_dir
    else:
      output_dir = str(Path(args.session_dir) / "combined_yolo")
    
    # Combine data
    output_path = combiner.combine_data(output_dir)
    
    print(f"\n✅ Successfully combined YOLO annotations with motor data!")
    print(f"📁 Output directory: {output_path}")
    print(f"📋 Next steps:")
    print(f"   1. Review dataset_config.yaml")
    print(f"   2. Run: python train_yolo.py")
    print(f"   3. Test your trained YOLO model")
    
    return 0
    
  except Exception as e:
    logger.error(f"Error combining data: {e}")
    return 1

if __name__ == "__main__":
  sys.exit(main())
