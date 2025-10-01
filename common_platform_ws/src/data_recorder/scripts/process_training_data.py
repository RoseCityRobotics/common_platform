#!/usr/bin/env python3
"""
Data processing script for teleoperation training data.

This script processes recorded teleoperation data into formats suitable for
neural network training. It can convert the data to various formats including
HDF5, TFRecord, or simple numpy arrays.

Usage:
    python3 process_training_data.py <session_directory> [options]
"""

import os
import sys
import argparse
import pandas as pd
import numpy as np
import cv2
from pathlib import Path
import h5py
import json
from typing import Tuple, List, Dict
import logging

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class TrainingDataProcessor:
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
  
  def validate_data(self) -> bool:
    """Validate that all referenced images exist and data is consistent."""
    logger.info("Validating data...")
    
    missing_images = []
    for _, row in self.df.iterrows():
      image_path = self.images_dir / row['image_file']
      if not image_path.exists():
        missing_images.append(row['image_file'])
    
    if missing_images:
      logger.error(f"Missing {len(missing_images)} images")
      return False
    
    # Check for reasonable velocity ranges
    max_linear = self.df['linear_vel'].abs().max()
    max_angular = self.df['angular_vel'].abs().max()
    
    if max_linear > 5.0:  # 5 m/s seems too fast for most robots
      logger.warning(f"Very high linear velocity detected: {max_linear} m/s")
    
    if max_angular > 10.0:  # 10 rad/s seems too fast
      logger.warning(f"Very high angular velocity detected: {max_angular} rad/s")
    
    logger.info("Data validation completed successfully")
    return True
  
  def load_images_and_labels(self, max_samples: int = None) -> Tuple[np.ndarray, np.ndarray]:
    """Load images and corresponding labels."""
    logger.info("Loading images and labels...")
    
    # Limit samples if specified
    df_subset = self.df if max_samples is None else self.df.head(max_samples)
    
    images = []
    labels = []
    
    for idx, row in df_subset.iterrows():
      # Load image
      image_path = self.images_dir / row['image_file']
      image = cv2.imread(str(image_path))
      if image is None:
        logger.warning(f"Could not load image: {image_path}")
        continue
      
      # Convert BGR to RGB
      image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
      images.append(image)
      
      # Create label vector [linear_vel, angular_vel, left_wheel_vel, right_wheel_vel]
      label = np.array([
        row['linear_vel'],
        row['angular_vel'],
        row['left_wheel_vel'],
        row['right_wheel_vel']
      ])
      labels.append(label)
      
      if (idx + 1) % 100 == 0:
        logger.info(f"Loaded {idx + 1} samples...")
    
    images = np.array(images)
    labels = np.array(labels)
    
    logger.info(f"Loaded {len(images)} images with shape {images.shape}")
    logger.info(f"Loaded {len(labels)} labels with shape {labels.shape}")
    
    return images, labels
  
  def save_to_hdf5(self, output_file: str, images: np.ndarray, labels: np.ndarray):
    """Save data to HDF5 format."""
    logger.info(f"Saving data to HDF5: {output_file}")
    
    with h5py.File(output_file, 'w') as f:
      f.create_dataset('images', data=images, compression='gzip')
      f.create_dataset('labels', data=labels, compression='gzip')
      
      # Add metadata
      f.attrs['num_samples'] = len(images)
      f.attrs['image_shape'] = images.shape[1:]
      f.attrs['label_shape'] = labels.shape[1:]
      f.attrs['session_dir'] = str(self.session_dir)
      
      # Add column names for labels
      f.create_dataset('label_names', data=[
        b'linear_vel', b'angular_vel', b'left_wheel_vel', b'right_wheel_vel'
      ])
    
    logger.info(f"Saved {len(images)} samples to {output_file}")
  
  def save_to_numpy(self, output_dir: str, images: np.ndarray, labels: np.ndarray):
    """Save data as numpy arrays."""
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    logger.info(f"Saving data to numpy format in: {output_dir}")
    
    # Save arrays
    np.save(output_path / "images.npy", images)
    np.save(output_path / "labels.npy", labels)
    
    # Save metadata
    metadata = {
      'num_samples': len(images),
      'image_shape': images.shape[1:],
      'label_shape': labels.shape[1:],
      'session_dir': str(self.session_dir),
      'label_names': ['linear_vel', 'angular_vel', 'left_wheel_vel', 'right_wheel_vel']
    }
    
    with open(output_path / "metadata.json", 'w') as f:
      json.dump(metadata, f, indent=2)
    
    logger.info(f"Saved {len(images)} samples to {output_dir}")
  
  def create_tfrecord_script(self, output_file: str):
    """Create a script to convert data to TFRecord format."""
    script_content = f'''#!/usr/bin/env python3
"""
Auto-generated script to create TFRecord from teleoperation data.
Run this script to convert your data to TFRecord format for TensorFlow training.
"""

import tensorflow as tf
import numpy as np
import json
from pathlib import Path

def _bytes_feature(value):
  """Returns a bytes_list from a string / byte."""
  if isinstance(value, type(tf.constant(0))):
    value = value.numpy()
  return tf.train.Feature(bytes_list=tf.train.BytesList(value=[value]))

def _float_feature(value):
  """Returns a float_list from a float / double."""
  return tf.train.Feature(float_list=tf.train.FloatList(value=[value]))

def _int64_feature(value):
  """Returns an int64_list from a bool / enum / int / uint."""
  return tf.train.Feature(int64_list=tf.train.Int64List(value=[value]))

def create_tfrecord():
  # Load data
  images = np.load("{self.session_dir}/processed/images.npy")
  labels = np.load("{self.session_dir}/processed/labels.npy")
  
  with open("{self.session_dir}/processed/metadata.json", 'r') as f:
    metadata = json.load(f)
  
  # Create TFRecord
  output_file = "{output_file}"
  with tf.io.TFRecordWriter(output_file) as writer:
    for i in range(len(images)):
      # Serialize image
      image_bytes = tf.io.encode_png(images[i]).numpy()
      
      # Create example
      feature = {{
        'image': _bytes_feature(image_bytes),
        'linear_vel': _float_feature(labels[i][0]),
        'angular_vel': _float_feature(labels[i][1]),
        'left_wheel_vel': _float_feature(labels[i][2]),
        'right_wheel_vel': _float_feature(labels[i][3]),
      }}
      
      example = tf.train.Example(features=tf.train.Features(feature=feature))
      writer.write(example.SerializeToString())
  
  print(f"Created TFRecord with {{len(images)}} samples: {{output_file}}")

if __name__ == "__main__":
  create_tfrecord()
'''
    
    script_path = Path(output_file)
    script_path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(script_path, 'w') as f:
      f.write(script_content)
    
    # Make executable
    os.chmod(script_path, 0o755)
    
    logger.info(f"Created TFRecord conversion script: {output_file}")
  
  def generate_statistics(self) -> Dict:
    """Generate statistics about the dataset."""
    stats = {
      'total_samples': len(self.df),
      'linear_vel_stats': {
        'mean': float(self.df['linear_vel'].mean()),
        'std': float(self.df['linear_vel'].std()),
        'min': float(self.df['linear_vel'].min()),
        'max': float(self.df['linear_vel'].max()),
      },
      'angular_vel_stats': {
        'mean': float(self.df['angular_vel'].mean()),
        'std': float(self.df['angular_vel'].std()),
        'min': float(self.df['angular_vel'].min()),
        'max': float(self.df['angular_vel'].max()),
      },
      'left_wheel_vel_stats': {
        'mean': float(self.df['left_wheel_vel'].mean()),
        'std': float(self.df['left_wheel_vel'].std()),
        'min': float(self.df['left_wheel_vel'].min()),
        'max': float(self.df['left_wheel_vel'].max()),
      },
      'right_wheel_vel_stats': {
        'mean': float(self.df['right_wheel_vel'].mean()),
        'std': float(self.df['right_wheel_vel'].std()),
        'min': float(self.df['right_wheel_vel'].min()),
        'max': float(self.df['right_wheel_vel'].max()),
      }
    }
    
    return stats

def main():
  parser = argparse.ArgumentParser(description='Process teleoperation training data')
  parser.add_argument('session_dir', help='Path to session directory containing recorded data')
  parser.add_argument('--output-format', choices=['hdf5', 'numpy', 'both'], default='both',
                     help='Output format for processed data')
  parser.add_argument('--max-samples', type=int, help='Maximum number of samples to process')
  parser.add_argument('--output-dir', help='Output directory (default: session_dir/processed)')
  parser.add_argument('--validate-only', action='store_true', help='Only validate data, do not process')
  
  args = parser.parse_args()
  
  try:
    # Initialize processor
    processor = TrainingDataProcessor(args.session_dir)
    
    # Validate data
    if not processor.validate_data():
      logger.error("Data validation failed")
      return 1
    
    if args.validate_only:
      logger.info("Validation completed successfully")
      return 0
    
    # Generate statistics
    stats = processor.generate_statistics()
    logger.info("Dataset statistics:")
    logger.info(f"  Total samples: {stats['total_samples']}")
    logger.info(f"  Linear velocity: mean={stats['linear_vel_stats']['mean']:.3f}, "
               f"std={stats['linear_vel_stats']['std']:.3f}")
    logger.info(f"  Angular velocity: mean={stats['angular_vel_stats']['mean']:.3f}, "
               f"std={stats['angular_vel_stats']['std']:.3f}")
    
    # Load data
    images, labels = processor.load_images_and_labels(max_samples=args.max_samples)
    
    # Determine output directory
    if args.output_dir:
      output_dir = args.output_dir
    else:
      output_dir = str(Path(args.session_dir) / "processed")
    
    # Process data
    if args.output_format in ['hdf5', 'both']:
      hdf5_file = Path(output_dir) / "training_data.h5"
      processor.save_to_hdf5(str(hdf5_file), images, labels)
    
    if args.output_format in ['numpy', 'both']:
      processor.save_to_numpy(output_dir, images, labels)
    
    # Create TFRecord conversion script
    tfrecord_script = Path(output_dir) / "create_tfrecord.py"
    processor.create_tfrecord_script(str(tfrecord_script))
    
    # Save statistics
    stats_file = Path(output_dir) / "statistics.json"
    with open(stats_file, 'w') as f:
      json.dump(stats, f, indent=2)
    
    logger.info(f"Processing completed successfully. Output saved to: {output_dir}")
    return 0
    
  except Exception as e:
    logger.error(f"Error processing data: {e}")
    return 1

if __name__ == "__main__":
  sys.exit(main())
