import os
import pandas as pd
import numpy as np
import torch
from torch.utils.data import Dataset
from PIL import Image
import cv2
from typing import List, Tuple, Optional, Dict, Any
import random
from pathlib import Path


class MazeNavigationDataset(Dataset):
  """
  Dataset for maze navigation imitation learning.
  
  Loads sequences of camera frames and corresponding action chunks from
  data recorder sessions. Each sample contains:
  - N consecutive frames (temporal window)
  - K future actions (action chunk) for each frame
  """
  
  def __init__(
    self,
    session_dirs: List[str],
    sequence_length: int = 10,
    chunk_size: int = 15,
    stride: int = 1,
    image_size: Tuple[int, int] = (224, 224),
    augment: bool = True,
    action_stats: Optional[Dict[str, float]] = None,
    split: str = 'train'
  ):
    """
    Initialize dataset.
    
    Args:
      session_dirs: List of session directory paths
      sequence_length: Number of consecutive frames (N)
      chunk_size: Number of future actions to predict (K)
      stride: Step size for sliding window
      image_size: Target image size (height, width)
      augment: Whether to apply data augmentation
      action_stats: Action normalization statistics (mean, std)
      split: Dataset split ('train', 'val', 'test')
    """
    self.session_dirs = session_dirs
    self.sequence_length = sequence_length
    self.chunk_size = chunk_size
    self.stride = stride
    self.image_size = image_size
    self.augment = augment and (split == 'train')
    self.split = split
    
    # Load lightweight session metadata (no images in RAM)
    self.sessions = self._load_all_sessions_compact()
    
    # Compute action statistics if not provided
    if action_stats is None:
      self.action_stats = self._compute_action_stats_compact()
    else:
      self.action_stats = action_stats
    
    # Create sequence indices
    self.sequence_indices = self._create_sequence_indices_compact()
    
    print(f"Loaded {len(self.sequence_indices)} sequences from {len(self.sessions)} sessions")
    print(f"Action stats - linear: mean={self.action_stats['linear_mean']:.3f}, std={self.action_stats['linear_std']:.3f}")
    print(f"Action stats - angular: mean={self.action_stats['angular_mean']:.3f}, std={self.action_stats['angular_std']:.3f}")
  
  def _load_all_sessions_compact(self) -> List[Dict[str, Any]]:
    """Load per-session compact metadata (filenames and actions only)."""
    sessions = []
    for session_dir in self.session_dirs:
      if not os.path.exists(session_dir):
        print(f"Warning: Session directory not found: {session_dir}")
        continue
      csv_path = os.path.join(session_dir, 'metadata', 'data_log.csv')
      images_dir = os.path.join(session_dir, 'images')
      if not os.path.exists(csv_path) or not os.path.exists(images_dir):
        print(f"Warning: Missing data in session: {session_dir}")
        continue
      # Load only required columns to minimize RAM
      df = pd.read_csv(csv_path, usecols=['image_file', 'cmd_linear_x', 'cmd_angular_z'])
      df = df.dropna(subset=['image_file', 'cmd_linear_x', 'cmd_angular_z'])
      if len(df) == 0:
        print(f"Warning: No valid rows in {csv_path}")
        continue
      image_files = df['image_file'].tolist()
      # Build absolute paths lazily in __getitem__ to keep small
      cmd_linear = df['cmd_linear_x'].to_numpy(dtype=np.float32)
      cmd_angular = df['cmd_angular_z'].to_numpy(dtype=np.float32)
      sessions.append({
        'session_dir': session_dir,
        'images_dir': images_dir,
        'image_files': image_files,
        'cmd_linear_x': cmd_linear,
        'cmd_angular_z': cmd_angular,
        'length': len(image_files)
      })
      print(f"Indexed {len(image_files)} samples from {session_dir}")
    return sessions
  
  def _compute_action_stats_compact(self) -> Dict[str, float]:
    """Compute normalization stats from compact per-session arrays."""
    if not self.sessions:
      return {'linear_mean': 0.0, 'linear_std': 1.0, 'angular_mean': 0.0, 'angular_std': 1.0}
    linear_actions = np.concatenate([s['cmd_linear_x'] for s in self.sessions])
    angular_actions = np.concatenate([s['cmd_angular_z'] for s in self.sessions])
    # Avoid zero std
    linear_std = float(np.std(linear_actions)) or 1.0
    angular_std = float(np.std(angular_actions)) or 1.0
    return {
      'linear_mean': float(np.mean(linear_actions)),
      'linear_std': linear_std,
      'angular_mean': float(np.mean(angular_actions)),
      'angular_std': angular_std
    }
  
  def _create_sequence_indices_compact(self) -> List[Tuple[int, int]]:
    """Create indices as (session_idx, start_idx) for valid sequences."""
    indices: List[Tuple[int, int]] = []
    for s_idx, session in enumerate(self.sessions):
      total = session['length']
      min_needed = self.sequence_length + self.chunk_size
      if total < min_needed:
        continue
      for start_idx in range(0, total - min_needed + 1, self.stride):
        indices.append((s_idx, start_idx))
    return indices
  
  def _load_image(self, image_path: str) -> np.ndarray:
    """Load and preprocess image."""
    try:
      # Load image
      image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
      if image is None:
        raise ValueError(f"Could not load image: {image_path}")
      
      # Normalize channel layout to RGB
      # OpenCV returns BGR for 3-channel JPEGs by default
      if len(image.shape) == 2:
        # Grayscale -> RGB
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
      elif image.shape[2] == 3:
        # BGR -> RGB
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
      elif image.shape[2] == 4:
        # BGRA -> RGB
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)
      
      # Resize image
      image = cv2.resize(image, (self.image_size[1], self.image_size[0]))
      
      # Normalize to [0, 1]
      image = image.astype(np.float32) / 255.0
      
      return image
    except Exception as e:
      print(f"Error loading image {image_path}: {e}")
      # Return zero image as fallback
      return np.zeros((self.image_size[0], self.image_size[1], 3), dtype=np.float32)
  
  def _augment_image(self, image: np.ndarray) -> np.ndarray:
    """Apply data augmentation to image."""
    if not self.augment:
      return image
    
    # Random horizontal flip
    if random.random() < 0.5:
      image = np.fliplr(image)
    
    # Random brightness adjustment
    if random.random() < 0.3:
      brightness_factor = random.uniform(0.8, 1.2)
      image = np.clip(image * brightness_factor, 0, 1)
    
    # Random contrast adjustment
    if random.random() < 0.3:
      contrast_factor = random.uniform(0.8, 1.2)
      mean = np.mean(image)
      image = np.clip((image - mean) * contrast_factor + mean, 0, 1)
    
    # Random color jitter
    if random.random() < 0.3:
      # Slight hue shift
      hsv = cv2.cvtColor((image * 255).astype(np.uint8), cv2.COLOR_RGB2HSV)
      hsv[:, :, 0] = (hsv[:, :, 0] + random.randint(-10, 10)) % 180
      image = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB).astype(np.float32) / 255.0
    
    return image
  
  def _normalize_actions(self, actions: np.ndarray) -> np.ndarray:
    """Normalize actions using computed statistics."""
    normalized = actions.copy()
    normalized[:, 0] = (normalized[:, 0] - self.action_stats['linear_mean']) / self.action_stats['linear_std']
    normalized[:, 1] = (normalized[:, 1] - self.action_stats['angular_mean']) / self.action_stats['angular_std']
    return normalized
  
  def _denormalize_actions(self, normalized_actions: np.ndarray) -> np.ndarray:
    """Denormalize actions back to original scale."""
    denormalized = normalized_actions.copy()
    denormalized[:, 0] = denormalized[:, 0] * self.action_stats['linear_std'] + self.action_stats['linear_mean']
    denormalized[:, 1] = denormalized[:, 1] * self.action_stats['angular_std'] + self.action_stats['angular_mean']
    return denormalized
  
  def __len__(self) -> int:
    return len(self.sequence_indices)
  
  def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
    """
    Get a sequence sample.
    
    Returns:
      images: (sequence_length, 3, height, width) - sequence of frames
      actions: (sequence_length, chunk_size, 2) - action chunks for each frame
    """
    session_idx, start_idx = self.sequence_indices[idx]
    session = self.sessions[session_idx]
    end_idx = start_idx + self.sequence_length
    
    # Decide sample-level horizontal flip once for the whole sequence
    do_hflip = self.augment and (random.random() < 0.5)
    
    # Load image sequence
    images = []
    for i in range(start_idx, end_idx):
      image_path = os.path.join(session['images_dir'], session['image_files'][i])
      image = self._load_image(image_path)
      # Apply non-geometric photometric augs per frame
      image = self._augment_image(image)
      if do_hflip:
        image = np.fliplr(image)
      images.append(image)
    
    # Build action chunks
    actions = []
    for i in range(start_idx, end_idx):
      future_end = min(i + self.chunk_size, session['length'])
      linear = session['cmd_linear_x'][i:future_end]
      angular = session['cmd_angular_z'][i:future_end]
      if do_hflip:
        # Negate angular velocity when horizontally flipping images
        angular = -angular
      frame_actions = np.stack([linear, angular], axis=1).astype(np.float32)
      # Pad if needed
      if frame_actions.shape[0] < self.chunk_size:
        pad_count = self.chunk_size - frame_actions.shape[0]
        pad_vals = np.repeat(frame_actions[-1:], pad_count, axis=0) if frame_actions.shape[0] > 0 else np.zeros((pad_count, 2), dtype=np.float32)
        frame_actions = np.concatenate([frame_actions, pad_vals], axis=0)
      actions.append(frame_actions)
    
    # Convert to numpy arrays
    images = np.array(images)  # (sequence_length, height, width, 3)
    actions = np.array(actions)  # (sequence_length, chunk_size, 2)
    
    # Normalize actions
    actions = self._normalize_actions(actions.reshape(-1, 2)).reshape(actions.shape)
    
    # Convert to tensors and rearrange dimensions
    images = torch.from_numpy(images).permute(0, 3, 1, 2)  # (sequence_length, 3, H, W)
    actions = torch.from_numpy(actions)  # (sequence_length, chunk_size, 2)
    
    return images, actions
  
  def get_action_stats(self) -> Dict[str, float]:
    """Get action normalization statistics."""
    return self.action_stats.copy()
  
  def set_action_stats(self, stats: Dict[str, float]):
    """Set action normalization statistics."""
    self.action_stats = stats.copy()


def create_datasets(
  data_root: str,
  train_ratio: float = 0.8,
  val_ratio: float = 0.1,
  test_ratio: float = 0.1,
  **kwargs
) -> Tuple[MazeNavigationDataset, MazeNavigationDataset, MazeNavigationDataset]:
  """
  Create train/val/test datasets from session directories.
  
  Args:
    data_root: Root directory containing session directories
    train_ratio: Fraction of data for training
    val_ratio: Fraction of data for validation
    test_ratio: Fraction of data for testing
    **kwargs: Additional arguments passed to MazeNavigationDataset
  
  Returns:
    Tuple of (train_dataset, val_dataset, test_dataset)
  """
  # Find all session directories
  session_dirs = []
  for item in os.listdir(data_root):
    session_path = os.path.join(data_root, item)
    if os.path.isdir(session_path) and item.startswith('session_'):
      session_dirs.append(session_path)
  
  session_dirs.sort()  # Ensure consistent ordering
  
  if len(session_dirs) == 0:
    raise ValueError(f"No session directories found in {data_root}")
  
  # Split sessions
  n_sessions = len(session_dirs)
  n_train = int(n_sessions * train_ratio)
  n_val = int(n_sessions * val_ratio)
  
  train_sessions = session_dirs[:n_train]
  val_sessions = session_dirs[n_train:n_train + n_val]
  test_sessions = session_dirs[n_train + n_val:]
  
  print(f"Dataset split: {len(train_sessions)} train, {len(val_sessions)} val, {len(test_sessions)} test sessions")
  
  # Create datasets
  train_dataset = MazeNavigationDataset(train_sessions, split='train', **kwargs)
  
  # Use train dataset's action stats for val/test to ensure consistent normalization
  val_dataset = MazeNavigationDataset(val_sessions, split='val', action_stats=train_dataset.get_action_stats(), **kwargs)
  test_dataset = MazeNavigationDataset(test_sessions, split='test', action_stats=train_dataset.get_action_stats(), **kwargs)
  
  return train_dataset, val_dataset, test_dataset