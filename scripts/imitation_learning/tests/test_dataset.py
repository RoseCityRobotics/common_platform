"""
Tests for the dataset module.

This module tests:
- Dataset initialization
- Session loading
- Action statistics computation
- Sequence index creation
- Image loading and preprocessing
- Data augmentation (especially horizontal flip)
- Action chunking and normalization
- Edge cases and error handling
"""

import os
import sys
import tempfile
import shutil
import pytest
import torch
import numpy as np
import pandas as pd
import cv2
from pathlib import Path

# Add src to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from imitation_learning.dataset import (
  MazeNavigationDataset,
  create_datasets
)


@pytest.fixture
def temp_dir():
  """Create a temporary directory for tests."""
  tmpdir = tempfile.mkdtemp()
  yield tmpdir
  shutil.rmtree(tmpdir)


@pytest.fixture
def mock_session_dir(temp_dir):
  """Create a mock session directory with sample data."""
  session_dir = os.path.join(temp_dir, "session_20240101_120000")
  images_dir = os.path.join(session_dir, "images")
  metadata_dir = os.path.join(session_dir, "metadata")
  
  os.makedirs(images_dir)
  os.makedirs(metadata_dir)
  
  # Create mock CSV data with realistic values
  n_samples = 100
  csv_data = {
    'timestamp': range(n_samples),
    'image_file': [f'{i:08d}.jpg' for i in range(n_samples)],
    'linear_vel': np.random.randn(n_samples).astype(float) * 0.1,
    'angular_vel': np.random.randn(n_samples).astype(float) * 0.5,
    'position_x': np.cumsum(np.random.randn(n_samples).astype(float) * 0.1),
    'position_y': np.cumsum(np.random.randn(n_samples).astype(float) * 0.1),
    'orientation_z': np.cumsum(np.random.randn(n_samples).astype(float) * 0.1),
    'cmd_linear_x': np.random.uniform(-0.3, 0.3, n_samples).astype(float),
    'cmd_angular_z': np.random.uniform(-1.0, 1.0, n_samples).astype(float),
  }
  
  df = pd.DataFrame(csv_data)
  df.to_csv(os.path.join(metadata_dir, "data_log.csv"), index=False)
  
  # Create dummy image files (small colored images to test color conversion)
  for i in range(n_samples):
    # Create images with different colors to verify loading
    img = np.ones((480, 640, 3), dtype=np.uint8) * (i % 255)
    cv2.imwrite(os.path.join(images_dir, f'{i:08d}.jpg'), img)
  
  return session_dir


@pytest.fixture
def multiple_session_dirs(temp_dir):
  """Create multiple mock session directories."""
  session_dirs = []
  for i in range(3):
    session_dir = os.path.join(temp_dir, f"session_20240101_12000{i}")
    images_dir = os.path.join(session_dir, "images")
    metadata_dir = os.path.join(session_dir, "metadata")
    
    os.makedirs(images_dir)
    os.makedirs(metadata_dir)
    
    n_samples = 100  # Same as mock_session_dir for consistency
    csv_data = {
      'timestamp': range(n_samples),
      'image_file': [f'{j:08d}.jpg' for j in range(n_samples)],
      'linear_vel': np.random.randn(n_samples).astype(float) * 0.1,
      'angular_vel': np.random.randn(n_samples).astype(float) * 0.5,
      'position_x': np.random.randn(n_samples).astype(float),
      'position_y': np.random.randn(n_samples).astype(float),
      'orientation_z': np.random.randn(n_samples).astype(float),
      'cmd_linear_x': np.random.uniform(-0.3, 0.3, n_samples).astype(float),
      'cmd_angular_z': np.random.uniform(-1.0, 1.0, n_samples).astype(float),
    }
    
    df = pd.DataFrame(csv_data)
    df.to_csv(os.path.join(metadata_dir, "data_log.csv"), index=False)
    
    for j in range(n_samples):
      img = np.zeros((480, 640, 3), dtype=np.uint8)
      cv2.imwrite(os.path.join(images_dir, f'{j:08d}.jpg'), img)
    
    session_dirs.append(session_dir)
  
  return session_dirs


class TestDatasetInitialization:
  """Test dataset initialization and setup."""
  
  def test_dataset_init(self, mock_session_dir):
    """Test basic dataset initialization."""
    dataset = MazeNavigationDataset(
      session_dirs=[mock_session_dir],
      sequence_length=10,
      chunk_size=15,
      augment=False
    )
    
    assert len(dataset.sessions) == 1
    assert dataset.sequence_length == 10
    assert dataset.chunk_size == 15
    assert dataset.image_size == (224, 224)  # Target resize size
    
    # Verify that the actual image size from the session matches the mock images
    # Mock images are created as 480x640 (height x width)
    session = dataset.sessions[0]
    assert 'image_size' in session
    assert session['image_size'] == (480, 640), f"Expected (480, 640), got {session['image_size']}"
  
  def test_dataset_loads_sessions(self, mock_session_dir):
    """Test that dataset loads session data correctly."""
    dataset = MazeNavigationDataset(
      session_dirs=[mock_session_dir],
      sequence_length=10,
      chunk_size=15,
      augment=False
    )
    
    assert len(dataset.sessions) == 1
    session = dataset.sessions[0]
    assert session['length'] == 100
    assert len(session['image_files']) == 100
    assert len(session['cmd_linear_x']) == 100
    assert len(session['cmd_angular_z']) == 100
  
  def test_dataset_with_multiple_sessions(self, multiple_session_dirs):
    """Test dataset with multiple session directories."""
    dataset = MazeNavigationDataset(
      session_dirs=multiple_session_dirs,
      sequence_length=10,
      chunk_size=15,
      augment=False
    )
    
    assert len(dataset.sessions) == 3
    for session in dataset.sessions:
      assert session['length'] == 100  # Updated to match n_samples in fixture
  
  def test_dataset_handles_missing_session(self, temp_dir):
    """Test dataset handles missing session directories gracefully."""
    missing_dir = os.path.join(temp_dir, "nonexistent_session")
    
    dataset = MazeNavigationDataset(
      session_dirs=[missing_dir],
      sequence_length=10,
      chunk_size=15,
      augment=False
    )
    
    assert len(dataset.sessions) == 0
    assert len(dataset) == 0
  
  def test_dataset_handles_missing_csv(self, temp_dir):
    """Test dataset handles missing CSV files gracefully."""
    session_dir = os.path.join(temp_dir, "session_no_csv")
    images_dir = os.path.join(session_dir, "images")
    os.makedirs(images_dir)
    
    dataset = MazeNavigationDataset(
      session_dirs=[session_dir],
      sequence_length=10,
      chunk_size=15,
      augment=False
    )
    
    assert len(dataset.sessions) == 0


class TestActionStatistics:
  """Test action statistics computation."""
  
  def test_compute_action_stats(self, mock_session_dir):
    """Test action statistics computation."""
    dataset = MazeNavigationDataset(
      session_dirs=[mock_session_dir],
      sequence_length=10,
      chunk_size=15,
      augment=False
    )
    
    stats = dataset.get_action_stats()
    
    assert 'linear_mean' in stats
    assert 'linear_std' in stats
    assert 'angular_mean' in stats
    assert 'angular_std' in stats
    
    assert stats['linear_std'] > 0
    assert stats['angular_std'] > 0
  
  def test_action_stats_with_provided_stats(self, mock_session_dir):
    """Test dataset uses provided action statistics."""
    provided_stats = {
      'linear_mean': 0.0,
      'linear_std': 1.0,
      'angular_mean': 0.0,
      'angular_std': 1.0
    }
    
    dataset = MazeNavigationDataset(
      session_dirs=[mock_session_dir],
      sequence_length=10,
      chunk_size=15,
      action_stats=provided_stats,
      augment=False
    )
    
    stats = dataset.get_action_stats()
    assert stats['linear_mean'] == 0.0
    assert stats['linear_std'] == 1.0


class TestSequenceIndices:
  """Test sequence index creation."""
  
  def test_sequence_indices_creation(self, mock_session_dir):
    """Test sequence indices are created correctly."""
    dataset = MazeNavigationDataset(
      session_dirs=[mock_session_dir],
      sequence_length=10,
      chunk_size=15,
      stride=1,
      augment=False
    )
    
    assert len(dataset.sequence_indices) > 0
    
    # Each index should be (session_idx, start_idx)
    for idx in dataset.sequence_indices[:5]:
      assert len(idx) == 2
      assert isinstance(idx[0], int)  # session_idx
      assert isinstance(idx[1], int)  # start_idx
  
  def test_sequence_indices_with_stride(self, mock_session_dir):
    """Test sequence indices respect stride parameter."""
    dataset_stride1 = MazeNavigationDataset(
      session_dirs=[mock_session_dir],
      sequence_length=10,
      chunk_size=15,
      stride=1,
      augment=False
    )
    
    dataset_stride5 = MazeNavigationDataset(
      session_dirs=[mock_session_dir],
      sequence_length=10,
      chunk_size=15,
      stride=5,
      augment=False
    )
    
    assert len(dataset_stride5.sequence_indices) < len(dataset_stride1.sequence_indices)
  
  def test_sequence_indices_short_session(self, temp_dir):
    """Test sequence indices for sessions too short."""
    session_dir = os.path.join(temp_dir, "session_short")
    images_dir = os.path.join(session_dir, "images")
    metadata_dir = os.path.join(session_dir, "metadata")
    
    os.makedirs(images_dir)
    os.makedirs(metadata_dir)
    
    # Create session with only 5 samples (less than sequence_length + chunk_size)
    n_samples = 5
    csv_data = {
      'timestamp': range(n_samples),
      'image_file': [f'{i:08d}.jpg' for i in range(n_samples)],
      'cmd_linear_x': np.random.randn(n_samples).astype(float),
      'cmd_angular_z': np.random.randn(n_samples).astype(float),
    }
    
    df = pd.DataFrame(csv_data)
    df.to_csv(os.path.join(metadata_dir, "data_log.csv"), index=False)
    
    for i in range(n_samples):
      img = np.zeros((480, 640, 3), dtype=np.uint8)
      cv2.imwrite(os.path.join(images_dir, f'{i:08d}.jpg'), img)
    
    dataset = MazeNavigationDataset(
      session_dirs=[session_dir],
      sequence_length=10,
      chunk_size=15,
      augment=False
    )
    
    # Should skip this session since it's too short
    assert len(dataset.sequence_indices) == 0


class TestImageLoading:
  """Test image loading and preprocessing."""
  
  def test_load_image(self, mock_session_dir):
    """Test image loading and preprocessing."""
    dataset = MazeNavigationDataset(
      session_dirs=[mock_session_dir],
      sequence_length=10,
      chunk_size=15,
      augment=False
    )
    
    # Load an image
    session = dataset.sessions[0]
    image_path = os.path.join(session['images_dir'], session['image_files'][0])
    image = dataset._load_image(image_path)
    
    # Check shape and range
    assert image.shape == (224, 224, 3)  # Resized to image_size
    assert image.dtype == np.float32
    assert image.min() >= 0.0
    assert image.max() <= 1.0
  
  def test_load_image_handles_missing_file(self, mock_session_dir):
    """Test image loading handles missing files gracefully."""
    dataset = MazeNavigationDataset(
      session_dirs=[mock_session_dir],
      sequence_length=10,
      chunk_size=15,
      augment=False
    )
    
    # Try to load non-existent image
    image = dataset._load_image("/nonexistent/path/image.jpg")
    
    # Should return zero image as fallback
    assert image.shape == (224, 224, 3)
    assert np.all(image == 0)


class TestDataAugmentation:
  """Test data augmentation, especially horizontal flip."""
  
  def test_horizontal_flip_with_angular_velocity(self, temp_dir):
    """Test that horizontal flip negates angular velocity."""
    # Create a session with known, non-zero angular velocities
    session_dir = os.path.join(temp_dir, "session_flip_test")
    images_dir = os.path.join(session_dir, "images")
    metadata_dir = os.path.join(session_dir, "metadata")
    
    os.makedirs(images_dir)
    os.makedirs(metadata_dir)
    
    # Create session with specific angular velocities for testing
    n_samples = 50
    csv_data = {
      'timestamp': range(n_samples),
      'image_file': [f'{i:08d}.jpg' for i in range(n_samples)],
      'cmd_linear_x': [0.1] * n_samples,  # Constant linear velocity
      'cmd_angular_z': [0.5] * n_samples,  # Constant positive angular velocity
    }
    
    df = pd.DataFrame(csv_data)
    df.to_csv(os.path.join(metadata_dir, "data_log.csv"), index=False)
    
    # Create images with asymmetric pattern to detect flipping
    for i in range(n_samples):
      img = np.zeros((480, 640, 3), dtype=np.uint8)
      # Create left-right asymmetry: left side brighter
      img[:, :320, :] = 200  # Left half bright
      img[:, 320:, :] = 50   # Right half dark
      cv2.imwrite(os.path.join(images_dir, f'{i:08d}.jpg'), img)
    
    # Create dataset with augmentation enabled
    dataset = MazeNavigationDataset(
      session_dirs=[session_dir],
      sequence_length=5,
      chunk_size=10,
      augment=True,
      split='train'
    )
    
    assert len(dataset) > 0
    
    # Use mocking to deterministically test both flipped and non-flipped cases
    import unittest.mock as mock
    
    # Get sample WITHOUT flip (random > 0.5)
    with mock.patch('random.random', return_value=0.7):
      images_normal, actions_normal = dataset[0]
    
    # Get sample WITH flip (random < 0.5)
    with mock.patch('random.random', return_value=0.3):
      images_flipped, actions_flipped = dataset[0]
    
    # Verify images are actually flipped
    img_normal = images_normal[0].numpy()  # First frame
    img_flipped = images_flipped[0].numpy()
    
    # Check that left side of normal matches right side of flipped (and vice versa)
    img_normal_left = img_normal[:, :img_normal.shape[1]//2, :]
    img_normal_right = img_normal[:, img_normal.shape[1]//2:, :]
    img_flipped_left = img_flipped[:, :img_flipped.shape[1]//2, :]
    img_flipped_right = img_flipped[:, img_flipped.shape[1]//2:, :]
    
    # Normal left should match flipped right (mirrored)
    assert np.allclose(img_normal_left, np.flip(img_flipped_right, axis=1)), \
      "Images should be horizontally flipped"
    assert np.allclose(img_normal_right, np.flip(img_flipped_left, axis=1)), \
      "Images should be horizontally flipped"
    
    # Verify angular velocities are negated
    # Actions shape: (sequence_length, chunk_size, 2)
    # Check first frame, first action, angular component (index 1)
    angular_normal = actions_normal[0, 0, 1].item()
    angular_flipped = actions_flipped[0, 0, 1].item()
    
    # Denormalize to get original values and verify they're negatives
    action_stats = dataset.get_action_stats()
    angular_mean = action_stats['angular_mean']
    angular_std = action_stats['angular_std']
    
    # Denormalize: original = normalized * std + mean
    angular_normal_original = angular_normal * angular_std + angular_mean
    angular_flipped_original = angular_flipped * angular_std + angular_mean
    
    # Original values should be negatives (0.5 and -0.5)
    assert abs(angular_normal_original + angular_flipped_original) < 0.01, \
      f"Original angular velocities should be negated: normal={angular_normal_original:.4f}, flipped={angular_flipped_original:.4f}, sum={angular_normal_original + angular_flipped_original:.4f}"
    
    # Also verify the normalized values have the expected relationship
    # flipped_normalized = (normalized_original * std + mean - mean) / std
    # where normalized_original = -original_normalized
    # So: flipped = (-normal_original * std - mean) / std = -normal - 2*mean/std
    expected_flipped = -angular_normal - 2 * angular_mean / angular_std
    assert abs(angular_flipped - expected_flipped) < 0.01, \
      f"Normalized angular velocities relationship: normal={angular_normal:.4f}, flipped={angular_flipped:.4f}, expected_flipped={expected_flipped:.4f}"
    
    # Also verify linear velocity is NOT negated (should be the same)
    linear_normal = actions_normal[0, 0, 0].item()
    linear_flipped = actions_flipped[0, 0, 0].item()
    assert abs(linear_normal - linear_flipped) < 0.01, \
      f"Linear velocity should NOT be negated: normal={linear_normal:.4f}, flipped={linear_flipped:.4f}"
  
  def test_augmentation_disabled_for_val_test(self, mock_session_dir):
    """Test augmentation is disabled for val/test splits."""
    train_dataset = MazeNavigationDataset(
      session_dirs=[mock_session_dir],
      sequence_length=5,
      chunk_size=10,
      augment=True,
      split='train'
    )
    
    val_dataset = MazeNavigationDataset(
      session_dirs=[mock_session_dir],
      sequence_length=5,
      chunk_size=10,
      augment=True,
      split='val'
    )
    
    assert train_dataset.augment == True
    assert val_dataset.augment == False


class TestActionChunking:
  """Test action chunking functionality."""
  
  def test_action_chunks_shape(self, mock_session_dir):
    """Test action chunks have correct shape."""
    dataset = MazeNavigationDataset(
      session_dirs=[mock_session_dir],
      sequence_length=10,
      chunk_size=15,
      augment=False
    )
    
    if len(dataset) > 0:
      images, actions = dataset[0]
      
      assert actions.shape == (10, 15, 2)  # (seq_len, chunk_size, action_dim)
      assert images.shape == (10, 3, 224, 224)  # (seq_len, channels, H, W)
  
  def test_action_chunks_normalized(self, mock_session_dir):
    """Test action chunks are normalized."""
    dataset = MazeNavigationDataset(
      session_dirs=[mock_session_dir],
      sequence_length=10,
      chunk_size=15,
      augment=False
    )
    
    if len(dataset) > 0:
      images, actions = dataset[0]
      
      # Normalized actions should have mean ~0 and std ~1
      linear_actions = actions[:, :, 0].numpy()
      angular_actions = actions[:, :, 1].numpy()
      
      # Check they're not in original scale (very approximate)
      assert np.abs(np.mean(linear_actions)) < 1.0  # Normalized means should be small
      assert np.abs(np.mean(angular_actions)) < 1.0
  
  def test_action_chunks_padding(self, mock_session_dir):
    """Test action chunks are padded correctly at end of session."""
    # Create a very short session to test padding
    temp_dir = os.path.dirname(mock_session_dir)
    short_session_dir = os.path.join(temp_dir, "session_short_padding")
    images_dir = os.path.join(short_session_dir, "images")
    metadata_dir = os.path.join(short_session_dir, "metadata")
    
    os.makedirs(images_dir)
    os.makedirs(metadata_dir)
    
    # Only 20 samples (enough for sequence but chunk might need padding)
    n_samples = 20
    csv_data = {
      'timestamp': range(n_samples),
      'image_file': [f'{i:08d}.jpg' for i in range(n_samples)],
      'cmd_linear_x': np.random.randn(n_samples).astype(float),
      'cmd_angular_z': np.random.randn(n_samples).astype(float),
    }
    
    df = pd.DataFrame(csv_data)
    df.to_csv(os.path.join(metadata_dir, "data_log.csv"), index=False)
    
    for i in range(n_samples):
      img = np.zeros((480, 640, 3), dtype=np.uint8)
      cv2.imwrite(os.path.join(images_dir, f'{i:08d}.jpg'), img)
    
    dataset = MazeNavigationDataset(
      session_dirs=[short_session_dir],
      sequence_length=10,
      chunk_size=15,
      augment=False
    )
    
    if len(dataset) > 0:
      images, actions = dataset[-1]  # Get last sequence
      # Should still have correct shape even if padding needed
      assert actions.shape == (10, 15, 2)


class TestDatasetGetItem:
  """Test __getitem__ functionality."""
  
  def test_getitem_returns_correct_types(self, mock_session_dir):
    """Test __getitem__ returns correct types."""
    dataset = MazeNavigationDataset(
      session_dirs=[mock_session_dir],
      sequence_length=10,
      chunk_size=15,
      augment=False
    )
    
    if len(dataset) > 0:
      images, actions = dataset[0]
      
      assert isinstance(images, torch.Tensor)
      assert isinstance(actions, torch.Tensor)
      
      assert images.dtype == torch.float32
      assert actions.dtype == torch.float32
  
  def test_getitem_consistent_shapes(self, mock_session_dir):
    """Test __getitem__ returns consistent shapes across samples."""
    dataset = MazeNavigationDataset(
      session_dirs=[mock_session_dir],
      sequence_length=10,
      chunk_size=15,
      augment=False
    )
    
    if len(dataset) >= 2:
      images1, actions1 = dataset[0]
      images2, actions2 = dataset[1]
      
      assert images1.shape == images2.shape
      assert actions1.shape == actions2.shape


class TestCreateDatasets:
  """Test create_datasets function."""
  
  def test_create_datasets_splits_sessions(self, multiple_session_dirs):
    """Test create_datasets splits sessions correctly."""
    data_root = os.path.dirname(multiple_session_dirs[0])
    
    train_dataset, val_dataset, test_dataset = create_datasets(
      data_root=data_root,
      train_ratio=0.6,
      val_ratio=0.2,
      test_ratio=0.2,
      sequence_length=10,
      chunk_size=15,
      image_size=(224, 224),
      augment=False
    )
    
    # Should have datasets (might be empty depending on split)
    assert isinstance(train_dataset, MazeNavigationDataset)
    assert isinstance(val_dataset, MazeNavigationDataset)
    assert isinstance(test_dataset, MazeNavigationDataset)
    
    # All should use same action stats (from train)
    train_stats = train_dataset.get_action_stats()
    val_stats = val_dataset.get_action_stats()
    test_stats = test_dataset.get_action_stats()
    
    assert train_stats == val_stats == test_stats
  
  def test_create_datasets_handles_no_sessions(self, temp_dir):
    """Test create_datasets handles empty data root."""
    with pytest.raises(ValueError, match="No session directories found"):
      create_datasets(
        data_root=temp_dir,
        train_ratio=0.8,
        val_ratio=0.1,
        test_ratio=0.1,
        sequence_length=10,
        chunk_size=15
      )


class TestDatasetEdgeCases:
  """Test edge cases and error handling."""
  
  def test_dataset_with_empty_csv(self, temp_dir):
    """Test dataset handles empty CSV gracefully."""
    session_dir = os.path.join(temp_dir, "session_empty_csv")
    images_dir = os.path.join(session_dir, "images")
    metadata_dir = os.path.join(session_dir, "metadata")
    
    os.makedirs(images_dir)
    os.makedirs(metadata_dir)
    
    # Create empty CSV with headers only
    df = pd.DataFrame(columns=['timestamp', 'image_file', 'cmd_linear_x', 'cmd_angular_z'])
    df.to_csv(os.path.join(metadata_dir, "data_log.csv"), index=False)
    
    dataset = MazeNavigationDataset(
      session_dirs=[session_dir],
      sequence_length=10,
      chunk_size=15,
      augment=False
    )
    
    assert len(dataset.sessions) == 0
  
  def test_dataset_with_missing_columns(self, temp_dir):
    """Test dataset handles missing CSV columns gracefully."""
    session_dir = os.path.join(temp_dir, "session_missing_cols")
    images_dir = os.path.join(session_dir, "images")
    metadata_dir = os.path.join(session_dir, "metadata")
    
    os.makedirs(images_dir)
    os.makedirs(metadata_dir)
    
    # Create CSV with missing required columns
    df = pd.DataFrame({'timestamp': range(10), 'image_file': [f'{i:08d}.jpg' for i in range(10)]})
    df.to_csv(os.path.join(metadata_dir, "data_log.csv"), index=False)
    
    dataset = MazeNavigationDataset(
      session_dirs=[session_dir],
      sequence_length=10,
      chunk_size=15,
      augment=False
    )
    
    assert len(dataset.sessions) == 0
  
  def test_dataset_with_nan_values(self, temp_dir):
    """Test dataset filters out NaN values."""
    session_dir = os.path.join(temp_dir, "session_nan")
    images_dir = os.path.join(session_dir, "images")
    metadata_dir = os.path.join(session_dir, "metadata")
    
    os.makedirs(images_dir)
    os.makedirs(metadata_dir)
    
    # Create CSV with some NaN values
    csv_data = {
      'timestamp': range(20),
      'image_file': [f'{i:08d}.jpg' for i in range(20)],
      'cmd_linear_x': [np.nan if i % 5 == 0 else 0.1 for i in range(20)],
      'cmd_angular_z': [0.2 if i % 5 != 0 else np.nan for i in range(20)],
    }
    
    df = pd.DataFrame(csv_data)
    df.to_csv(os.path.join(metadata_dir, "data_log.csv"), index=False)
    
    for i in range(20):
      img = np.zeros((480, 640, 3), dtype=np.uint8)
      cv2.imwrite(os.path.join(images_dir, f'{i:08d}.jpg'), img)
    
    dataset = MazeNavigationDataset(
      session_dirs=[session_dir],
      sequence_length=5,
      chunk_size=5,
      augment=False
    )
    
    # Should filter out rows with NaN
    assert len(dataset.sessions) == 1
    # Length might be less than 20 due to NaN filtering
    assert dataset.sessions[0]['length'] < 20
  
  def test_dataset_with_mismatched_image_sizes(self, temp_dir):
    """Test dataset handles mismatched image sizes gracefully."""
    session_dir = os.path.join(temp_dir, "session_mismatched_sizes")
    images_dir = os.path.join(session_dir, "images")
    metadata_dir = os.path.join(session_dir, "metadata")
    
    os.makedirs(images_dir)
    os.makedirs(metadata_dir)
    
    # Create CSV
    n_samples = 10
    csv_data = {
      'timestamp': range(n_samples),
      'image_file': [f'{i:08d}.jpg' for i in range(n_samples)],
      'cmd_linear_x': np.random.randn(n_samples).astype(float),
      'cmd_angular_z': np.random.randn(n_samples).astype(float),
    }
    
    df = pd.DataFrame(csv_data)
    df.to_csv(os.path.join(metadata_dir, "data_log.csv"), index=False)
    
    # Create images with mismatched sizes
    for i in range(n_samples):
      if i < 5:
        # First 5 images: 480x640
        img = np.zeros((480, 640, 3), dtype=np.uint8)
      else:
        # Last 5 images: 320x240 (different size)
        img = np.zeros((320, 240, 3), dtype=np.uint8)
      cv2.imwrite(os.path.join(images_dir, f'{i:08d}.jpg'), img)
    
    dataset = MazeNavigationDataset(
      session_dirs=[session_dir],
      sequence_length=5,
      chunk_size=5,
      augment=False
    )
    
    # Should skip session due to mismatched image sizes
    assert len(dataset.sessions) == 0


if __name__ == '__main__':
  pytest.main([__file__, '-v'])

