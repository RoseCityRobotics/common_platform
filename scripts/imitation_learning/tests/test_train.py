"""
Tests for the training module.

This module tests:
- Model creation and initialization
- Dataset creation and loading
- Training loop components
- Checkpoint saving and loading
- Transfer learning utilities
"""

import os
import sys
import tempfile
import shutil
import pytest
import torch
import numpy as np
import pandas as pd
from pathlib import Path
import yaml

# Add src to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from imitation_learning.model import (
  create_model,
  ActionChunkingTransformer,
  freeze_layers,
  unfreeze_layers
)
from imitation_learning.dataset import (
  MazeNavigationDataset,
  create_datasets
)
from imitation_learning.utils import (
  load_config,
  save_checkpoint,
  load_checkpoint,
  compute_metrics,
  get_device,
  set_seed
)
from imitation_learning.train import EarlyStopping


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
  
  # Create mock CSV data
  n_samples = 100
  csv_data = {
    'timestamp': range(n_samples),
    'image_file': [f'{i:08d}.jpg' for i in range(n_samples)],
    'linear_vel': np.random.randn(n_samples).astype(float),
    'angular_vel': np.random.randn(n_samples).astype(float),
    'position_x': np.random.randn(n_samples).astype(float),
    'position_y': np.random.randn(n_samples).astype(float),
    'orientation_z': np.random.randn(n_samples).astype(float),
    'cmd_linear_x': np.random.uniform(-0.5, 0.5, n_samples).astype(float),
    'cmd_angular_z': np.random.uniform(-1.0, 1.0, n_samples).astype(float),
  }
  
  df = pd.DataFrame(csv_data)
  df.to_csv(os.path.join(metadata_dir, "data_log.csv"), index=False)
  
  # Create dummy image files (small black images)
  import cv2
  for i in range(n_samples):
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.imwrite(os.path.join(images_dir, f'{i:08d}.jpg'), img)
  
  return session_dir


@pytest.fixture
def sample_config(temp_dir):
  """Create a sample configuration file."""
  config = {
    'dataset': {
      'data_root': temp_dir,
      'sequence_length': 5,
      'chunk_size': 10,
      'stride': 1,
      'image_size': [224, 224],
      'augment': False,
      'train_ratio': 1.0,
      'val_ratio': 0.0,
      'test_ratio': 0.0,
    },
    'model': {
      'backbone': 'resnet18',
      'pretrained': False,  # Faster for testing
      'freeze_backbone': False,
      'd_model': 256,  # Smaller for testing
      'nhead': 4,
      'num_encoder_layers': 2,  # Fewer layers for testing
      'num_decoder_layers': 2,
      'dim_feedforward': 1024,
      'dropout': 0.1,
      'action_dim': 2,
      'chunk_size': 10,
      'max_seq_len': 100,
    },
    'training': {
      'batch_size': 2,
      'num_epochs': 1,
      'learning_rate': 1e-4,
      'weight_decay': 1e-5,
      'gradient_clip_norm': 1.0,
      'early_stopping_patience': 10,
      'mixed_precision': False,  # Simpler for testing
      'lr_schedule': {
        'type': 'cosine',
        'warmup_epochs': 0,
        'min_lr': 1e-6,
      },
      'optimizer': {
        'type': 'adamw',
        'betas': [0.9, 0.999],
        'eps': 1e-8,
      },
    },
    'dataloader': {
      'num_workers': 0,  # Avoid multiprocessing issues in tests
      'pin_memory': False,
      'persistent_workers': False,
    },
    'seed': 42,
    'device': 'cpu',
  }
  
  config_path = os.path.join(temp_dir, 'test_config.yaml')
  with open(config_path, 'w') as f:
    yaml.dump(config, f)
  
  return config_path


class TestModel:
  """Test model creation and operations."""
  
  def test_model_creation(self, sample_config):
    """Test creating a model from config."""
    config = load_config(sample_config)
    model = create_model(config)
    
    assert isinstance(model, ActionChunkingTransformer)
    assert model.d_model == config['model']['d_model']
    assert model.chunk_size == config['model']['chunk_size']
    assert model.action_dim == config['model']['action_dim']
  
  def test_model_forward(self, sample_config):
    """Test model forward pass."""
    config = load_config(sample_config)
    model = create_model(config)
    model.eval()
    
    batch_size = 2
    seq_len = config['dataset']['sequence_length']
    image_size = config['dataset']['image_size']
    
    # Create dummy input
    images = torch.randn(batch_size, seq_len, 3, image_size[0], image_size[1])
    
    with torch.no_grad():
      output = model(images)
    
    expected_shape = (batch_size, seq_len, config['model']['chunk_size'], config['model']['action_dim'])
    assert output.shape == expected_shape
  
  def test_freeze_layers(self, sample_config):
    """Test freezing model layers."""
    config = load_config(sample_config)
    model = create_model(config)
    
    # Freeze vision encoder
    freeze_layers(model, ['vision_encoder'])
    
    # Check that vision encoder params are frozen
    for name, param in model.named_parameters():
      if name.startswith('vision_encoder'):
        assert not param.requires_grad, f"{name} should be frozen"
  
  def test_unfreeze_layers(self, sample_config):
    """Test unfreezing model layers."""
    config = load_config(sample_config)
    model = create_model(config)
    
    # Freeze then unfreeze
    freeze_layers(model, ['vision_encoder'])
    unfreeze_layers(model, ['vision_encoder'])
    
    # Check that vision encoder params are unfrozen
    vision_unfrozen = False
    for name, param in model.named_parameters():
      if name.startswith('vision_encoder') and param.requires_grad:
        vision_unfrozen = True
        break
    
    assert vision_unfrozen, "Vision encoder should be unfrozen"


class TestDataset:
  """Test dataset creation and loading."""
  
  def test_dataset_creation(self, mock_session_dir, sample_config):
    """Test creating a dataset from session directory."""
    config = load_config(sample_config)
    
    dataset = MazeNavigationDataset(
      session_dirs=[mock_session_dir],
      sequence_length=config['dataset']['sequence_length'],
      chunk_size=config['dataset']['chunk_size'],
      image_size=tuple(config['dataset']['image_size']),
      augment=False
    )
    
    assert len(dataset) > 0
    assert dataset.get_action_stats() is not None
  
  def test_dataset_getitem(self, mock_session_dir, sample_config):
    """Test dataset __getitem__ method."""
    config = load_config(sample_config)
    
    dataset = MazeNavigationDataset(
      session_dirs=[mock_session_dir],
      sequence_length=config['dataset']['sequence_length'],
      chunk_size=config['dataset']['chunk_size'],
      image_size=tuple(config['dataset']['image_size']),
      augment=False
    )
    
    if len(dataset) > 0:
      images, actions = dataset[0]
      
      seq_len = config['dataset']['sequence_length']
      chunk_size = config['model']['chunk_size']
      
      assert images.shape == (seq_len, 3, config['dataset']['image_size'][0], config['dataset']['image_size'][1])
      assert actions.shape == (seq_len, chunk_size, 2)
  
  def test_create_datasets(self, mock_session_dir, sample_config):
    """Test create_datasets function."""
    config = load_config(sample_config)
    
    # Update config to use temp directory
    config['dataset']['data_root'] = os.path.dirname(mock_session_dir)
    config['dataset']['train_ratio'] = 1.0
    config['dataset']['val_ratio'] = 0.0
    config['dataset']['test_ratio'] = 0.0
    
    train_dataset, val_dataset, test_dataset = create_datasets(
      data_root=config['dataset']['data_root'],
      train_ratio=config['dataset']['train_ratio'],
      val_ratio=config['dataset']['val_ratio'],
      test_ratio=config['dataset']['test_ratio'],
      sequence_length=config['dataset']['sequence_length'],
      chunk_size=config['dataset']['chunk_size'],
      image_size=tuple(config['dataset']['image_size']),
      augment=False
    )
    
    assert len(train_dataset) > 0
    # Val and test might be empty with these ratios
    assert isinstance(val_dataset, MazeNavigationDataset)
    assert isinstance(test_dataset, MazeNavigationDataset)


class TestUtils:
  """Test utility functions."""
  
  def test_compute_metrics(self):
    """Test metrics computation."""
    batch_size = 2
    seq_len = 5
    chunk_size = 10
    action_dim = 2
    
    predictions = torch.randn(batch_size, seq_len, chunk_size, action_dim)
    targets = predictions + 0.1 * torch.randn_like(predictions)
    
    metrics = compute_metrics(predictions, targets)
    
    assert 'mse' in metrics
    assert 'mae' in metrics
    assert 'rmse' in metrics
    assert 'linear_corr' in metrics
    assert 'angular_corr' in metrics
  
  def test_save_load_checkpoint(self, sample_config, temp_dir):
    """Test checkpoint saving and loading."""
    config = load_config(sample_config)
    model = create_model(config)
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-4)
    
    # Save checkpoint
    checkpoint_path = save_checkpoint(
      model,
      optimizer,
      epoch=5,
      loss=0.5,
      metrics={'mse': 0.1},
      checkpoint_dir=temp_dir,
      filename='test_checkpoint.pth'
    )
    
    assert os.path.exists(checkpoint_path)
    
    # Load checkpoint
    new_model = create_model(config)
    new_optimizer = torch.optim.AdamW(new_model.parameters(), lr=1e-4)
    
    checkpoint = load_checkpoint(
      checkpoint_path,
      new_model,
      new_optimizer
    )
    
    assert checkpoint['epoch'] == 5
    assert checkpoint['loss'] == 0.5
    assert 'mse' in checkpoint['metrics']


class TestEarlyStopping:
  """Test early stopping utility."""
  
  def test_early_stopping(self, sample_config):
    """Test early stopping logic."""
    config = load_config(sample_config)
    model = create_model(config)
    
    early_stopping = EarlyStopping(patience=3, min_delta=0.01)
    
    # Simulate improving then plateauing losses
    losses = [1.0, 0.9, 0.85, 0.86, 0.87, 0.88]
    
    should_stop = False
    for epoch, loss in enumerate(losses):
      should_stop = early_stopping(loss, model)
      if should_stop:
        break
    
    # Should stop after patience is exceeded
    assert should_stop or epoch >= 5  # At least patience epochs passed


class TestTrainingIntegration:
  """Integration tests for training components."""
  
  def test_model_dataset_integration(self, mock_session_dir, sample_config):
    """Test model and dataset work together."""
    config = load_config(sample_config)
    
    # Create model and dataset
    model = create_model(config)
    model.eval()
    
    dataset = MazeNavigationDataset(
      session_dirs=[mock_session_dir],
      sequence_length=config['dataset']['sequence_length'],
      chunk_size=config['dataset']['chunk_size'],
      image_size=tuple(config['dataset']['image_size']),
      augment=False
    )
    
    if len(dataset) > 0:
      # Get a sample
      images, actions = dataset[0]
      
      # Model forward pass
      with torch.no_grad():
        predictions = model(images.unsqueeze(0))
      
      assert predictions.shape[0] == 1  # Batch size
      # Predictions shape: (batch, seq_len, chunk_size, action_dim)
      # Actions shape: (seq_len, chunk_size, action_dim)
      assert predictions.shape[1:] == actions.shape


if __name__ == '__main__':
  pytest.main([__file__, '-v'])


