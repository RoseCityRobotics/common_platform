import os
import json
import torch
import torch.nn as nn
import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, Any, Optional, Tuple, List
import yaml
from pathlib import Path
import logging
from datetime import datetime


def setup_logging(log_dir: str, name: str = 'training') -> logging.Logger:
  """Setup logging configuration."""
  os.makedirs(log_dir, exist_ok=True)
  
  # Create logger
  logger = logging.getLogger(name)
  logger.setLevel(logging.INFO)
  
  # Remove existing handlers
  for handler in logger.handlers[:]:
    logger.removeHandler(handler)
  
  # Create formatter
  formatter = logging.Formatter(
    '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
  )
  
  # File handler
  log_file = os.path.join(log_dir, f'{name}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log')
  file_handler = logging.FileHandler(log_file)
  file_handler.setLevel(logging.INFO)
  file_handler.setFormatter(formatter)
  logger.addHandler(file_handler)
  
  # Console handler
  console_handler = logging.StreamHandler()
  console_handler.setLevel(logging.INFO)
  console_handler.setFormatter(formatter)
  logger.addHandler(console_handler)
  
  return logger


def load_config(config_path: str) -> Dict[str, Any]:
  """Load configuration from YAML file."""
  with open(config_path, 'r') as f:
    config = yaml.safe_load(f)
  
  # Ensure numeric values are properly converted (YAML sometimes parses scientific notation as strings)
  def convert_numeric_values(obj):
    """Recursively convert string numbers to floats/ints."""
    if isinstance(obj, dict):
      return {k: convert_numeric_values(v) for k, v in obj.items()}
    elif isinstance(obj, list):
      return [convert_numeric_values(item) for item in obj]
    elif isinstance(obj, str):
      # Try to convert string to number if possible
      stripped = obj.strip()
      if not stripped:
        return obj
      
      # Try converting to number (handles int, float, and scientific notation)
      try:
        # Try float first (handles scientific notation like "1e-4")
        float_val = float(stripped)
        # If it's actually an integer, return int
        if '.' not in stripped and 'e' not in stripped.lower():
          try:
            return int(stripped)
          except ValueError:
            pass
        return float_val
      except ValueError:
        # Not a number, return as-is
        return obj
    else:
      return obj
  
  config = convert_numeric_values(config)
  return config


def save_config(config: Dict[str, Any], config_path: str):
  """Save configuration to YAML file."""
  os.makedirs(os.path.dirname(config_path), exist_ok=True)
  with open(config_path, 'w') as f:
    yaml.dump(config, f, default_flow_style=False, indent=2)


def save_checkpoint(
  model: nn.Module,
  optimizer: torch.optim.Optimizer,
  epoch: int,
  loss: float,
  metrics: Dict[str, float],
  checkpoint_dir: str,
  filename: str = None
) -> str:
  """Save model checkpoint."""
  os.makedirs(checkpoint_dir, exist_ok=True)
  
  if filename is None:
    filename = f'checkpoint_epoch_{epoch:04d}.pth'
  
  checkpoint_path = os.path.join(checkpoint_dir, filename)
  
  checkpoint = {
    'epoch': epoch,
    'model_state_dict': model.state_dict(),
    'optimizer_state_dict': optimizer.state_dict(),
    'loss': loss,
    'metrics': metrics,
    'timestamp': datetime.now().isoformat()
  }
  
  torch.save(checkpoint, checkpoint_path)
  return checkpoint_path


def load_checkpoint(
  checkpoint_path: str,
  model: nn.Module,
  optimizer: Optional[torch.optim.Optimizer] = None,
  device: str = 'cpu'
) -> Dict[str, Any]:
  """Load model checkpoint."""
  checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
  
  model.load_state_dict(checkpoint['model_state_dict'])
  
  if optimizer is not None and 'optimizer_state_dict' in checkpoint:
    optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
  
  return checkpoint


def compute_kl_loss(z_mean: torch.Tensor, z_logvar: torch.Tensor) -> torch.Tensor:
  """
  Compute KL divergence loss between learned distribution and standard normal prior.
  
  Following ALOHA paper: L_reg = D_KL(q_φ(z|a_{t:t+k}, ō_t) || N(0, I))
  
  Note: With z_mean fixed to 0, the KL simplifies to:
        KL = -0.5 * sum(1 + logvar - exp(logvar))
  
  Args:
    z_mean: (batch, z_dim) - mean of learned distribution (should be zeros)
    z_logvar: (batch, z_dim) - log variance of learned distribution
  
  Returns:
    kl_loss: Scalar KL divergence loss
  """
  # KL divergence: -0.5 * sum(1 + logvar - mean^2 - exp(logvar))
  # With z_mean = 0, this simplifies to: -0.5 * sum(1 + logvar - exp(logvar))
  kl_loss = -0.5 * torch.sum(1 + z_logvar - z_logvar.exp(), dim=1)
  kl_loss = torch.mean(kl_loss)
  return kl_loss


def compute_metrics(
  predictions: torch.Tensor,
  targets: torch.Tensor,
  action_stats: Optional[Dict[str, float]] = None
) -> Dict[str, float]:
  """
  Compute evaluation metrics.
  
  Args:
    predictions: (batch, seq_len, chunk_size, 2) predicted actions
    targets: (batch, seq_len, chunk_size, 2) target actions
    action_stats: Action normalization statistics for denormalization
  
  Returns:
    Dictionary of metrics
  """
  # Denormalize if stats provided
  if action_stats is not None:
    pred_denorm = denormalize_actions(predictions, action_stats)
    target_denorm = denormalize_actions(targets, action_stats)
  else:
    pred_denorm = predictions
    target_denorm = targets
  
  # Flatten for metric computation
  pred_flat = pred_denorm.view(-1, 2)
  target_flat = target_denorm.view(-1, 2)
  
  # MSE
  mse = torch.mean((pred_flat - target_flat) ** 2).item()
  
  # MAE
  mae = torch.mean(torch.abs(pred_flat - target_flat)).item()
  
  # RMSE
  rmse = torch.sqrt(torch.mean((pred_flat - target_flat) ** 2)).item()
  
  # Per-action metrics
  linear_mse = torch.mean((pred_flat[:, 0] - target_flat[:, 0]) ** 2).item()
  angular_mse = torch.mean((pred_flat[:, 1] - target_flat[:, 1]) ** 2).item()
  
  linear_mae = torch.mean(torch.abs(pred_flat[:, 0] - target_flat[:, 0])).item()
  angular_mae = torch.mean(torch.abs(pred_flat[:, 1] - target_flat[:, 1])).item()
  
  # Correlation
  linear_corr = torch.corrcoef(torch.stack([pred_flat[:, 0], target_flat[:, 0]]))[0, 1].item()
  angular_corr = torch.corrcoef(torch.stack([pred_flat[:, 1], target_flat[:, 1]]))[0, 1].item()
  
  return {
    'mse': mse,
    'mae': mae,
    'rmse': rmse,
    'linear_mse': linear_mse,
    'angular_mse': angular_mse,
    'linear_mae': linear_mae,
    'angular_mae': angular_mae,
    'linear_corr': linear_corr,
    'angular_corr': angular_corr
  }


def denormalize_actions(
  normalized_actions: torch.Tensor,
  action_stats: Dict[str, float]
) -> torch.Tensor:
  """Denormalize actions back to original scale."""
  denormalized = normalized_actions.clone()
  denormalized[..., 0] = (denormalized[..., 0] * action_stats['linear_std'] + 
                         action_stats['linear_mean'])
  denormalized[..., 1] = (denormalized[..., 1] * action_stats['angular_std'] + 
                         action_stats['angular_mean'])
  return denormalized


def plot_training_curves(
  train_losses: List[float],
  val_losses: List[float],
  save_path: str,
  title: str = 'Training Curves'
):
  """Plot and save training curves."""
  plt.figure(figsize=(12, 4))
  
  # Loss curves
  plt.subplot(1, 2, 1)
  plt.plot(train_losses, label='Train Loss', alpha=0.8)
  plt.plot(val_losses, label='Val Loss', alpha=0.8)
  plt.xlabel('Epoch')
  plt.ylabel('Loss')
  plt.title('Loss Curves')
  plt.legend()
  plt.grid(True, alpha=0.3)
  
  # Learning rate (if available)
  plt.subplot(1, 2, 2)
  plt.plot(range(len(train_losses)), train_losses, label='Train Loss', alpha=0.8)
  plt.xlabel('Epoch')
  plt.ylabel('Loss')
  plt.title('Training Progress')
  plt.legend()
  plt.grid(True, alpha=0.3)
  
  plt.tight_layout()
  plt.savefig(save_path, dpi=150, bbox_inches='tight')
  plt.close()


def plot_action_predictions(
  predictions: torch.Tensor,
  targets: torch.Tensor,
  save_path: str,
  action_stats: Optional[Dict[str, float]] = None,
  max_samples: int = 5
):
  """
  Plot action predictions vs targets.
  
  Args:
    predictions: (batch, seq_len, chunk_size, 2) predicted actions
    targets: (batch, seq_len, chunk_size, 2) target actions
    save_path: Path to save plot
    action_stats: Action normalization statistics
    max_samples: Maximum number of samples to plot
  """
  # Denormalize if stats provided
  if action_stats is not None:
    pred_denorm = denormalize_actions(predictions, action_stats)
    target_denorm = denormalize_actions(targets, action_stats)
  else:
    pred_denorm = predictions
    target_denorm = targets
  
  # Take first few samples
  n_samples = min(max_samples, pred_denorm.shape[0])
  
  fig, axes = plt.subplots(2, n_samples, figsize=(4 * n_samples, 8))
  if n_samples == 1:
    axes = axes.reshape(2, 1)
  
  for i in range(n_samples):
    # Linear velocity
    ax = axes[0, i]
    seq_len = pred_denorm.shape[1]
    chunk_size = pred_denorm.shape[2]
    
    # Plot first action of each chunk (immediate prediction)
    pred_linear = pred_denorm[i, :, 0, 0].cpu().numpy()
    target_linear = target_denorm[i, :, 0, 0].cpu().numpy()
    
    ax.plot(range(seq_len), target_linear, 'b-', label='Target', alpha=0.7)
    ax.plot(range(seq_len), pred_linear, 'r--', label='Predicted', alpha=0.7)
    ax.set_title(f'Sample {i+1} - Linear Velocity')
    ax.set_xlabel('Time Step')
    ax.set_ylabel('Linear Velocity (m/s)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Angular velocity
    ax = axes[1, i]
    pred_angular = pred_denorm[i, :, 0, 1].cpu().numpy()
    target_angular = target_denorm[i, :, 0, 1].cpu().numpy()
    
    ax.plot(range(seq_len), target_angular, 'b-', label='Target', alpha=0.7)
    ax.plot(range(seq_len), pred_angular, 'r--', label='Predicted', alpha=0.7)
    ax.set_title(f'Sample {i+1} - Angular Velocity')
    ax.set_xlabel('Time Step')
    ax.set_ylabel('Angular Velocity (rad/s)')
    ax.legend()
    ax.grid(True, alpha=0.3)
  
  plt.tight_layout()
  plt.savefig(save_path, dpi=150, bbox_inches='tight')
  plt.close()


def plot_action_chunks(
  predictions: torch.Tensor,
  targets: torch.Tensor,
  save_path: str,
  action_stats: Optional[Dict[str, float]] = None,
  sample_idx: int = 0,
  time_idx: int = 0
):
  """
  Plot action chunks for a specific sample and time step.
  
  Args:
    predictions: (batch, seq_len, chunk_size, 2) predicted actions
    targets: (batch, seq_len, chunk_size, 2) target actions
    save_path: Path to save plot
    action_stats: Action normalization statistics
    sample_idx: Sample index to plot
    time_idx: Time step index to plot
  """
  # Denormalize if stats provided
  if action_stats is not None:
    pred_denorm = denormalize_actions(predictions, action_stats)
    target_denorm = denormalize_actions(targets, action_stats)
  else:
    pred_denorm = predictions
    target_denorm = targets
  
  # Extract chunk for specific sample and time
  pred_chunk = pred_denorm[sample_idx, time_idx].cpu().numpy()  # (chunk_size, 2)
  target_chunk = target_denorm[sample_idx, time_idx].cpu().numpy()  # (chunk_size, 2)
  
  chunk_size = pred_chunk.shape[0]
  
  fig, axes = plt.subplots(1, 2, figsize=(12, 5))
  
  # Linear velocity chunk
  ax = axes[0]
  ax.plot(range(chunk_size), target_chunk[:, 0], 'b-o', label='Target', alpha=0.7)
  ax.plot(range(chunk_size), pred_chunk[:, 0], 'r--s', label='Predicted', alpha=0.7)
  ax.set_title(f'Action Chunk - Linear Velocity (Sample {sample_idx}, Time {time_idx})')
  ax.set_xlabel('Chunk Step')
  ax.set_ylabel('Linear Velocity (m/s)')
  ax.legend()
  ax.grid(True, alpha=0.3)
  
  # Angular velocity chunk
  ax = axes[1]
  ax.plot(range(chunk_size), target_chunk[:, 1], 'b-o', label='Target', alpha=0.7)
  ax.plot(range(chunk_size), pred_chunk[:, 1], 'r--s', label='Predicted', alpha=0.7)
  ax.set_title(f'Action Chunk - Angular Velocity (Sample {sample_idx}, Time {time_idx})')
  ax.set_xlabel('Chunk Step')
  ax.set_ylabel('Angular Velocity (rad/s)')
  ax.legend()
  ax.grid(True, alpha=0.3)
  
  plt.tight_layout()
  plt.savefig(save_path, dpi=150, bbox_inches='tight')
  plt.close()


def count_parameters(model: nn.Module) -> int:
  """Count the number of trainable parameters in a model."""
  return sum(p.numel() for p in model.parameters() if p.requires_grad)


def get_device() -> torch.device:
  """Get the best available device."""
  if torch.cuda.is_available():
    return torch.device('cuda')
  elif torch.backends.mps.is_available():
    return torch.device('mps')
  else:
    return torch.device('cpu')


def set_seed(seed: int):
  """Set random seed for reproducibility."""
  torch.manual_seed(seed)
  torch.cuda.manual_seed_all(seed)
  np.random.seed(seed)
  torch.backends.cudnn.deterministic = True
  torch.backends.cudnn.benchmark = False


def create_experiment_dir(base_dir: str, experiment_name: str) -> str:
  """Create experiment directory with timestamp."""
  timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
  exp_dir = os.path.join(base_dir, f'{experiment_name}_{timestamp}')
  os.makedirs(exp_dir, exist_ok=True)
  return exp_dir


def save_experiment_info(
  exp_dir: str,
  config: Dict[str, Any],
  model_info: Dict[str, Any]
):
  """Save experiment information."""
  info = {
    'config': config,
    'model_info': model_info,
    'timestamp': datetime.now().isoformat()
  }
  
  info_path = os.path.join(exp_dir, 'experiment_info.json')
  with open(info_path, 'w') as f:
    json.dump(info, f, indent=2, default=str)


def load_experiment_info(exp_dir: str) -> Dict[str, Any]:
  """Load experiment information."""
  info_path = os.path.join(exp_dir, 'experiment_info.json')
  with open(info_path, 'r') as f:
    return json.load(f)