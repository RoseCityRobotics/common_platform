#!/usr/bin/env python3
"""
Training script for maze navigation imitation learning.

This script implements the full training pipeline including:
- Dataset loading and preprocessing
- Model training with validation
- Early stopping and checkpointing
- TensorBoard logging
- Mixed precision training
"""

import os
import sys
import argparse
import yaml
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter
from torch.amp import GradScaler, autocast
import numpy as np
from tqdm import tqdm
import logging
from pathlib import Path

# Add training directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from dataset import create_datasets
from model import create_model, load_pretrained_weights, freeze_layers
from utils import (
  setup_logging, load_config, save_config, save_checkpoint, load_checkpoint,
  compute_metrics, plot_training_curves, plot_action_predictions,
  count_parameters, get_device, set_seed, create_experiment_dir,
  save_experiment_info
)


class EarlyStopping:
  """Early stopping utility."""
  
  def __init__(self, patience: int = 10, min_delta: float = 0.0, restore_best_weights: bool = True):
    self.patience = patience
    self.min_delta = min_delta
    self.restore_best_weights = restore_best_weights
    self.best_loss = float('inf')
    self.counter = 0
    self.best_weights = None
    
  def __call__(self, val_loss: float, model: nn.Module) -> bool:
    if val_loss < self.best_loss - self.min_delta:
      self.best_loss = val_loss
      self.counter = 0
      if self.restore_best_weights:
        self.best_weights = model.state_dict().copy()
    else:
      self.counter += 1
    
    if self.counter >= self.patience:
      if self.restore_best_weights and self.best_weights is not None:
        model.load_state_dict(self.best_weights)
      return True
    return False


class CosineAnnealingWarmupRestarts:
  """Cosine annealing with warmup and restarts."""
  
  def __init__(self, optimizer, first_cycle_steps: int, cycle_mult: float = 1.0,
               max_lr: float = 0.1, min_lr: float = 0.001, warmup_steps: int = 0,
               gamma: float = 1.0):
    self.optimizer = optimizer
    self.first_cycle_steps = first_cycle_steps
    self.cycle_mult = cycle_mult
    self.base_max_lr = max_lr
    self.max_lr = max_lr
    self.min_lr = min_lr
    self.warmup_steps = warmup_steps
    self.gamma = gamma
    
    self.cur_cycle_steps = first_cycle_steps
    self.cycle = 0
    self.step_in_cycle = 0
    
  def step(self):
    self.step_in_cycle += 1
    
    if self.step_in_cycle == self.cur_cycle_steps:
      self.step_in_cycle = 0
      self.cycle += 1
      self.cur_cycle_steps = int(self.first_cycle_steps * (self.cycle_mult ** self.cycle))
      self.max_lr = self.base_max_lr * (self.gamma ** self.cycle)
    
    if self.step_in_cycle < self.warmup_steps:
      lr = self.min_lr + (self.max_lr - self.min_lr) * self.step_in_cycle / self.warmup_steps
    else:
      progress = (self.step_in_cycle - self.warmup_steps) / (self.cur_cycle_steps - self.warmup_steps)
      lr = self.min_lr + (self.max_lr - self.min_lr) * 0.5 * (1 + np.cos(np.pi * progress))
    
    for param_group in self.optimizer.param_groups:
      param_group['lr'] = lr
    
    return lr


def train_epoch(
  model: nn.Module,
  dataloader: DataLoader,
  optimizer: optim.Optimizer,
  criterion: nn.Module,
  device: torch.device,
  scaler: GradScaler,
  mixed_precision: bool = True,
  log_frequency: int = 100
) -> float:
  """Train for one epoch."""
  model.train()
  total_loss = 0.0
  num_batches = len(dataloader)
  
  # Determine device type for mixed precision
  device_type = 'cpu'
  if device.type == 'cuda':
    device_type = 'cuda'
  elif device.type == 'mps':
    device_type = 'mps'
  
  pbar = tqdm(dataloader, desc="Training", leave=False)
  for batch_idx, (images, actions) in enumerate(pbar):
    images = images.to(device)
    actions = actions.to(device)
    
    optimizer.zero_grad()
    
    if mixed_precision and device_type == 'cuda':
      # Mixed precision only supported for CUDA
      with autocast(device_type='cuda'):
        predictions = model(images)
        loss = criterion(predictions, actions)
      
      scaler.scale(loss).backward()
      scaler.step(optimizer)
      scaler.update()
    else:
      # Full precision for MPS or CPU
      predictions = model(images)
      loss = criterion(predictions, actions)
      loss.backward()
      optimizer.step()
    
    total_loss += loss.item()
    
    # Update progress bar
    pbar.set_postfix({'loss': f'{loss.item():.4f}'})
    
    # Log every log_frequency steps
    if batch_idx % log_frequency == 0:
      logging.info(f'Batch {batch_idx}/{num_batches}, Loss: {loss.item():.4f}')
  
  return total_loss / num_batches


def validate_epoch(
  model: nn.Module,
  dataloader: DataLoader,
  criterion: nn.Module,
  device: torch.device,
  action_stats: dict
) -> tuple:
  """Validate for one epoch."""
  model.eval()
  total_loss = 0.0
  all_predictions = []
  all_targets = []
  
  with torch.no_grad():
    for images, actions in tqdm(dataloader, desc="Validation", leave=False):
      images = images.to(device)
      actions = actions.to(device)
      
      predictions = model(images)
      loss = criterion(predictions, actions)
      
      total_loss += loss.item()
      all_predictions.append(predictions.cpu())
      all_targets.append(actions.cpu())
  
  # Concatenate all predictions and targets
  all_predictions = torch.cat(all_predictions, dim=0)
  all_targets = torch.cat(all_targets, dim=0)
  
  # Compute metrics
  metrics = compute_metrics(all_predictions, all_targets, action_stats)
  
  return total_loss / len(dataloader), metrics


def main():
  parser = argparse.ArgumentParser(description='Train maze navigation model')
  parser.add_argument('--config', type=str, default='config.yaml',
                     help='Path to configuration file')
  parser.add_argument('--data_root', type=str, default=None,
                     help='Override data root directory')
  parser.add_argument('--output_dir', type=str, default=None,
                     help='Override output directory')
  parser.add_argument('--resume', type=str, default=None,
                     help='Resume training from checkpoint')
  parser.add_argument('--pretrained', type=str, default=None,
                     help='Path to pretrained model weights')
  parser.add_argument('--device', type=str, default=None,
                     help='Override device (cpu, cuda, mps)')
  parser.add_argument('--seed', type=int, default=None,
                     help='Override random seed')
  
  args = parser.parse_args()
  
  # Load configuration
  config = load_config(args.config)
  
  # Override config with command line arguments
  if args.data_root:
    config['dataset']['data_root'] = args.data_root
  if args.output_dir:
    config['paths']['output_dir'] = args.output_dir
  if args.device:
    config['device'] = args.device
  if args.seed:
    config['seed'] = args.seed
  
  # Set random seed
  set_seed(config['seed'])
  
  # Get device
  if config['device'] == 'auto':
    device = get_device()
  else:
    device = torch.device(config['device'])
  
  print(f"Using device: {device}")
  if device.type == 'mps':
    print("Apple Silicon GPU (MPS) detected and enabled!")
  elif device.type == 'cuda':
    print("CUDA GPU detected and enabled!")
  else:
    print("Using CPU (no GPU acceleration)")
  
  # Create experiment directory
  exp_dir = create_experiment_dir(
    config['paths']['output_dir'],
    config['experiment']['name']
  )
  
  # Setup logging
  logger = setup_logging(exp_dir, 'training')
  logger.info(f"Starting training experiment: {exp_dir}")
  logger.info(f"Configuration: {config}")
  
  # Save configuration
  save_config(config, os.path.join(exp_dir, 'config.yaml'))
  
  # Create datasets
  logger.info("Loading datasets...")
  train_dataset, val_dataset, test_dataset = create_datasets(
    data_root=config['dataset']['data_root'],
    train_ratio=config['dataset']['train_ratio'],
    val_ratio=config['dataset']['val_ratio'],
    test_ratio=config['dataset']['test_ratio'],
    sequence_length=config['dataset']['sequence_length'],
    chunk_size=config['dataset']['chunk_size'],
    stride=config['dataset']['stride'],
    image_size=tuple(config['dataset']['image_size']),
    augment=config['dataset']['augment']
  )
  
  # Create data loaders
  train_loader = DataLoader(
    train_dataset,
    batch_size=config['training']['batch_size'],
    shuffle=True,
    num_workers=config['dataloader']['num_workers'],
    pin_memory=config['dataloader']['pin_memory'],
    persistent_workers=config['dataloader']['persistent_workers']
  )
  
  val_loader = DataLoader(
    val_dataset,
    batch_size=config['training']['batch_size'],
    shuffle=False,
    num_workers=config['dataloader']['num_workers'],
    pin_memory=config['dataloader']['pin_memory'],
    persistent_workers=config['dataloader']['persistent_workers']
  )
  
  # Create model
  logger.info("Creating model...")
  model = create_model(config)
  model = model.to(device)
  
  # Print model info
  model_info = model.get_model_info()
  logger.info(f"Model info: {model_info}")
  print(f"Model has {count_parameters(model):,} trainable parameters")
  
  # Load pretrained weights if specified
  if args.pretrained:
    logger.info(f"Loading pretrained weights from {args.pretrained}")
    model = load_pretrained_weights(model, args.pretrained, device)
  
  # Transfer learning setup
  if config['transfer_learning']['enabled']:
    logger.info("Setting up transfer learning...")
    if config['transfer_learning']['freeze_vision']:
      freeze_layers(model, ['vision_encoder'])
    if config['transfer_learning']['freeze_encoder']:
      freeze_layers(model, ['temporal_encoder'])
    if config['transfer_learning']['freeze_decoder']:
      freeze_layers(model, ['action_decoder'])
  
  # Create optimizer
  if config['training']['optimizer']['type'] == 'adamw':
    optimizer = optim.AdamW(
      model.parameters(),
      lr=config['training']['learning_rate'],
      weight_decay=config['training']['weight_decay'],
      betas=config['training']['optimizer']['betas'],
      eps=config['training']['optimizer']['eps']
    )
  elif config['training']['optimizer']['type'] == 'adam':
    optimizer = optim.Adam(
      model.parameters(),
      lr=config['training']['learning_rate'],
      weight_decay=config['training']['weight_decay'],
      betas=config['training']['optimizer']['betas'],
      eps=config['training']['optimizer']['eps']
    )
  else:
    raise ValueError(f"Unsupported optimizer: {config['training']['optimizer']['type']}")
  
  # Create loss function
  criterion = nn.MSELoss()
  
  # Create learning rate scheduler
  if config['training']['lr_schedule']['type'] == 'cosine':
    scheduler = CosineAnnealingWarmupRestarts(
      optimizer,
      first_cycle_steps=len(train_loader) * config['training']['num_epochs'],
      max_lr=config['training']['learning_rate'],
      min_lr=config['training']['lr_schedule']['min_lr'],
      warmup_steps=len(train_loader) * config['training']['lr_schedule']['warmup_epochs']
    )
  else:
    scheduler = None
  
  # Mixed precision scaler (device-aware)
  # Note: MPS doesn't support autocast, so mixed precision is only enabled for CUDA
  device_type = 'cpu'
  if device.type == 'cuda':
    device_type = 'cuda'
  elif device.type == 'mps':
    device_type = 'mps'
  
  # Only enable mixed precision for CUDA (MPS doesn't support autocast)
  use_mixed_precision = config['training']['mixed_precision'] and device_type == 'cuda'
  if use_mixed_precision:
    scaler = GradScaler(device_type)
    logger.info(f"Mixed precision training enabled for device: {device_type}")
  else:
    scaler = None
    if config['training']['mixed_precision']:
      if device_type == 'mps':
        logger.info("Mixed precision requested but MPS doesn't support autocast. Using full precision on MPS.")
      elif device_type == 'cpu':
        logger.info("Mixed precision requested but not available for CPU. Disabling.")
  
  # Early stopping
  early_stopping = EarlyStopping(
    patience=config['training']['early_stopping_patience'],
    restore_best_weights=True
  )
  
  # TensorBoard writer
  if config['logging']['tensorboard']:
    writer = SummaryWriter(os.path.join(exp_dir, 'tensorboard'))
  else:
    writer = None
  
  # Resume from checkpoint if specified
  start_epoch = 0
  best_val_loss = float('inf')
  train_losses = []
  val_losses = []
  
  if args.resume:
    logger.info(f"Resuming from checkpoint: {args.resume}")
    checkpoint = load_checkpoint(args.resume, model, optimizer, device)
    start_epoch = checkpoint['epoch'] + 1
    best_val_loss = checkpoint['loss']
    train_losses = checkpoint.get('train_losses', [])
    val_losses = checkpoint.get('val_losses', [])
  
  # Save experiment info
  save_experiment_info(exp_dir, config, model_info)
  
  # Training loop
  logger.info("Starting training...")
  for epoch in range(start_epoch, config['training']['num_epochs']):
    logger.info(f"Epoch {epoch+1}/{config['training']['num_epochs']}")
    
    # Train
    train_loss = train_epoch(
      model, train_loader, optimizer, criterion, device, scaler,
      use_mixed_precision, config['logging']['log_frequency']
    )
    train_losses.append(train_loss)
    
    # Validate
    if (epoch + 1) % config['validation']['val_frequency'] == 0:
      val_loss, val_metrics = validate_epoch(
        model, val_loader, criterion, device, train_dataset.get_action_stats()
      )
      val_losses.append(val_loss)
      
      logger.info(f"Epoch {epoch+1} - Train Loss: {train_loss:.4f}, Val Loss: {val_loss:.4f}")
      logger.info(f"Validation metrics: {val_metrics}")
      
      # Log to TensorBoard
      if writer:
        writer.add_scalar('Loss/Train', train_loss, epoch)
        writer.add_scalar('Loss/Validation', val_loss, epoch)
        writer.add_scalar('Learning_Rate', optimizer.param_groups[0]['lr'], epoch)
        
        for metric_name, metric_value in val_metrics.items():
          writer.add_scalar(f'Metrics/{metric_name}', metric_value, epoch)
      
      # Check for best model
      if val_loss < best_val_loss:
        best_val_loss = val_loss
        if config['checkpoint']['save_best']:
          save_checkpoint(
            model, optimizer, epoch, val_loss, val_metrics,
            os.path.join(exp_dir, config['checkpoint']['save_dir']),
            'best_model.pth'
          )
      
      # Early stopping
      if early_stopping(val_loss, model):
        logger.info(f"Early stopping triggered at epoch {epoch+1}")
        break
    else:
      val_losses.append(val_losses[-1] if val_losses else train_loss)
    
    # Update learning rate
    if scheduler:
      current_lr = scheduler.step()
      if writer:
        writer.add_scalar('Learning_Rate', current_lr, epoch)
    
    # Save checkpoint
    if (epoch + 1) % config['checkpoint']['save_frequency'] == 0:
      save_checkpoint(
        model, optimizer, epoch, train_loss, {},
        os.path.join(exp_dir, config['checkpoint']['save_dir']),
        f'checkpoint_epoch_{epoch+1:04d}.pth'
      )
    
    # Log sample predictions
    if config['logging']['log_actions'] and (epoch + 1) % config['logging']['log_frequency_images'] == 0:
      model.eval()
      with torch.no_grad():
        sample_images, sample_actions = next(iter(val_loader))
        sample_images = sample_images.to(device)
        sample_predictions = model(sample_images)
        
        # Plot and save predictions
        plot_path = os.path.join(exp_dir, f'predictions_epoch_{epoch+1:04d}.png')
        plot_action_predictions(
          sample_predictions.cpu(),
          sample_actions,
          plot_path,
          train_dataset.get_action_stats()
        )
        
        if writer:
          writer.add_image('Predictions', 
                          torch.from_numpy(plt.imread(plot_path)).permute(2, 0, 1), epoch)
  
  # Save final model
  save_checkpoint(
    model, optimizer, epoch, train_loss, {},
    os.path.join(exp_dir, config['checkpoint']['save_dir']),
    'final_model.pth'
  )
  
  # Plot training curves
  plot_path = os.path.join(exp_dir, 'training_curves.png')
  plot_training_curves(train_losses, val_losses, plot_path)
  
  # Close TensorBoard writer
  if writer:
    writer.close()
  
  logger.info("Training completed!")
  logger.info(f"Best validation loss: {best_val_loss:.4f}")
  logger.info(f"Results saved to: {exp_dir}")


if __name__ == '__main__':
  main()