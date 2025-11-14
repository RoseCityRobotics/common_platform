#!/usr/bin/env python3
"""
Transfer learning utilities for loading ALOHA pretrained weights.

This module provides utilities to:
- Load ALOHA pretrained model weights
- Map weights between ALOHA and our model architectures
- Set up fine-tuning configurations
- Freeze/unfreeze specific layers
"""

import os
import torch
import torch.nn as nn
from typing import Dict, Any, Optional, List, Tuple
import logging
from pathlib import Path


def load_aloha_checkpoint(checkpoint_path: str, device: str = 'cpu') -> Dict[str, Any]:
  """
  Load ALOHA checkpoint and return state dict.
  
  Args:
    checkpoint_path: Path to ALOHA checkpoint file
    device: Device to load on
  
  Returns:
    State dictionary from checkpoint
  """
  if not os.path.exists(checkpoint_path):
    raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")
  
  checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
  
  # Handle different checkpoint formats
  if 'model_state_dict' in checkpoint:
    state_dict = checkpoint['model_state_dict']
  elif 'state_dict' in checkpoint:
    state_dict = checkpoint['state_dict']
  elif 'model' in checkpoint:
    state_dict = checkpoint['model']
  else:
    state_dict = checkpoint
  
  return state_dict


def map_aloha_vision_weights(
  aloha_state_dict: Dict[str, torch.Tensor],
  our_model: nn.Module,
  camera_idx: int = 0,
  strict: bool = False
) -> Dict[str, torch.Tensor]:
  """
  Map ALOHA vision encoder weights to our model.
  
  ALOHA has 4 camera streams, we use 1. This function maps one of ALOHA's
  camera streams to our single vision encoder.
  
  Args:
    aloha_state_dict: ALOHA model state dictionary
    our_model: Our model to load weights into
    camera_idx: Which ALOHA camera to use (0-3)
    strict: Whether to strictly enforce key matching
  
  Returns:
    Mapped state dictionary for our model
  """
  our_state_dict = our_model.state_dict()
  mapped_state_dict = {}
  
  # ALOHA vision encoder pattern: vision_encoder.{camera_idx}.backbone.*
  aloha_prefix = f'vision_encoder.{camera_idx}.'
  our_prefix = 'vision_encoder.'
  
  vision_transferred = 0
  
  for aloha_key, value in aloha_state_dict.items():
    if aloha_key.startswith(aloha_prefix):
      # Convert ALOHA key to our key
      our_key = aloha_key.replace(aloha_prefix, our_prefix)
      
      if our_key in our_state_dict:
        if value.shape == our_state_dict[our_key].shape:
          mapped_state_dict[our_key] = value
          vision_transferred += 1
          logging.info(f"Mapped {aloha_key} -> {our_key}")
        else:
          logging.warning(f"Shape mismatch: {aloha_key} {value.shape} vs {our_key} {our_state_dict[our_key].shape}")
      else:
        logging.debug(f"No matching key for {aloha_key} -> {our_key}")
  
  logging.info(f"Vision encoder: {vision_transferred} weights transferred from camera {camera_idx}")
  return mapped_state_dict


def map_aloha_transformer_weights(
  aloha_state_dict: Dict[str, torch.Tensor],
  our_model: nn.Module,
  strict: bool = False
) -> Dict[str, torch.Tensor]:
  """
  Map ALOHA transformer weights to our model.
  
  This maps the temporal encoder (self-attention) but NOT the action decoder
  since ALOHA outputs 14-DOF manipulation actions while we output 2-DOF navigation.
  
  Args:
    aloha_state_dict: ALOHA model state dictionary
    our_model: Our model to load weights into
    strict: Whether to strictly enforce key matching
  
  Returns:
    Mapped state dictionary for our model
  """
  our_state_dict = our_model.state_dict()
  mapped_state_dict = {}
  
  encoder_transferred = 0
  
  # Map temporal encoder weights (direct match)
  for aloha_key, value in aloha_state_dict.items():
    if aloha_key.startswith('temporal_encoder.'):
      if aloha_key in our_state_dict:
        if value.shape == our_state_dict[aloha_key].shape:
          mapped_state_dict[aloha_key] = value
          encoder_transferred += 1
          logging.info(f"Mapped {aloha_key}")
        else:
          logging.warning(f"Shape mismatch: {aloha_key} {value.shape} vs {our_state_dict[aloha_key].shape}")
      else:
        logging.debug(f"No matching key for {aloha_key}")
  
  # Note: We deliberately skip action_decoder and action_head weights
  # because ALOHA outputs 14-DOF manipulation actions while we output 2-DOF navigation
  logging.info(f"Temporal encoder: {encoder_transferred} weights transferred")
  logging.info("Action decoder: Skipped (different output space: 14-DOF -> 2-DOF)")
  
  return mapped_state_dict


def load_aloha_weights(
  model: nn.Module,
  aloha_checkpoint_path: str,
  device: str = 'cpu',
  load_vision: bool = True,
  load_transformer: bool = True,
  camera_idx: int = 0,
  strict: bool = False
) -> nn.Module:
  """
  Load ALOHA pretrained weights into our model.
  
  Args:
    model: Our model to load weights into
    aloha_checkpoint_path: Path to ALOHA checkpoint
    device: Device to load on
    load_vision: Whether to load vision encoder weights
    load_transformer: Whether to load transformer weights
    camera_idx: Which ALOHA camera to use (0-3)
    strict: Whether to strictly enforce key matching
  
  Returns:
    Model with loaded weights
  """
  logging.info(f"Loading ALOHA weights from {aloha_checkpoint_path}")
  
  # Load ALOHA checkpoint
  aloha_state_dict = load_aloha_checkpoint(aloha_checkpoint_path, device)
  
  # Get our model's state dict
  our_state_dict = model.state_dict()
  mapped_state_dict = {}
  
  # Map vision encoder weights (from specified camera)
  if load_vision:
    vision_mapped = map_aloha_vision_weights(aloha_state_dict, model, camera_idx, strict)
    mapped_state_dict.update(vision_mapped)
  
  # Map transformer weights (temporal encoder only)
  if load_transformer:
    transformer_mapped = map_aloha_transformer_weights(aloha_state_dict, model, strict)
    mapped_state_dict.update(transformer_mapped)
  
  # Load mapped weights
  if mapped_state_dict:
    our_state_dict.update(mapped_state_dict)
    model.load_state_dict(our_state_dict, strict=False)
    logging.info(f"Loaded {len(mapped_state_dict)} weights from ALOHA")
  else:
    logging.warning("No weights were mapped from ALOHA checkpoint")
  
  return model


def transfer_aloha_weights_practical(
  model: nn.Module,
  aloha_checkpoint_path: str,
  device: str = 'cpu',
  camera_idx: int = 0
) -> nn.Module:
  """
  Practical ALOHA weight transfer for differential drive robot.
  
  This is a simplified, robust version that handles the key differences:
  - ALOHA: 4 cameras -> Our model: 1 camera
  - ALOHA: 14-DOF actions -> Our model: 2-DOF actions
  
  Args:
    model: Our navigation model
    aloha_checkpoint_path: Path to ALOHA checkpoint
    device: Device to load on
    camera_idx: Which ALOHA camera to use (0-3)
  
  Returns:
    Model with transferred weights
  """
  print("Loading ALOHA checkpoint...")
  checkpoint = torch.load(aloha_checkpoint_path, map_location=device, weights_only=False)
  
  if 'model_state_dict' in checkpoint:
    aloha_state = checkpoint['model_state_dict']
  else:
    aloha_state = checkpoint
  
  our_state = model.state_dict()
  transferred_weights = {}
  
  # 1. Transfer vision encoder (use specified camera stream)
  print(f"Transferring vision encoder from ALOHA camera {camera_idx}...")
  vision_transferred = 0
  aloha_vision_prefix = f'vision_encoder.{camera_idx}.'
  our_vision_prefix = 'vision_encoder.'
  
  for key, value in aloha_state.items():
    if key.startswith(aloha_vision_prefix):
      our_key = key.replace(aloha_vision_prefix, our_vision_prefix)
      if our_key in our_state and value.shape == our_state[our_key].shape:
        transferred_weights[our_key] = value
        vision_transferred += 1
      else:
        print(f"  Skipping {key} -> {our_key} (shape mismatch)")
  
  # 2. Transfer temporal encoder (direct match)
  print("Transferring temporal encoder...")
  encoder_transferred = 0
  for key, value in aloha_state.items():
    if key.startswith('temporal_encoder.'):
      if key in our_state and value.shape == our_state[key].shape:
        transferred_weights[key] = value
        encoder_transferred += 1
      else:
        print(f"  Skipping {key} (shape mismatch)")
  
  # 3. Load transferred weights
  model.load_state_dict(transferred_weights, strict=False)
  
  print(f"Transfer complete:")
  print(f"  Vision encoder: {vision_transferred} weights transferred from camera {camera_idx}")
  print(f"  Temporal encoder: {encoder_transferred} weights transferred")
  print(f"  Action decoder: Random initialization (different output space)")
  
  return model


def create_fine_tuning_config(
  base_config: Dict[str, Any],
  aloha_checkpoint_path: str,
  freeze_vision: bool = True,
  freeze_encoder: bool = False,
  freeze_decoder: bool = False,
  fine_tune_lr: float = 1e-5
) -> Dict[str, Any]:
  """
  Create fine-tuning configuration from base config.
  
  Args:
    base_config: Base training configuration
    aloha_checkpoint_path: Path to ALOHA checkpoint
    freeze_vision: Whether to freeze vision encoder
    freeze_encoder: Whether to freeze temporal encoder
    freeze_decoder: Whether to freeze action decoder
    fine_tune_lr: Learning rate for fine-tuning
  
  Returns:
    Updated configuration for fine-tuning
  """
  config = base_config.copy()
  
  # Enable transfer learning
  config['transfer_learning']['enabled'] = True
  config['transfer_learning']['pretrained_path'] = aloha_checkpoint_path
  config['transfer_learning']['freeze_vision'] = freeze_vision
  config['transfer_learning']['freeze_encoder'] = freeze_encoder
  config['transfer_learning']['freeze_decoder'] = freeze_decoder
  config['transfer_learning']['fine_tune_lr'] = fine_tune_lr
  
  # Adjust learning rate for fine-tuning
  if freeze_vision or freeze_encoder or freeze_decoder:
    config['training']['learning_rate'] = fine_tune_lr
  
  # Reduce early stopping patience for fine-tuning
  config['training']['early_stopping_patience'] = 5
  
  # Reduce number of epochs for fine-tuning
  config['training']['num_epochs'] = min(50, config['training']['num_epochs'])
  
  return config


def analyze_aloha_checkpoint(checkpoint_path: str) -> Dict[str, Any]:
  """
  Analyze ALOHA checkpoint to understand its structure.
  
  Args:
    checkpoint_path: Path to ALOHA checkpoint
  
  Returns:
    Analysis results
  """
  state_dict = load_aloha_checkpoint(checkpoint_path)
  
  analysis = {
    'total_keys': len(state_dict),
    'vision_keys': [],
    'transformer_keys': [],
    'other_keys': [],
    'key_shapes': {}
  }
  
  for key, tensor in state_dict.items():
    analysis['key_shapes'][key] = list(tensor.shape)
    
    if 'backbone' in key or 'vision' in key or 'encoder' in key.lower():
      analysis['vision_keys'].append(key)
    elif 'transformer' in key or 'attention' in key or 'decoder' in key:
      analysis['transformer_keys'].append(key)
    else:
      analysis['other_keys'].append(key)
  
  return analysis


def print_checkpoint_analysis(checkpoint_path: str):
  """Print analysis of ALOHA checkpoint."""
  analysis = analyze_aloha_checkpoint(checkpoint_path)
  
  print(f"ALOHA Checkpoint Analysis: {checkpoint_path}")
  print("=" * 50)
  print(f"Total parameters: {analysis['total_keys']}")
  print(f"Vision-related keys: {len(analysis['vision_keys'])}")
  print(f"Transformer-related keys: {len(analysis['transformer_keys'])}")
  print(f"Other keys: {len(analysis['other_keys'])}")
  
  print("\nVision Keys:")
  for key in analysis['vision_keys'][:10]:  # Show first 10
    print(f"  {key}: {analysis['key_shapes'][key]}")
  if len(analysis['vision_keys']) > 10:
    print(f"  ... and {len(analysis['vision_keys']) - 10} more")
  
  print("\nTransformer Keys:")
  for key in analysis['transformer_keys'][:10]:  # Show first 10
    print(f"  {key}: {analysis['key_shapes'][key]}")
  if len(analysis['transformer_keys']) > 10:
    print(f"  ... and {len(analysis['transformer_keys']) - 10} more")
  
  print("\nOther Keys:")
  for key in analysis['other_keys'][:10]:  # Show first 10
    print(f"  {key}: {analysis['key_shapes'][key]}")
  if len(analysis['other_keys']) > 10:
    print(f"  ... and {len(analysis['other_keys']) - 10} more")


def main():
  """Command line interface for transfer learning utilities."""
  import argparse
  
  parser = argparse.ArgumentParser(description='Transfer learning utilities')
  parser.add_argument('--checkpoint', type=str, required=True,
                     help='Path to ALOHA checkpoint')
  parser.add_argument('--analyze', action='store_true',
                     help='Analyze checkpoint structure')
  parser.add_argument('--create-config', type=str, default=None,
                     help='Create fine-tuning config and save to file')
  parser.add_argument('--base-config', type=str, default='config.yaml',
                     help='Base configuration file')
  
  args = parser.parse_args()
  
  if args.analyze:
    print_checkpoint_analysis(args.checkpoint)
  
  if args.create_config:
    from utils import load_config, save_config
    
    base_config = load_config(args.base_config)
    fine_tune_config = create_fine_tuning_config(
      base_config, args.checkpoint,
      freeze_vision=True,
      freeze_encoder=False,
      freeze_decoder=False,
      fine_tune_lr=1e-5
    )
    
    save_config(fine_tune_config, args.create_config)
    print(f"Fine-tuning configuration saved to: {args.create_config}")


if __name__ == '__main__':
  main()


