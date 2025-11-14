#!/usr/bin/env python3
"""
Example usage of the maze navigation training system.

This script demonstrates how to:
1. Create and configure datasets
2. Create and inspect models
3. Run training with different configurations
4. Evaluate trained models
5. Use transfer learning
"""

import os
import sys
import yaml
import torch
from pathlib import Path

# Add training directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from dataset import create_datasets
from model import create_model, load_pretrained_weights
from utils import get_device, count_parameters, load_config


def example_dataset_creation():
  """Example: Create and inspect datasets."""
  print("=" * 50)
  print("EXAMPLE: Dataset Creation")
  print("=" * 50)
  
  # Load configuration
  config = load_config('config.yaml')
  
  # Create datasets
  print("Creating datasets...")
  train_dataset, val_dataset, test_dataset = create_datasets(
    data_root=config['dataset']['data_root'],
    sequence_length=config['dataset']['sequence_length'],
    chunk_size=config['dataset']['chunk_size'],
    image_size=tuple(config['dataset']['image_size'])
  )
  
  print(f"Train dataset: {len(train_dataset)} sequences")
  print(f"Val dataset: {len(val_dataset)} sequences")
  print(f"Test dataset: {len(test_dataset)} sequences")
  
  # Inspect a sample
  print("\nInspecting a sample...")
  images, actions = train_dataset[0]
  print(f"Image shape: {images.shape}")
  print(f"Actions shape: {actions.shape}")
  print(f"Action stats: {train_dataset.get_action_stats()}")
  
  return train_dataset, val_dataset, test_dataset


def example_model_creation():
  """Example: Create and inspect model."""
  print("\n" + "=" * 50)
  print("EXAMPLE: Model Creation")
  print("=" * 50)
  
  # Load configuration
  config = load_config('config.yaml')
  
  # Create model
  print("Creating model...")
  model = create_model(config)
  
  # Get device
  device = get_device()
  model = model.to(device)
  
  # Print model info
  model_info = model.get_model_info()
  print(f"Model info: {model_info}")
  print(f"Trainable parameters: {count_parameters(model):,}")
  
  # Test forward pass
  print("\nTesting forward pass...")
  batch_size = 2
  seq_len = config['dataset']['sequence_length']
  image_size = config['dataset']['image_size']
  
  dummy_images = torch.randn(batch_size, seq_len, 3, image_size[0], image_size[1])
  dummy_images = dummy_images.to(device)
  
  with torch.no_grad():
    predictions = model(dummy_images)
  
  print(f"Input shape: {dummy_images.shape}")
  print(f"Output shape: {predictions.shape}")
  
  return model


def example_training_config():
  """Example: Create custom training configuration."""
  print("\n" + "=" * 50)
  print("EXAMPLE: Custom Training Configuration")
  print("=" * 50)
  
  # Load base config
  config = load_config('config.yaml')
  
  # Modify for quick training
  config['training']['num_epochs'] = 5
  config['training']['batch_size'] = 4
  config['dataset']['sequence_length'] = 5
  config['dataset']['chunk_size'] = 10
  
  # Save custom config
  custom_config_path = 'example_config.yaml'
  with open(custom_config_path, 'w') as f:
    yaml.dump(config, f, default_flow_style=False, indent=2)
  
  print(f"Custom config saved to: {custom_config_path}")
  print("Key changes:")
  print(f"  - Epochs: {config['training']['num_epochs']}")
  print(f"  - Batch size: {config['training']['batch_size']}")
  print(f"  - Sequence length: {config['dataset']['sequence_length']}")
  print(f"  - Chunk size: {config['dataset']['chunk_size']}")
  
  return custom_config_path


def example_transfer_learning():
  """Example: Transfer learning setup."""
  print("\n" + "=" * 50)
  print("EXAMPLE: Transfer Learning Setup")
  print("=" * 50)
  
  try:
    from transfer_learning import create_fine_tuning_config, print_checkpoint_analysis
    
    # Load base config
    config = load_config('config.yaml')
    
    # Simulate ALOHA checkpoint path
    aloha_checkpoint = "pretrained/aloha_model.pth"
    
    print(f"Creating fine-tuning config for: {aloha_checkpoint}")
    
    # Create fine-tuning configuration
    fine_tune_config = create_fine_tuning_config(
      config,
      aloha_checkpoint,
      freeze_vision=True,      # Freeze vision encoder
      freeze_encoder=False,    # Train temporal encoder
      freeze_decoder=False,    # Train action decoder
      fine_tune_lr=1e-5        # Lower learning rate
    )
    
    # Save fine-tuning config
    fine_tune_path = 'example_finetune_config.yaml'
    with open(fine_tune_path, 'w') as f:
      yaml.dump(fine_tune_config, f, default_flow_style=False, indent=2)
    
    print(f"Fine-tuning config saved to: {fine_tune_path}")
    print("Transfer learning settings:")
    print(f"  - Freeze vision: {fine_tune_config['transfer_learning']['freeze_vision']}")
    print(f"  - Freeze encoder: {fine_tune_config['transfer_learning']['freeze_encoder']}")
    print(f"  - Freeze decoder: {fine_tune_config['transfer_learning']['freeze_decoder']}")
    print(f"  - Fine-tune LR: {fine_tune_config['transfer_learning']['fine_tune_lr']}")
    
    return fine_tune_path
    
  except ImportError:
    print("Transfer learning module not available")
    return None


def example_evaluation():
  """Example: Model evaluation setup."""
  print("\n" + "=" * 50)
  print("EXAMPLE: Model Evaluation")
  print("=" * 50)
  
  # Simulate checkpoint path
  checkpoint_path = "checkpoints/best_model.pth"
  
  print("Evaluation command examples:")
  print(f"# Basic evaluation")
  print(f"python evaluate.py --checkpoint {checkpoint_path}")
  print()
  print(f"# Evaluation with custom settings")
  print(f"python evaluate.py --checkpoint {checkpoint_path} --batch_size 32 --num_samples 1000")
  print()
  print(f"# Evaluation with custom output directory")
  print(f"python evaluate.py --checkpoint {checkpoint_path} --output_dir my_results/")
  
  print("\nEvaluation will generate:")
  print("  - Action prediction plots")
  print("  - Action chunk analysis")
  print("  - Detailed error analysis")
  print("  - Quantitative metrics")
  print("  - Summary report")


def main():
  """Run all examples."""
  print("Maze Navigation Training System - Usage Examples")
  print("=" * 60)
  
  # Check if config exists
  if not os.path.exists('config.yaml'):
    print("Error: config.yaml not found. Please create it first.")
    return
  
  try:
    # Example 1: Dataset creation
    train_dataset, val_dataset, test_dataset = example_dataset_creation()
    
    # Example 2: Model creation
    model = example_model_creation()
    
    # Example 3: Custom training config
    custom_config = example_training_config()
    
    # Example 4: Transfer learning
    fine_tune_config = example_transfer_learning()
    
    # Example 5: Evaluation
    example_evaluation()
    
    print("\n" + "=" * 60)
    print("EXAMPLES COMPLETED")
    print("=" * 60)
    print("\nNext steps:")
    print("1. Collect data using data_recorder package")
    print("2. Update config.yaml with your data path")
    print("3. Run training: python train.py")
    print("4. Evaluate model: python evaluate.py --checkpoint checkpoints/best_model.pth")
    
    if fine_tune_config:
      print("5. Try transfer learning: python train.py --config example_finetune_config.yaml")
    
  except Exception as e:
    print(f"Error running examples: {e}")
    print("Make sure you have the required dependencies installed:")
    print("pip install -r requirements.txt")


if __name__ == '__main__':
  main()


