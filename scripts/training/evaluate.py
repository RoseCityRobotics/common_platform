#!/usr/bin/env python3
"""
Evaluation script for maze navigation imitation learning model.

This script evaluates a trained model on the test set and generates:
- Quantitative metrics (MSE, MAE, correlation, etc.)
- Qualitative visualizations (action predictions vs ground truth)
- Action chunk analysis
- Performance summaries
"""

import os
import sys
import argparse
import yaml
import torch
import torch.nn as nn
from torch.utils.data import DataLoader
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from tqdm import tqdm
import logging
from pathlib import Path

# Add training directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from dataset import create_datasets
from model import create_model, load_pretrained_weights
from utils import (
  load_config, load_checkpoint, compute_metrics, plot_action_predictions,
  plot_action_chunks, denormalize_actions, get_device, set_seed
)


def evaluate_model(
  model: nn.Module,
  dataloader: DataLoader,
  device: torch.device,
  action_stats: dict,
  save_dir: str,
  model_name: str = "model"
) -> dict:
  """
  Evaluate model on dataset and generate comprehensive results.
  
  Args:
    model: Trained model
    dataloader: Data loader for evaluation
    device: Device to run evaluation on
    action_stats: Action normalization statistics
    save_dir: Directory to save results
    model_name: Name for saved files
  
  Returns:
    Dictionary containing evaluation metrics and results
  """
  model.eval()
  
  all_predictions = []
  all_targets = []
  all_images = []
  
  # Collect all predictions and targets
  with torch.no_grad():
    for batch_idx, (images, actions) in enumerate(tqdm(dataloader, desc="Evaluating")):
      images = images.to(device)
      actions = actions.to(device)
      
      # Get predictions
      predictions = model(images)
      
      # Store results
      all_predictions.append(predictions.cpu())
      all_targets.append(actions.cpu())
      all_images.append(images.cpu())
  
  # Concatenate all results
  all_predictions = torch.cat(all_predictions, dim=0)
  all_targets = torch.cat(all_targets, dim=0)
  all_images = torch.cat(all_images, dim=0)
  
  print(f"Evaluated on {len(all_predictions)} samples")
  
  # Compute metrics
  metrics = compute_metrics(all_predictions, all_targets, action_stats)
  
  # Print metrics
  print("\n" + "="*50)
  print("EVALUATION METRICS")
  print("="*50)
  for metric_name, metric_value in metrics.items():
    print(f"{metric_name:15s}: {metric_value:.6f}")
  print("="*50)
  
  # Generate visualizations
  os.makedirs(save_dir, exist_ok=True)
  
  # 1. Action predictions vs ground truth
  print("Generating action prediction plots...")
  plot_path = os.path.join(save_dir, f'{model_name}_action_predictions.png')
  plot_action_predictions(
    all_predictions, all_targets, plot_path, action_stats, max_samples=5
  )
  
  # 2. Action chunk analysis
  print("Generating action chunk analysis...")
  chunk_plot_path = os.path.join(save_dir, f'{model_name}_action_chunks.png')
  plot_action_chunks(
    all_predictions, all_targets, chunk_plot_path, action_stats,
    sample_idx=0, time_idx=0
  )
  
  # 3. Detailed analysis plots
  print("Generating detailed analysis...")
  generate_detailed_analysis(
    all_predictions, all_targets, action_stats, save_dir, model_name
  )
  
  # 4. Save metrics to file
  metrics_path = os.path.join(save_dir, f'{model_name}_metrics.json')
  import json
  with open(metrics_path, 'w') as f:
    json.dump(metrics, f, indent=2)
  
  # 5. Generate summary report
  generate_summary_report(metrics, save_dir, model_name)
  
  return {
    'metrics': metrics,
    'predictions': all_predictions,
    'targets': all_targets,
    'images': all_images
  }


def generate_detailed_analysis(
  predictions: torch.Tensor,
  targets: torch.Tensor,
  action_stats: dict,
  save_dir: str,
  model_name: str
):
  """Generate detailed analysis plots and statistics."""
  
  # Denormalize actions
  pred_denorm = denormalize_actions(predictions, action_stats)
  target_denorm = denormalize_actions(targets, action_stats)
  
  # Flatten for analysis
  pred_flat = pred_denorm.view(-1, 2).numpy()
  target_flat = target_denorm.view(-1, 2).numpy()
  
  # Create detailed analysis plots
  fig, axes = plt.subplots(2, 3, figsize=(18, 12))
  
  # 1. Linear velocity scatter plot
  ax = axes[0, 0]
  ax.scatter(target_flat[:, 0], pred_flat[:, 0], alpha=0.5, s=1)
  ax.plot([target_flat[:, 0].min(), target_flat[:, 0].max()],
          [target_flat[:, 0].min(), target_flat[:, 0].max()], 'r--', alpha=0.8)
  ax.set_xlabel('Target Linear Velocity (m/s)')
  ax.set_ylabel('Predicted Linear Velocity (m/s)')
  ax.set_title('Linear Velocity: Predicted vs Target')
  ax.grid(True, alpha=0.3)
  
  # 2. Angular velocity scatter plot
  ax = axes[0, 1]
  ax.scatter(target_flat[:, 1], pred_flat[:, 1], alpha=0.5, s=1)
  ax.plot([target_flat[:, 1].min(), target_flat[:, 1].max()],
          [target_flat[:, 1].min(), target_flat[:, 1].max()], 'r--', alpha=0.8)
  ax.set_xlabel('Target Angular Velocity (rad/s)')
  ax.set_ylabel('Predicted Angular Velocity (rad/s)')
  ax.set_title('Angular Velocity: Predicted vs Target')
  ax.grid(True, alpha=0.3)
  
  # 3. Error distribution - Linear
  ax = axes[0, 2]
  linear_errors = pred_flat[:, 0] - target_flat[:, 0]
  ax.hist(linear_errors, bins=50, alpha=0.7, density=True)
  ax.set_xlabel('Linear Velocity Error (m/s)')
  ax.set_ylabel('Density')
  ax.set_title(f'Linear Velocity Error Distribution\n(Mean: {linear_errors.mean():.4f}, Std: {linear_errors.std():.4f})')
  ax.grid(True, alpha=0.3)
  
  # 4. Error distribution - Angular
  ax = axes[1, 0]
  angular_errors = pred_flat[:, 1] - target_flat[:, 1]
  ax.hist(angular_errors, bins=50, alpha=0.7, density=True)
  ax.set_xlabel('Angular Velocity Error (rad/s)')
  ax.set_ylabel('Density')
  ax.set_title(f'Angular Velocity Error Distribution\n(Mean: {angular_errors.mean():.4f}, Std: {angular_errors.std():.4f})')
  ax.grid(True, alpha=0.3)
  
  # 5. Action magnitude analysis
  ax = axes[1, 1]
  target_magnitude = np.sqrt(target_flat[:, 0]**2 + target_flat[:, 1]**2)
  pred_magnitude = np.sqrt(pred_flat[:, 0]**2 + pred_flat[:, 1]**2)
  ax.scatter(target_magnitude, pred_magnitude, alpha=0.5, s=1)
  ax.plot([target_magnitude.min(), target_magnitude.max()],
          [target_magnitude.min(), target_magnitude.max()], 'r--', alpha=0.8)
  ax.set_xlabel('Target Action Magnitude')
  ax.set_ylabel('Predicted Action Magnitude')
  ax.set_title('Action Magnitude: Predicted vs Target')
  ax.grid(True, alpha=0.3)
  
  # 6. Temporal error analysis (first action of each chunk)
  ax = axes[1, 2]
  first_action_pred = pred_denorm[:, :, 0, :].view(-1, 2).numpy()
  first_action_target = target_denorm[:, :, 0, :].view(-1, 2).numpy()
  first_action_errors = np.sqrt(np.sum((first_action_pred - first_action_target)**2, axis=1))
  
    # Plot error over time (sample a subset for visualization)
  sample_indices = np.random.choice(len(first_action_errors), min(1000, len(first_action_errors)), replace=False)
  ax.plot(sample_indices, first_action_errors[sample_indices], alpha=0.7)
  ax.set_xlabel('Sample Index')
  ax.set_ylabel('Action Error (L2 norm)')
  ax.set_title('Temporal Error Analysis (First Actions)')
  ax.grid(True, alpha=0.3)
  
  plt.tight_layout()
  plt.savefig(os.path.join(save_dir, f'{model_name}_detailed_analysis.png'), 
              dpi=150, bbox_inches='tight')
  plt.close()
  
  # Generate chunk-wise analysis
  generate_chunk_analysis(pred_denorm, target_denorm, save_dir, model_name)


def generate_chunk_analysis(
  predictions: torch.Tensor,
  targets: torch.Tensor,
  save_dir: str,
  model_name: str
):
  """Generate analysis of action chunk predictions."""
  
  batch_size, seq_len, chunk_size, action_dim = predictions.shape
  
  # Compute chunk-wise metrics
  chunk_errors = []
  chunk_correlations = []
  
  for i in range(seq_len):
    pred_chunk = predictions[:, i, :, :].view(-1, action_dim).numpy()
    target_chunk = targets[:, i, :, :].view(-1, action_dim).numpy()
    
    # L2 error for each chunk
    chunk_error = np.sqrt(np.sum((pred_chunk - target_chunk)**2, axis=1))
    chunk_errors.append(chunk_error)
    
    # Correlation for each chunk
    linear_corr = np.corrcoef(pred_chunk[:, 0], target_chunk[:, 0])[0, 1]
    angular_corr = np.corrcoef(pred_chunk[:, 1], target_chunk[:, 1])[0, 1]
    chunk_correlations.append([linear_corr, angular_corr])
  
  chunk_errors = np.array(chunk_errors)  # (seq_len, batch_size)
  chunk_correlations = np.array(chunk_correlations)  # (seq_len, 2)
  
  # Plot chunk analysis
  fig, axes = plt.subplots(2, 2, figsize=(15, 10))
  
  # 1. Error vs chunk position
  ax = axes[0, 0]
  mean_errors = np.mean(chunk_errors, axis=1)
  std_errors = np.std(chunk_errors, axis=1)
  ax.errorbar(range(seq_len), mean_errors, yerr=std_errors, capsize=5)
  ax.set_xlabel('Time Step in Sequence')
  ax.set_ylabel('Mean Action Error (L2 norm)')
  ax.set_title('Action Error vs Time Step')
  ax.grid(True, alpha=0.3)
  
  # 2. Correlation vs chunk position
  ax = axes[0, 1]
  ax.plot(range(seq_len), chunk_correlations[:, 0], 'b-o', label='Linear Velocity')
  ax.plot(range(seq_len), chunk_correlations[:, 1], 'r-s', label='Angular Velocity')
  ax.set_xlabel('Time Step in Sequence')
  ax.set_ylabel('Correlation with Target')
  ax.set_title('Action Correlation vs Time Step')
  ax.legend()
  ax.grid(True, alpha=0.3)
  
  # 3. Error distribution by chunk position
  ax = axes[1, 0]
  ax.boxplot([chunk_errors[i] for i in range(0, seq_len, max(1, seq_len//10))])
  ax.set_xlabel('Time Step (sampled)')
  ax.set_ylabel('Action Error (L2 norm)')
  ax.set_title('Error Distribution by Time Step')
  ax.grid(True, alpha=0.3)
  
  # 4. Future prediction accuracy (how well do we predict further into the future?)
  ax = axes[1, 1]
  future_errors = []
  for future_step in range(chunk_size):
    pred_future = predictions[:, :, future_step, :].view(-1, action_dim).numpy()
    target_future = targets[:, :, future_step, :].view(-1, action_dim).numpy()
    future_error = np.sqrt(np.sum((pred_future - target_future)**2, axis=1))
    future_errors.append(np.mean(future_error))
  
  ax.plot(range(chunk_size), future_errors, 'g-o')
  ax.set_xlabel('Steps into Future')
  ax.set_ylabel('Mean Action Error (L2 norm)')
  ax.set_title('Prediction Accuracy vs Future Steps')
  ax.grid(True, alpha=0.3)
  
  plt.tight_layout()
  plt.savefig(os.path.join(save_dir, f'{model_name}_chunk_analysis.png'), 
              dpi=150, bbox_inches='tight')
  plt.close()


def generate_summary_report(metrics: dict, save_dir: str, model_name: str):
  """Generate a text summary report."""
  
  report_path = os.path.join(save_dir, f'{model_name}_summary_report.txt')
  
  with open(report_path, 'w') as f:
    f.write("="*60 + "\n")
    f.write("MAZE NAVIGATION MODEL EVALUATION REPORT\n")
    f.write("="*60 + "\n\n")
    
    f.write("OVERALL PERFORMANCE METRICS\n")
    f.write("-" * 30 + "\n")
    f.write(f"MSE (Mean Squared Error):     {metrics['mse']:.6f}\n")
    f.write(f"MAE (Mean Absolute Error):     {metrics['mae']:.6f}\n")
    f.write(f"RMSE (Root Mean Squared Error): {metrics['rmse']:.6f}\n\n")
    
    f.write("PER-ACTION METRICS\n")
    f.write("-" * 20 + "\n")
    f.write(f"Linear Velocity MSE:          {metrics['linear_mse']:.6f}\n")
    f.write(f"Linear Velocity MAE:          {metrics['linear_mae']:.6f}\n")
    f.write(f"Linear Velocity Correlation:   {metrics['linear_corr']:.6f}\n\n")
    
    f.write(f"Angular Velocity MSE:         {metrics['angular_mse']:.6f}\n")
    f.write(f"Angular Velocity MAE:         {metrics['angular_mae']:.6f}\n")
    f.write(f"Angular Velocity Correlation:  {metrics['angular_corr']:.6f}\n\n")
    
    f.write("INTERPRETATION\n")
    f.write("-" * 15 + "\n")
    f.write("• Lower MSE/MAE values indicate better prediction accuracy\n")
    f.write("• Correlation values closer to 1.0 indicate better linear relationship\n")
    f.write("• Linear velocity controls forward/backward movement\n")
    f.write("• Angular velocity controls rotation (left/right turning)\n")
    
    # Performance assessment
    f.write("\nPERFORMANCE ASSESSMENT\n")
    f.write("-" * 22 + "\n")
    
    if metrics['mse'] < 0.01:
      f.write("• EXCELLENT: Very low prediction error\n")
    elif metrics['mse'] < 0.05:
      f.write("• GOOD: Low prediction error\n")
    elif metrics['mse'] < 0.1:
      f.write("• FAIR: Moderate prediction error\n")
    else:
      f.write("• POOR: High prediction error - consider more training or data\n")
    
    if metrics['linear_corr'] > 0.9:
      f.write("• EXCELLENT: Very strong linear velocity correlation\n")
    elif metrics['linear_corr'] > 0.7:
      f.write("• GOOD: Strong linear velocity correlation\n")
    elif metrics['linear_corr'] > 0.5:
      f.write("• FAIR: Moderate linear velocity correlation\n")
    else:
      f.write("• POOR: Weak linear velocity correlation\n")
    
    if metrics['angular_corr'] > 0.9:
      f.write("• EXCELLENT: Very strong angular velocity correlation\n")
    elif metrics['angular_corr'] > 0.7:
      f.write("• GOOD: Strong angular velocity correlation\n")
    elif metrics['angular_corr'] > 0.5:
      f.write("• FAIR: Moderate angular velocity correlation\n")
    else:
      f.write("• POOR: Weak angular velocity correlation\n")
    
    f.write("\n" + "="*60 + "\n")


def main():
  parser = argparse.ArgumentParser(description='Evaluate maze navigation model')
  parser.add_argument('--config', type=str, default='config.yaml',
                     help='Path to configuration file')
  parser.add_argument('--checkpoint', type=str, required=True,
                     help='Path to model checkpoint')
  parser.add_argument('--data_root', type=str, default=None,
                     help='Override data root directory')
  parser.add_argument('--output_dir', type=str, default='evaluation_results',
                     help='Output directory for evaluation results')
  parser.add_argument('--device', type=str, default=None,
                     help='Override device (cpu, cuda, mps)')
  parser.add_argument('--batch_size', type=int, default=None,
                     help='Override batch size')
  parser.add_argument('--num_samples', type=int, default=None,
                     help='Number of samples to evaluate (None for all)')
  
  args = parser.parse_args()
  
  # Load configuration
  config = load_config(args.config)
  
  # Override config with command line arguments
  if args.data_root:
    config['dataset']['data_root'] = args.data_root
  if args.device:
    config['device'] = args.device
  if args.batch_size:
    config['training']['batch_size'] = args.batch_size
  
  # Set random seed
  set_seed(config['seed'])
  
  # Get device
  if config['device'] == 'auto':
    device = get_device()
  else:
    device = torch.device(config['device'])
  
  print(f"Using device: {device}")
  
  # Create output directory
  os.makedirs(args.output_dir, exist_ok=True)
  
  # Create datasets
  print("Loading test dataset...")
  _, _, test_dataset = create_datasets(
    data_root=config['dataset']['data_root'],
    train_ratio=config['dataset']['train_ratio'],
    val_ratio=config['dataset']['val_ratio'],
    test_ratio=config['dataset']['test_ratio'],
    sequence_length=config['dataset']['sequence_length'],
    chunk_size=config['dataset']['chunk_size'],
    stride=config['dataset']['stride'],
    image_size=tuple(config['dataset']['image_size']),
    augment=False  # No augmentation for evaluation
  )
  
  # Limit number of samples if specified
  if args.num_samples:
    indices = torch.randperm(len(test_dataset))[:args.num_samples]
    test_dataset = torch.utils.data.Subset(test_dataset, indices)
  
  # Create data loader
  test_loader = DataLoader(
    test_dataset,
    batch_size=config['training']['batch_size'],
    shuffle=False,
    num_workers=config['dataloader']['num_workers'],
    pin_memory=config['dataloader']['pin_memory']
  )
  
  # Create model
  print("Loading model...")
  model = create_model(config)
  model = model.to(device)
  
  # Load checkpoint
  print(f"Loading checkpoint from {args.checkpoint}")
  checkpoint = load_checkpoint(args.checkpoint, model, device=device)
  
  # Get model name from checkpoint path
  model_name = os.path.splitext(os.path.basename(args.checkpoint))[0]
  
  print(f"Model info: {model.get_model_info()}")
  
  # Evaluate model
  print("Starting evaluation...")
  results = evaluate_model(
    model, test_loader, device, test_dataset.get_action_stats(),
    args.output_dir, model_name
  )
  
  print(f"\nEvaluation completed! Results saved to: {args.output_dir}")
  print(f"Summary report: {os.path.join(args.output_dir, f'{model_name}_summary_report.txt')}")


if __name__ == '__main__':
  main()