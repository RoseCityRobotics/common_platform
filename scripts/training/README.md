# Maze Navigation Imitation Learning

This directory contains a complete PyTorch training pipeline for robot maze navigation using imitation learning with a transformer-based Action Chunking architecture adapted from ALOHA.

## Overview

The system trains a neural network to predict robot actions (linear and angular velocities) from sequences of camera images. The model uses:

- **Vision Encoder**: ResNet/EfficientNet backbone for image feature extraction
- **Temporal Encoder**: Transformer encoder for sequence processing
- **Action Decoder**: Transformer decoder with learned queries for action chunking

## Quick Start

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

### 2. Prepare Data

Ensure your data is collected using the `data_recorder` package and organized as:

```
/home/rcr/teleop_data/
├── session_20240101_120000/
│   ├── images/
│   │   ├── 00000000.jpg
│   │   ├── 00000001.jpg
│   │   └── ...
│   └── metadata/
│       └── data_log.csv
├── session_20240101_130000/
└── ...
```

### 3. Train Model

```bash
# Basic training
python train.py --config config.yaml

# With custom data path
python train.py --config config.yaml --data_root /path/to/your/data

# Resume from checkpoint
python train.py --config config.yaml --resume checkpoints/checkpoint_epoch_0100.pth
```

### 4. Evaluate Model

```bash
# Evaluate on test set
python evaluate.py --checkpoint checkpoints/best_model.pth --config config.yaml

# With custom output directory
python evaluate.py --checkpoint checkpoints/best_model.pth --output_dir results/
```

## Configuration

Edit `config.yaml` to customize training:

### Key Parameters

- **Dataset**: Data paths, sequence length, chunk size
- **Model**: Architecture, backbone, transformer layers
- **Training**: Learning rate, batch size, epochs, optimization
- **Transfer Learning**: ALOHA weight loading, layer freezing

### Example Configuration

```yaml
dataset:
  data_root: "/home/rcr/teleop_data"
  sequence_length: 10      # Number of input frames
  chunk_size: 15           # Actions to predict per frame
  image_size: [224, 224]   # Input image size

model:
  backbone: "resnet18"     # Vision backbone
  d_model: 512            # Transformer dimension
  chunk_size: 15          # Action chunk size

training:
  batch_size: 16
  learning_rate: 1e-4
  num_epochs: 100
```

## Transfer Learning

### Load ALOHA Pretrained Weights

```bash
# Analyze ALOHA checkpoint
python transfer_learning.py --checkpoint aloha_model.pth --analyze

# Create fine-tuning config
python transfer_learning.py --checkpoint aloha_model.pth --create-config fine_tune_config.yaml

# Train with ALOHA weights
python train.py --config fine_tune_config.yaml --pretrained aloha_model.pth
```

### Fine-tuning Options

- **Freeze Vision**: Keep pretrained vision encoder frozen
- **Freeze Encoder**: Keep temporal encoder frozen
- **Freeze Decoder**: Keep action decoder frozen
- **Full Fine-tuning**: Train all layers with lower learning rate

## Data Format

The system expects data in the format produced by `data_recorder_node.cpp`:

### CSV Format (`data_log.csv`)
```csv
timestamp,image_file,linear_vel,angular_vel,position_x,position_y,orientation_z,cmd_linear_x,cmd_angular_z
1234567890,00000000.jpg,0.1,0.05,1.0,2.0,0.1,0.2,0.1
```

### Image Format
- **Format**: JPG files in `images/` subdirectory
- **Size**: 640x480 (resized to 224x224 during training)
- **Color**: RGB (converted from BGR by data recorder)

## Model Architecture

### Action Chunking Transformer

```
Input: (batch, seq_len, 3, H, W) - Image sequences
  ↓
Vision Encoder: ResNet/EfficientNet
  ↓
Features: (seq_len, batch, d_model)
  ↓
Temporal Encoder: Transformer layers
  ↓
Contextualized Features: (seq_len, batch, d_model)
  ↓
Action Decoder: Cross-attention with learned queries
  ↓
Output: (batch, seq_len, chunk_size, 2) - Action chunks
```

### Key Features

- **Temporal Processing**: Handles variable-length sequences
- **Action Chunking**: Predicts multiple future actions per frame
- **Transfer Learning**: Compatible with ALOHA pretrained weights
- **Efficient Training**: Mixed precision, gradient clipping, early stopping

## Training Process

### 1. Data Loading
- Loads multiple session directories
- Creates temporal windows of consecutive frames
- Applies data augmentation (training only)
- Normalizes actions using training statistics

### 2. Model Training
- MSE loss between predicted and target action chunks
- AdamW optimizer with cosine annealing
- Mixed precision training for efficiency
- Gradient clipping for stability

### 3. Validation
- Evaluates on held-out validation set
- Computes comprehensive metrics
- Early stopping based on validation loss
- Saves best model checkpoint

### 4. Evaluation
- Tests on held-out test set
- Generates detailed analysis plots
- Computes correlation and error metrics
- Creates summary reports

## Output Files

### Training Outputs
```
outputs/experiment_20240101_120000/
├── config.yaml                    # Training configuration
├── experiment_info.json           # Model and experiment info
├── checkpoints/                   # Model checkpoints
│   ├── best_model.pth
│   ├── checkpoint_epoch_0050.pth
│   └── final_model.pth
├── tensorboard/                   # TensorBoard logs
├── training_curves.png            # Loss curves
└── predictions_epoch_*.png        # Sample predictions
```

### Evaluation Outputs
```
evaluation_results/
├── model_action_predictions.png   # Prediction vs target plots
├── model_action_chunks.png        # Action chunk analysis
├── model_detailed_analysis.png    # Comprehensive analysis
├── model_chunk_analysis.png       # Temporal analysis
├── model_metrics.json             # Quantitative metrics
└── model_summary_report.txt       # Human-readable summary
```

## Usage Examples

### Basic Training
```bash
# Train with default settings
python train.py

# Train with custom config
python train.py --config my_config.yaml

# Train with specific data path
python train.py --data_root /path/to/data --output_dir /path/to/output
```

### Transfer Learning
```bash
# Analyze ALOHA checkpoint
python transfer_learning.py --checkpoint aloha_weights.pth --analyze

# Create fine-tuning configuration
python transfer_learning.py --checkpoint aloha_weights.pth --create-config aloha_finetune.yaml

# Train with ALOHA weights
python train.py --config aloha_finetune.yaml --pretrained aloha_weights.pth
```

### Evaluation
```bash
# Evaluate best model
python evaluate.py --checkpoint checkpoints/best_model.pth

# Evaluate with custom settings
python evaluate.py --checkpoint checkpoints/best_model.pth --batch_size 32 --num_samples 1000
```

### Resume Training
```bash
# Resume from specific checkpoint
python train.py --resume checkpoints/checkpoint_epoch_0050.pth

# Resume with different config
python train.py --config new_config.yaml --resume checkpoints/best_model.pth
```

## Troubleshooting

### Common Issues

1. **CUDA Out of Memory**
   - Reduce batch size in config
   - Use mixed precision training
   - Reduce image size or sequence length

2. **No Data Found**
   - Check data_root path in config
   - Ensure session directories exist
   - Verify CSV files have correct format

3. **Poor Performance**
   - Increase training data
   - Adjust learning rate
   - Try different backbone (ResNet34, EfficientNet)
   - Use transfer learning from ALOHA

4. **Slow Training**
   - Increase num_workers in dataloader
   - Use mixed precision training
   - Reduce image size
   - Use GPU if available

### Debug Commands

```bash
# Check data loading
python -c "from dataset import create_datasets; train, val, test = create_datasets('/path/to/data'); print(f'Train: {len(train)}, Val: {len(val)}, Test: {len(test)}')"

# Check model architecture
python -c "from model import create_model; import yaml; config = yaml.safe_load(open('config.yaml')); model = create_model(config); print(model.get_model_info())"

# Analyze checkpoint
python transfer_learning.py --checkpoint your_checkpoint.pth --analyze
```

## Performance Tips

1. **Data Quality**: Ensure high-quality teleoperation data
2. **Data Quantity**: Collect at least 50+ sessions for good performance
3. **Augmentation**: Use data augmentation to improve generalization
4. **Transfer Learning**: Start with ALOHA pretrained weights
5. **Hyperparameter Tuning**: Experiment with learning rates and model sizes
6. **Regularization**: Use dropout and weight decay to prevent overfitting

## Integration with Robot

After training, integrate the model with your robot:

```python
import torch
from model import create_model, load_pretrained_weights

# Load trained model
model = create_model(config)
model = load_pretrained_weights(model, 'checkpoints/best_model.pth')
model.eval()

# Process robot camera feed
def predict_action(image_sequence):
    with torch.no_grad():
        actions = model.predict_actions(image_sequence, use_first_action_only=True)
        return actions[0, -1, :]  # Return last frame's first action
```

## Contributing

1. Follow the existing code structure
2. Add comprehensive docstrings
3. Include type hints
4. Test with small datasets first
5. Update documentation for new features


