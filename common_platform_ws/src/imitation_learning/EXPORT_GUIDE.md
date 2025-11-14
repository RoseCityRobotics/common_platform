# Model Export Parameter Guide

This guide explains how to choose the correct parameters when exporting your trained model to ONNX format using `export_to_onnx.py`.

## Quick Answer

**For most cases, you can use the defaults and let the script read from your config file:**

```bash
python export_to_onnx.py \
  --checkpoint outputs/experiment/checkpoints/best_model.pth \
  --config outputs/experiment/config.yaml \
  --output model.onnx
```

The script will automatically read `sequence_length` and `image_size` from your config file.

## Parameter Details

### `--batch-size` (Default: 1)

**Always use 1 for inference.**

- **Training**: Uses larger batch sizes (e.g., 64) for efficiency
- **Inference**: Always uses batch size 1 since we process one sequence at a time in real-time
- **ONNX Export**: Should match inference batch size (1)
- **Hailo Compilation**: Will be fixed to 1 in the compiled model

**Recommendation**: Don't specify this parameter - let it default to 1.

### `--seq-len` (Default: Read from config)

**Must match your training `sequence_length`.**

- **Training**: Defined in `config.yaml` as `dataset.sequence_length` (default: 10)
- **Inference**: The ROS2 node uses the same `sequence_length` parameter
- **ONNX Export**: Must match what the model was trained with

**How to find the correct value:**
1. Check your training config file: `outputs/experiment/config.yaml`
2. Look for `dataset.sequence_length` (typically 10)
3. Or check the ROS2 node parameter: `sequence_length` (default: 10)

**Recommendation**: Don't specify this parameter - let the script read it from your config file automatically.

### `--height` and `--width` (Default: Read from config)

**Must match your training `image_size`.**

- **Training**: Defined in `config.yaml` as `dataset.image_size` (default: [224, 224])
- **Inference**: The ROS2 node uses `input_width` and `input_height` parameters
- **ONNX Export**: Must match what the model was trained with

**How to find the correct values:**
1. Check your training config file: `outputs/experiment/config.yaml`
2. Look for `dataset.image_size` (typically [224, 224])
3. Or check the ROS2 node parameters: `input_width` and `input_height` (default: 224)

**Recommendation**: Don't specify these parameters - let the script read them from your config file automatically.

## Examples

### Example 1: Using Config Defaults (Recommended)

```bash
# Script automatically reads from config.yaml
python export_to_onnx.py \
  --checkpoint outputs/maze_navigation_20251113_220036/checkpoints/best_model.pth \
  --config outputs/maze_navigation_20251113_220036/config.yaml \
  --output model.onnx
```

This will use:
- `batch_size = 1` (inference default)
- `seq_len = 10` (from `config.yaml`: `dataset.sequence_length`)
- `height = 224`, `width = 224` (from `config.yaml`: `dataset.image_size`)

### Example 2: Custom Sequence Length

If you trained with a different sequence length:

```bash
python export_to_onnx.py \
  --checkpoint best_model.pth \
  --config config.yaml \
  --output model.onnx \
  --seq-len 5
```

**Important**: Make sure this matches:
- Your training config's `dataset.sequence_length`
- Your ROS2 node's `sequence_length` parameter

### Example 3: Custom Image Size

If you trained with different image dimensions:

```bash
python export_to_onnx.py \
  --checkpoint best_model.pth \
  --config config.yaml \
  --output model.onnx \
  --height 320 \
  --width 320
```

**Important**: Make sure this matches:
- Your training config's `dataset.image_size`
- Your ROS2 node's `input_width` and `input_height` parameters

## Verification

After export, verify the ONNX model:

```bash
python -c "import onnx; model = onnx.load('model.onnx'); print('Input:', [i.shape for i in model.graph.input]); print('Output:', [o.shape for o in model.graph.output])"
```

Expected output:
```
Input: [('images', [1, 10, 3, 224, 224])]  # (batch, seq_len, channels, height, width)
Output: [('actions', [1, 15, 2])]          # (batch, chunk_size, action_dim)
```

## Common Mistakes

### ❌ Wrong: Using Training Batch Size

```bash
# DON'T do this - training batch size is 64, but inference should be 1
python export_to_onnx.py --batch-size 64 ...
```

**Why**: The model will be compiled for batch size 1 during inference. Using 64 wastes memory and may cause compilation issues.

### ❌ Wrong: Mismatched Sequence Length

```bash
# DON'T do this if your model was trained with seq_len=10
python export_to_onnx.py --seq-len 5 ...
```

**Why**: The model architecture expects the sequence length it was trained with. Mismatched values will cause shape errors or incorrect behavior.

### ❌ Wrong: Mismatched Image Size

```bash
# DON'T do this if your model was trained with 224x224
python export_to_onnx.py --height 320 --width 320 ...
```

**Why**: The vision encoder expects the input size it was trained with. Different sizes will cause shape errors.

## Summary Table

| Parameter | Training Value | Inference Value | Source |
|-----------|---------------|-----------------|--------|
| `batch_size` | 64 (or config) | **Always 1** | Fixed for inference |
| `seq_len` | From config | **Same as training** | `config.yaml`: `dataset.sequence_length` |
| `height` | From config | **Same as training** | `config.yaml`: `dataset.image_size[0]` |
| `width` | From config | **Same as training** | `config.yaml`: `dataset.image_size[1]` |

## Best Practice

**Always use the config file from your training experiment** - it contains all the correct values:

```bash
python export_to_onnx.py \
  --checkpoint outputs/YOUR_EXPERIMENT/checkpoints/best_model.pth \
  --config outputs/YOUR_EXPERIMENT/config.yaml \
  --output model.onnx
```

The script will automatically use the correct values from the config, ensuring consistency between training and inference.

