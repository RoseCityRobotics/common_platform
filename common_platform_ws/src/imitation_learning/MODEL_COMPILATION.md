# Model Compilation Guide for Raspberry Pi 5 AI Hat

This guide explains how to convert a trained PyTorch imitation learning model to HEF (Hailo Executable Format) for inference on the Raspberry Pi 5 AI Hat.

## Prerequisites

1. **Trained PyTorch Model**: A `.pth` checkpoint file from the imitation learning training pipeline
2. **Hailo Model Zoo Tools**: Hailo's model compilation tools (Hailo Dataflow Compiler)
3. **Model Architecture Knowledge**: Understanding of your model's input/output shapes

## Overview

The compilation process involves:
1. Exporting PyTorch model to ONNX format
2. Optimizing ONNX model for Hailo
3. Compiling ONNX to HEF format using Hailo Dataflow Compiler

## Step 1: Export PyTorch Model to ONNX

Use the provided export script to convert your trained model to ONNX format:

A helper script is provided at `scripts/imitation_learning/export_to_onnx.py`. 

**Usage:**
```bash
python export_to_onnx.py \
  --checkpoint /path/to/checkpoints/best_model.pth \
  --config /path/to/config.yaml \
  --output /path/to/model.onnx \
  --batch-size 1 \
  --seq-len 10 \
  --height 224 \
  --width 224
```

## Step 2: Optimize ONNX Model (Optional)

**Note:** This step is optional. The Hailo Dataflow Compiler performs its own optimizations, so onnx-simplifier is not strictly necessary. However, it can help catch ONNX export issues early and sometimes produces a cleaner model for the compiler.

If you want to use onnx-simplifier:

```bash
# Install onnx-simplifier if not already installed
pip install onnx-simplifier

# Simplify ONNX model
python -m onnxsim model.onnx model_simplified.onnx
```

**What onnx-simplifier does:**
- Constant folding (evaluates constant expressions at compile time)
- Removes redundant operations
- Simplifies graph structure
- Can help identify ONNX export issues early

**What Hailo Dataflow Compiler does:**
- Hardware-specific optimizations for Hailo chips
- Quantization (INT8 conversion) with calibration
- Resource allocation and scheduling
- Model translation to HEF format

The Hailo compiler will optimize the model regardless, so you can skip this step if you prefer a simpler workflow.

## Step 3: Compile ONNX to HEF using Hailo Dataflow Compiler

The Hailo Dataflow Compiler (HDF) converts ONNX models to HEF format. This tool is typically provided by Hailo and runs on a development machine (not necessarily on the Raspberry Pi).

### Installation

1. Download Hailo Dataflow Compiler from Hailo's developer portal
2. Follow Hailo's installation instructions
3. Ensure you have the correct version for your Hailo-8L chip

### Compilation Command

```bash
hailo compile \
  --input model.onnx \
  --output model.hef \
  --input-shape images:1,10,3,224,224 \
  --output-shape actions:1,15,2 \
  --quantization-calibration-dataset /path/to/calibration/images \
  --target hailo8l
```

**Key Parameters:**
- `--input`: Input ONNX model path
- `--output`: Output HEF file path
- `--input-shape`: Input tensor shape `(batch, seq_len, channels, height, width)`
- `--output-shape`: Output tensor shape `(batch, chunk_size, action_dim)` - **Note:** The model's forward pass returns `(batch, seq_len, chunk_size, action_dim)`, but for inference we only need the first timestep's chunk, so we extract `(batch, chunk_size, action_dim)`
- `--quantization-calibration-dataset`: Path to calibration images for quantization
- `--target`: Target Hailo chip (hailo8l for Raspberry Pi 5 AI Hat)

### Quantization Calibration

For best performance, you need a calibration dataset:

1. **Collect Calibration Images**: Use a subset of your training images (typically 100-1000 images)
2. **Prepare Calibration Dataset**: Organize images in a directory structure that Hailo tools can read
3. **Run Calibration**: The compiler will use these images to calibrate quantization

**Example calibration dataset structure:**
```
calibration_dataset/
  image_001.jpg
  image_002.jpg
  ...
  image_100.jpg
```

## Step 4: Verify HEF Model

After compilation, verify the HEF model:

```bash
hailo info model.hef
```

This will display:
- Input/output shapes
- Model size
- Supported features
- Performance estimates

## Step 5: Deploy to Raspberry Pi

1. **Copy HEF file to Raspberry Pi:**
```bash
scp model.hef user@raspberry-pi:/home/rcr/repos/common_platform/models/
```

2. **Update model path in launch file or parameter:**
```bash
ros2 launch imitation_learning imitation_learning.launch.py \
  model_path:=/home/rcr/repos/common_platform/models/model.hef
```

## Troubleshooting

### Common Issues

1. **ONNX Export Fails**
   - Check that all operations in your model are ONNX-compatible
   - Some PyTorch operations may need to be replaced with ONNX-compatible alternatives
   - Use `torch.onnx.export` with `verbose=True` to see detailed export information

2. **HEF Compilation Fails**
   - Verify input/output shapes match your model architecture
   - Check that quantization calibration dataset is accessible
   - Ensure you're using the correct target chip (hailo8l)

3. **Inference Performance Issues**
   - Use calibration dataset for better quantization
   - Consider model pruning or quantization-aware training
   - Check HailoRT logs for performance bottlenecks

4. **Shape Mismatches**
   - Verify input shape in ROS2 node matches compiled model
   - Check sequence_length parameter matches model's expected sequence length
   - Ensure image preprocessing matches training preprocessing

## Model Architecture Considerations

When designing models for Hailo compilation:

1. **Supported Operations**: Check Hailo's documentation for supported ONNX operations
2. **Input/Output Shapes**: Fixed shapes are preferred, though dynamic shapes may be supported
3. **Quantization**: Models are quantized to INT8 for inference - consider quantization-aware training
4. **Memory Constraints**: Hailo-8L has limited memory - large models may not fit

## Alternative: Simplified Single-Frame Model

If temporal sequences are challenging, you can create a simplified single-frame model:

1. **Modify Model Architecture**: Remove temporal components, use only current frame
2. **Retrain or Fine-tune**: Train the simplified model
3. **Export and Compile**: Follow the same export/compile process with `seq_len=1`

This simplifies the compilation process and may improve inference speed.

## References

- [Hailo Developer Portal](https://hailo.ai/developer-zone/)
- [Hailo Dataflow Compiler Documentation](https://hailo.ai/documentation/)
- [ONNX Export Guide](https://pytorch.org/docs/stable/onnx.html)
- [Raspberry Pi 5 AI Hat Documentation](https://www.raspberrypi.com/documentation/)

## Example Complete Workflow

```bash
# 1. Export to ONNX
python export_to_onnx.py \
  --checkpoint outputs/experiment_name/checkpoints/best_model.pth \
  --config outputs/experiment_name/config.yaml \
  --output model.onnx

# 2. Simplify ONNX (optional)
python -m onnxsim model.onnx model_simplified.onnx

# 3. Compile to HEF
# Note: Output shape is (batch, chunk_size, action_dim) = (1, 15, 2)
# The export script extracts the first timestep from the model's output
hailo compile \
  --input model_simplified.onnx \
  --output model.hef \
  --input-shape images:1,10,3,224,224 \
  --output-shape actions:1,15,2 \
  --quantization-calibration-dataset /path/to/calibration/images \
  --target hailo8l

# 4. Verify
hailo info model.hef

# 5. Deploy
scp model.hef pi@raspberry-pi:/home/rcr/repos/common_platform/models/

# 6. Run ROS2 node
ros2 launch imitation_learning imitation_learning.launch.py \
  model_path:=/home/rcr/repos/common_platform/models/model.hef
```

