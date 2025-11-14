# Model Compilation Guide for Raspberry Pi 5 AI Hat

This guide explains how to convert a trained PyTorch imitation learning model to HEF (Hailo Executable Format) for inference on the Raspberry Pi 5 AI Hat.

**⚠️ Important Note:** Complex transformer architectures (like the action chunking transformer used in this project) may not be fully supported by Hailo's ONNX parser. If you encounter parsing errors, you may need to:
- Simplify the model architecture
- Replace unsupported operations with Hailo-compatible alternatives
- Consider alternative deployment strategies (CPU/GPU inference)
- Contact Hailo support for guidance on custom architectures

## Prerequisites

1. **Trained PyTorch Model**: A `.pth` checkpoint file from the imitation learning training pipeline
2. **Hailo Model Zoo Tools**: Hailo's model compilation tools (Hailo Dataflow Compiler)
3. **Model Architecture Knowledge**: Understanding of your model's input/output shapes
4. **Hailo Compatibility**: Awareness that complex transformer operations may require modifications

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

The Hailo compilation process is a two-step procedure:
1. **Parse ONNX to HAR**: Convert ONNX model to Hailo Archive (HAR) format
2. **Compile HAR to HEF**: Compile HAR to Hailo Executable Format (HEF)

### Installation

1. Download Hailo Dataflow Compiler from Hailo's developer portal
2. Follow Hailo's installation instructions
3. Ensure you have the correct version for your Hailo-8L chip

### Step 3a: Parse ONNX to HAR

First, convert your ONNX model to HAR format:

```bash
# Check parser help
hailo parser onnx --help
```

**First, check the actual tensor names in your ONNX model:**

```bash
python -c "import onnx; model = onnx.load('model.onnx'); \
  print('Inputs:', [i.name for i in model.graph.input]); \
  print('Outputs:', [o.name for o in model.graph.output])"
```

The parser command syntax:

**Try these options in order:**

**Option 1: Without tensor-shapes** (if ONNX model already has shape information):

```bash
hailo parser onnx \
  /path/to/model.onnx \
  --har-path /path/to/model.har \
  --hw-arch hailo8l
```

**Option 2: Colon-separated format** (if shapes need to be specified):

```bash
hailo parser onnx \
  /path/to/model.onnx \
  --har-path /path/to/model.har \
  --tensor-shapes images:[1,10,3,224,224] \
  --hw-arch hailo8l
```

**Option 3: JSON format** (if the above don't work):

```bash
hailo parser onnx \
  /path/to/model.onnx \
  --har-path /path/to/model.har \
  --tensor-shapes '{"images": [1,10,3,224,224]}' \
  --hw-arch hailo8l
```

**Key Parameters:**
- `model_path`: Input ONNX model path (positional argument)
- `--har-path`: Output HAR file path
- `--tensor-shapes`: Input tensor shapes (output shapes are inferred from the model)
  - **Format 1 (JSON):** `'{"images": [1,10,3,224,224]}'` - Only input tensor
  - **Format 2 (colon-separated):** `images:[1,10,3,224,224]` - Alternative format
  - Input shape: `[1,10,3,224,224]` = `(batch, seq_len, channels, height, width)`
  - **Note:** Output shapes are typically inferred automatically from the ONNX model
- `--hw-arch hailo8l`: Target hardware architecture (hailo8l for Raspberry Pi 5 AI Hat)

**Optional Parameters:**
- `--net-name`: Network name (optional)
- `--input-format`: Input format specification (if needed)
- `--parsing-report-path`: Path to save parsing report (useful for debugging)

**Note:** Calibration for quantization is typically handled during the compilation step (Step 3b), not during parsing.

### Step 3b: Compile HAR to HEF

Once you have a HAR file, compile it to HEF format:

```bash
hailo compiler \
  model.har \
  --hw-arch hailo8l \
  --output-dir /path/to/output/directory
```

**Key Parameters:**
- `har_path`: Path to the HAR file (positional argument)
- `--hw-arch hailo8l`: Target hardware architecture (hailo8l for Raspberry Pi 5 AI Hat)
- `--output-dir`: Directory where the HEF file will be saved (defaults to current directory)

**Optional Parameters:**
- `--model-script`: Path to a custom model script (for advanced post-processing)
- `--auto-model-script`: Path to save auto-generated model script (useful for debugging)

### Troubleshooting: Model Compatibility Issues

**If you encounter parsing errors** (like `IndexError: list index out of range` or unsupported operations):

This typically means the Hailo parser doesn't fully support some operations in your transformer model. Common issues:

1. **Complex Transformer Operations**: Some transformer operations (attention mechanisms, positional encodings, etc.) may not be directly supported
2. **Missing Attributes**: Some layers may be missing expected attributes in the ONNX export

**Potential Solutions:**

1. **Check Hailo Documentation**: Verify which ONNX operations are supported for your Hailo chip version
2. **Simplify the Model**: Consider creating a simplified version without unsupported operations
3. **Use Hailo Model Zoo**: If possible, adapt your model to use operations supported by Hailo Model Zoo
4. **Contact Hailo Support**: For custom architectures, Hailo may provide guidance on supported operations
5. **Alternative Deployment**: Consider running inference on CPU/GPU instead of Hailo for complex transformer models

**Note:** Transformer models with attention mechanisms and complex temporal processing may require significant modifications to work with Hailo's hardware constraints. You may need to:
- Replace unsupported operations with Hailo-compatible alternatives
- Simplify the architecture (e.g., remove temporal components, use simpler attention)
- Use quantization-aware training specifically for Hailo

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

# 3a. Parse ONNX to HAR
# Convert ONNX model to Hailo Archive format
# Try without tensor-shapes first (if ONNX has shape info):
hailo parser onnx \
  model.onnx \
  --har-path model.har \
  --hw-arch hailo8l
# If that fails, try with colon-separated format:
# --tensor-shapes images:[1,10,3,224,224]
# Or JSON format:
# --tensor-shapes '{"images": [1,10,3,224,224]}'

# 3b. Compile HAR to HEF
# Compile HAR to Hailo Executable Format
hailo compiler \
  model.har \
  --hw-arch hailo8l \
  --output-dir .

# 4. Verify
hailo info model.hef

# 5. Deploy
scp model.hef pi@raspberry-pi:/home/rcr/repos/common_platform/models/

# 6. Run ROS2 node
ros2 launch imitation_learning imitation_learning.launch.py \
  model_path:=/home/rcr/repos/common_platform/models/model.hef
```

