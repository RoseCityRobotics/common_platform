#!/usr/bin/env python3
"""
Export imitation learning model to ONNX format for Hailo compilation.

This script exports a trained PyTorch imitation learning model to ONNX format,
which can then be compiled to HEF format for deployment on the Raspberry Pi 5 AI Hat.

Usage:
    python export_to_onnx.py \
      --checkpoint /path/to/checkpoints/best_model.pth \
      --config /path/to/config.yaml \
      --output /path/to/model.onnx \
      --batch-size 1 \
      --seq-len 10 \
      --height 224 \
      --width 224
"""

import torch
import torch.onnx
import sys
import os
import argparse
from pathlib import Path

# Add the imitation learning module to path
script_dir = Path(__file__).parent
src_dir = script_dir / "src"
sys.path.insert(0, str(src_dir))

from imitation_learning.model import create_model, load_pretrained_weights
from imitation_learning.utils import load_config


def export_to_onnx(
    checkpoint_path: str,
    config_path: str,
    output_path: str,
    input_shape: tuple = (1, 10, 3, 224, 224),  # (batch, seq_len, channels, height, width)
    opset_version: int = 13,
    simplify: bool = True
):
    """
    Export PyTorch model to ONNX format.
    
    Args:
        checkpoint_path: Path to PyTorch checkpoint (.pth file)
        config_path: Path to model config YAML
        output_path: Output ONNX file path
        input_shape: Input tensor shape (batch, seq_len, channels, height, width)
        opset_version: ONNX opset version (13 recommended for Hailo)
        simplify: Whether to simplify the ONNX model after export
    """
    print("=" * 60)
    print("Imitation Learning Model ONNX Export")
    print("=" * 60)
    
    # Load config
    print(f"\nLoading config from: {config_path}")
    config = load_config(config_path)
    
    # Create model
    print("Creating model architecture...")
    model = create_model(config)
    
    # Load weights
    print(f"Loading weights from: {checkpoint_path}")
    model = load_pretrained_weights(model, checkpoint_path, device='cpu')
    model.eval()
    
    # Print model info
    model_info = model.get_model_info()
    print(f"\nModel Information:")
    print(f"  Total parameters: {model_info['total_parameters']:,}")
    print(f"  Trainable parameters: {model_info['trainable_parameters']:,}")
    print(f"  Backbone: {model_info['backbone']}")
    print(f"  Action dimension: {model_info['action_dim']}")
    print(f"  Chunk size: {model_info['chunk_size']}")
    
    # Create dummy input
    print(f"\nCreating dummy input with shape: {input_shape}")
    dummy_input = torch.randn(*input_shape)
    
    # Export to ONNX
    print(f"\nExporting model to ONNX: {output_path}")
    print(f"  Input shape: {input_shape}")
    print(f"  Opset version: {opset_version}")
    
    try:
        torch.onnx.export(
            model,
            dummy_input,
            output_path,
            export_params=True,
            opset_version=opset_version,
            do_constant_folding=True,
            input_names=['images'],
            output_names=['actions'],
            dynamic_axes={
                'images': {0: 'batch_size'},  # Allow variable batch size
                'actions': {0: 'batch_size'}
            },
            verbose=False
        )
        print(f"✓ ONNX model exported successfully to: {output_path}")
    except Exception as e:
        print(f"✗ Error exporting to ONNX: {e}")
        raise
    
    # Simplify ONNX model if requested
    if simplify:
        try:
            print("\nSimplifying ONNX model...")
            import onnx
            from onnxsim import simplify
            
            # Load the model
            onnx_model = onnx.load(output_path)
            
            # Simplify
            simplified_model, check = simplify(onnx_model)
            
            if check:
                # Save simplified model
                simplified_path = output_path.replace('.onnx', '_simplified.onnx')
                onnx.save(simplified_model, simplified_path)
                print(f"✓ Simplified model saved to: {simplified_path}")
                print(f"  Original size: {os.path.getsize(output_path) / 1024 / 1024:.2f} MB")
                print(f"  Simplified size: {os.path.getsize(simplified_path) / 1024 / 1024:.2f} MB")
            else:
                print("⚠ Simplification check failed, using original model")
        except ImportError:
            print("⚠ onnx-simplifier not installed, skipping simplification")
            print("  Install with: pip install onnx-simplifier")
        except Exception as e:
            print(f"⚠ Error simplifying model: {e}")
            print("  Continuing with original model...")
    
    print("\n" + "=" * 60)
    print("Export Complete!")
    print("=" * 60)
    print("\nNext steps:")
    print("1. Verify ONNX model: python -c \"import onnx; onnx.checker.check_model('model.onnx')\"")
    print("2. Compile to HEF using Hailo Dataflow Compiler")
    print("3. See MODEL_COMPILATION.md for detailed compilation instructions")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Export imitation learning model to ONNX format for Hailo compilation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic export
  python export_to_onnx.py \\
    --checkpoint outputs/experiment/checkpoints/best_model.pth \\
    --config outputs/experiment/config.yaml \\
    --output model.onnx

  # Custom input dimensions
  python export_to_onnx.py \\
    --checkpoint best_model.pth \\
    --config config.yaml \\
    --output model.onnx \\
    --seq-len 5 \\
    --height 320 \\
    --width 320
        """
    )
    
    parser.add_argument('--checkpoint', type=str, required=True,
                        help='Path to PyTorch checkpoint file (.pth)')
    parser.add_argument('--config', type=str, required=True,
                        help='Path to model config YAML file')
    parser.add_argument('--output', type=str, required=True,
                        help='Output ONNX file path')
    parser.add_argument('--batch-size', type=int, default=1,
                        help='Batch size for export (default: 1)')
    parser.add_argument('--seq-len', type=int, default=10,
                        help='Sequence length (default: 10)')
    parser.add_argument('--height', type=int, default=224,
                        help='Image height (default: 224)')
    parser.add_argument('--width', type=int, default=224,
                        help='Image width (default: 224)')
    parser.add_argument('--opset-version', type=int, default=13,
                        help='ONNX opset version (default: 13)')
    parser.add_argument('--no-simplify', action='store_true',
                        help='Skip ONNX simplification step')
    
    args = parser.parse_args()
    
    # Validate paths
    if not os.path.exists(args.checkpoint):
        print(f"Error: Checkpoint file not found: {args.checkpoint}")
        sys.exit(1)
    
    if not os.path.exists(args.config):
        print(f"Error: Config file not found: {args.config}")
        sys.exit(1)
    
    # Create output directory if needed
    output_dir = os.path.dirname(args.output)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)
    
    input_shape = (args.batch_size, args.seq_len, 3, args.height, args.width)
    
    export_to_onnx(
        checkpoint_path=args.checkpoint,
        config_path=args.config,
        output_path=args.output,
        input_shape=input_shape,
        opset_version=args.opset_version,
        simplify=not args.no_simplify
    )

