#!/usr/bin/env python3
"""
Export YOLOv11 model for Hailo compilation.

This script exports a trained YOLOv11 model to ONNX format with raw detection
head outputs (bypassing the Detect layer post-processing). This is required
for Hailo compilation, which needs the raw outputs to apply its own optimized
post-processing.

Usage:
    python3 export_yolo_for_hailo.py --model path/to/best.pt --output model.onnx
"""

import argparse
import torch
from ultralytics import YOLO
import onnx


def export_for_hailo(model_path, output_path, imgsz=640, opset=11):
  """
  Export YOLOv11 model with raw detection head outputs for Hailo compilation.
  
  Args:
      model_path: Path to the trained .pt model file
      output_path: Path for the output .onnx file
      imgsz: Input image size (default: 640)
      opset: ONNX opset version (default: 11)
  """
  print(f"Loading model from: {model_path}")
  model = YOLO(model_path)
  pt_model = model.model
  pt_model.eval()

  # Bypass the Detect layer post-processing to get raw outputs
  # The Detect layer normally combines and processes the outputs,
  # but Hailo needs the raw detection head outputs
  original_forward = pt_model.model[-1].forward
  pt_model.model[-1].forward = lambda x: x  # Return neck outputs directly

  print(f"Creating dummy input: (1, 3, {imgsz}, {imgsz})")
  dummy_input = torch.randn(1, 3, imgsz, imgsz)

  # Test forward pass to determine number of outputs
  print("Running test forward pass...")
  with torch.no_grad():
    test_out = pt_model(dummy_input)
    if isinstance(test_out, (list, tuple)):
      num_outputs = len(test_out)
    else:
      num_outputs = 1
      test_out = [test_out]
    
    print(f"Model has {num_outputs} output tensors")
    for i, out in enumerate(test_out):
      print(f"  output{i}: shape {out.shape}")

  # Generate output names
  output_names = [f"output{i}" for i in range(num_outputs)]

  # Export to ONNX
  print(f"\nExporting to ONNX (opset {opset})...")
  torch.onnx.export(
    pt_model,
    dummy_input,
    output_path,
    opset_version=opset,
    input_names=["images"],
    output_names=output_names,
    dynamic_axes=None  # Static shapes required for Hailo
  )

  # Verify the export
  print(f"\nVerifying exported model...")
  onnx_model = onnx.load(output_path)
  onnx.checker.check_model(onnx_model)
  
  print(f"\n✅ Export successful!")
  print(f"   Output file: {output_path}")
  print(f"   Input: images")
  print(f"   Outputs: {output_names}")
  print(f"\nNext steps:")
  print(f"  1. Verify outputs: python3 -c 'import onnx; m=onnx.load(\"{output_path}\"); print([o.name for o in m.graph.output])'")
  print(f"  2. Copy to Hailo compilation machine")
  print(f"  3. Create y11.model file with these output names")


def main():
  parser = argparse.ArgumentParser(
    description="Export YOLOv11 model for Hailo compilation",
    formatter_class=argparse.RawDescriptionHelpFormatter,
    epilog="""
Examples:
  # Export with default settings
  python3 export_yolo_for_hailo.py --model runs/train/weights/best.pt --output yolov11n_finetune.onnx
  
  # Export with custom image size
  python3 export_yolo_for_hailo.py --model best.pt --output model.onnx --imgsz 416
    """
  )
  
  parser.add_argument(
    "--model",
    type=str,
    required=True,
    help="Path to the trained YOLOv11 .pt model file"
  )
  
  parser.add_argument(
    "--output",
    type=str,
    default="yolov11n_finetune.onnx",
    help="Output path for the ONNX file (default: yolov11n_finetune.onnx)"
  )
  
  parser.add_argument(
    "--imgsz",
    type=int,
    default=640,
    help="Input image size (default: 640)"
  )
  
  parser.add_argument(
    "--opset",
    type=int,
    default=11,
    help="ONNX opset version (default: 11, compatible with Hailo)"
  )
  
  args = parser.parse_args()
  
  try:
    export_for_hailo(args.model, args.output, args.imgsz, args.opset)
  except Exception as e:
    print(f"\n❌ Export failed: {e}")
    import traceback
    traceback.print_exc()
    exit(1)


if __name__ == "__main__":
  main()

