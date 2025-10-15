# YOLOv11n to Hailo-8 HEF Compilation Guide

This guide describes how to convert a **fine-tuned YOLOv11n model** (from Ultralytics) into a **Hailo Executable File (.hef)** for use on the Raspberry Pi AI Kit with the Hailo-8L accelerator.

---

## 1. Export the Model to ONNX

From your Ultralytics environment on macOS or Linux:

```bash
yolo export model=runs/detect/y11n_custom/weights/best.pt format=onnx imgsz=640 opset=13 simplify=True dynamic=False
```

- `format=onnx` → exports the model for Hailo’s compiler  
- `imgsz=640` → input resolution (match training)  
- `dynamic=False` → ensures static dimensions required by Hailo  

This produces `best.onnx` (rename it for clarity):

```bash
mv runs/detect/y11n_custom/weights/best.onnx yolov11n_finetune.onnx
```

---

## 2. Prepare Calibration Images

Hailo needs a small unlabeled image set (≈ 200–1000 images) from your dataset to calibrate the quantization step.

```bash
mkdir -p ~/scratch/images
# Copy random training/validation images here
```

---

## 3. Verify Model Inputs and Outputs

Check your ONNX for input and output node names:

```bash
python3 - <<'PY'
import onnx
m = onnx.load("yolov11n_finetune.onnx")
print("INPUTS:")
for i in m.graph.input:
    print(" -", i.name)
print("\nOUTPUTS:")
for o in m.graph.output:
    print(" -", o.name)
PY
```

Expected outputs (6 tensors):

```
/model.23/cv3.0/cv3.0.2/Conv
/model.23/cv2.0/cv2.0.2/Conv
/model.23/cv3.1/cv3.1.2/Conv
/model.23/cv2.1/cv2.1.2/Conv
/model.23/cv3.2/cv3.2.2/Conv
/model.23/cv2.2/cv2.2.2/Conv
```

---

## 4. Create the Model Script

The Hailo compiler needs to know which outputs correspond to the
**regression** and **classification** heads for each detection scale.

Create `y11.model`:

```text
set-input-nodes yolov11n/input_layer1
set-output-nodes /model.23/cv3.0/cv3.0.2/Conv /model.23/cv2.0/cv2.0.2/Conv /model.23/cv3.1/cv3.1.2/Conv /model.23/cv2.1/cv2.1.2/Conv /model.23/cv3.2/cv3.2.2/Conv /model.23/cv2.2/cv2.2.2/Conv
postprocess yolov8 classes=2 reg_max=16
nms iou=0.7 score=0.25
```

> **Note:**  
> The postprocess keyword is still named `yolov8` because YOLO v8 – v11 share the same DFL decoupled head structure.  
> This configuration is valid for YOLOv11n.

---

## 5. Run the Compilation

Activate your **Hailo-8 environment** (legacy suite, not the 15-series SDK).

```bash
hailomz compile   --ckpt ~/scratch/yolov11n_finetune.onnx   --calib-path ~/scratch/images   --yaml ./hailo_model_zoo/hailo_model_zoo/cfg/networks/yolov11n.yaml   --hw-arch hailo8   --model-script y11.model
```

### Key flags
| Option | Purpose |
|---------|----------|
| `--ckpt` | Path to your ONNX model |
| `--calib-path` | Directory of calibration images |
| `--yaml` | Network config (use `yolov8n.yaml` or `yolov11n.yaml` if present) |
| `--hw-arch hailo8` | Target chip (for Pi AI Hat / Hailo-8L) |
| `--model-script y11.model` | Explicit output node mapping and post-processing |

After successful compilation, you’ll see:

```
Saved HAR to yolov11n.har
Saved HEF to yolov11n.hef
```

---

## 6. Deploy to Raspberry Pi AI Kit

1. Copy `yolov11n.hef` to the Pi:

   ```bash
   scp yolov11n.hef pi@raspberrypi.local:/home/pi/
   ```

2. On the Pi, verify the device:

   ```bash
   hailortcli fw-control identify
   ```

3. Test inference:

   ```bash
   hailortcli run --hef yolov11n.hef
   ```

For application-level use, load it with the Python or C++ `hailort` API.

---

## 7. Troubleshooting

| Symptom | Likely cause / fix |
|----------|--------------------|
| `ValueError: None values not supported` during calibration | The post-process could not find reg/cls pairs — check the six output names and `y11.model`. |
| “invalid choice: 'hailo8'” | You’re using the Hailo-15 SDK. Install the **Hailo-8 legacy AI Suite v3/v4**. |
| Poor accuracy | Provide ≥ 200 calibration images and use a realistic dataset. |
| No `.hef` produced | Check for earlier “Optimization” or “Statistics” exceptions; usually due to incorrect output mapping. |

---

## 8. Summary

**Pipeline overview**

1. Fine-tune YOLOv11n with Ultralytics.  
2. Export to static ONNX.  
3. Prepare 200–1000 calibration images.  
4. Confirm 6 outputs (reg/cls pairs).  
5. Create `y11.model` for Hailo post-processing.  
6. Run `hailomz compile` with `--hw-arch hailo8`.  
7. Deploy the generated `.hef` to your Raspberry Pi AI Kit.

You now have a fully quantized and compiled YOLOv11n detector ready to run on the Hailo-8 accelerator.
