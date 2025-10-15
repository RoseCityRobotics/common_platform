# YOLOv11n Fine-Tuning Workflow (Ultralytics on macOS)

This guide documents the full process used to fine-tune a **YOLOv11n** object-detection model using custom data labeled in **Label Studio**.

---

## 1. Data Labeling with Label Studio

1. Launch Label Studio and create a new **Object Detection** project.  
2. Upload your images into the project.  
3. Use the **Rectangle Labels** tool to annotate each object (e.g., “Purple ball”, “Green ball”).  
4. When finished, export the annotations as **JSON** files (one per task).

Each exported JSON (e.g., `annotations/999`) contains fields like:

```json
{
  "id": 999,
  "result": [
    {
      "type": "rectanglelabels",
      "value": {
        "x": 27.5,
        "y": 45.2,
        "width": 20.4,
        "height": 18.3,
        "rectanglelabels": ["Purple ball"]
      }
    }
  ],
  "task": {
    "data": {
      "image": "/data/local-files/?d=home/rcr/teleop_data/images/00011190.jpg"
    }
  }
}
```

---

## 2. Convert Label Studio JSON to YOLO Format

Label Studio’s JSON format must be converted into YOLO-style text files.

A conversion script `ls_to_yolo.py` was used:

```bash
python3 scripts/ls_to_yolo.py
```

Output example:

```
Converted items: 1504
Class mapping: {0: 'Purple ball', 1: 'Green ball'}
Paste this into data.yaml as names: ['Purple ball', 'Green ball']
Stats: {'total_items': 1504, 'with_any_result': 271, 'with_boxes': 271, 'images_copied': 1504, 'missing_images': 0, 'empty_results': 1233}
```

This script created:
- YOLO-formatted label `.txt` files (one per image)
- a **data split** structure:
  ```
  data/
  ├── images/train/
  ├── images/val/
  ├── labels/train/
  ├── labels/val/
  ```

---

## 3. Clean Up the Dataset

A secondary script removed images with **no bounding boxes**:

```bash
python3 prune_images_without_boxes.py --images-dir data/images --annotations-dir annotations --delete
```

Result:

```
Found 271 stems with rectangles.
Images scanned: 1504  |  keep: 271  |  prune: 1233
```

After pruning, only labeled images remained in `data/images/`.

---

## 4. Prepare the `data.yaml` File

Create a file `data.yaml` describing the dataset and class names:

```yaml
train: data/images/train
val: data/images/val

nc: 2
names: ['Purple ball', 'Green ball']
```

---

## 5. Verify Ultralytics Installation

Confirm Ultralytics (YOLOv11) is installed in your virtual environment:

```bash
pip install ultralytics
```

You can test on a sample dataset to confirm everything works:

```bash
yolo detect train model=yolo11n.pt data=coco8.yaml epochs=1 imgsz=640
```

---

## 6. Fine-Tune the YOLOv11n Model

Run training on your custom data:

```bash
yolo train model=yolo11n.pt data=data.yaml epochs=100 imgsz=640 batch=16 device=mps project=runs name=y11n_custom
```

Key arguments:
| Option | Description |
|---------|--------------|
| `model=yolo11n.pt` | Base pretrained YOLOv11n weights |
| `data=data.yaml` | Custom dataset definition |
| `epochs=100` | Train for 100 epochs |
| `imgsz=640` | Input image size |
| `batch=16` | Batch size (adjust to available RAM) |
| `device=mps` | Use Apple Metal GPU acceleration |
| `project=runs` | Output directory |
| `name=y11n_custom` | Experiment name |

---

## 7. Monitor Training Progress

During training, YOLO prints metrics per epoch:

| Metric | Meaning |
|---------|----------|
| `box_loss` | Bounding-box regression error |
| `cls_loss` | Classification loss |
| `dfl_loss` | Distribution Focal Loss (for box boundaries) |
| `Instances` | Number of labeled objects in batch |

Validation metrics (computed every few epochs):
- **Precision** – fraction of predicted boxes that are correct.  
- **Recall** – fraction of ground truths correctly detected.  
- **mAP@50** – mean average precision at IoU ≥ 0.50.  
- **mAP@50-95** – stricter average over IoU 0.50–0.95.

---

## 8. Results and Best Model

After 100 epochs, YOLO saves results in:

```
runs/train/y11n_custom/
├── weights/
│   ├── best.pt
│   └── last.pt
├── results.csv
└── confusion_matrix.png
```

Use `best.pt` for inference or export.

---

## 9. Export the Model for Deployment

Export to ONNX (static 640×640):

```bash
yolo export model=runs/train/y11n_custom/weights/best.pt format=onnx imgsz=640 opset=13 simplify=True dynamic=False
```

This creates `yolov11n_finetune.onnx`, ready for conversion to `.hef` using the Hailo compiler.

---

## 10. Summary

| Stage | Purpose | Tool |
|--------|----------|------|
| Label images | Create bounding-box annotations | Label Studio |
| Convert annotations | JSON → YOLO format | `ls_to_yolo.py` |
| Prune dataset | Remove unlabeled images | `prune_images_without_boxes.py` |
| Fine-tune | Train YOLOv11n on 2 classes | Ultralytics YOLO |
| Export | Produce ONNX for Hailo | `yolo export` |

Final output:  
✅ `best.pt` (PyTorch weights)  
✅ `yolov11n_finetune.onnx` (for hardware deployment)

---
