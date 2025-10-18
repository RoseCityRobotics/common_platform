# YOLOv11n Fine-Tuning Workflow (Ultralytics on macOS)

This guide documents the full process used to fine-tune a **YOLOv11n** object-detection model using custom data labeled in **Label Studio**.
**Note that the following steps are best performmed on a computer with a good GPU and plenty of memory. Not the Raspberry Pi 5!**

---

## 0. [Data Labeling with Label Studio](../data_annotation/label_studio.md)

Each exported JSON (e.g., `annotations/999`) contains fields like:

```json
{
  "id": 999,
  "result": [],
  "created_username": "",
  "created_ago": "0\u00a0minutes",
  "completed_by": {
    "id": 1,
    "first_name": "",
    "last_name": "",
    "email": ""
  },
  "task": {
    "id": 999,
    "data": {
      "image": "/data/local-files/?d=home/rcr/teleop_data/images/00011190.jpg"
    },
    "meta": {},
    "created_at": "2025-10-08T23:41:28.798109Z",
    "updated_at": "2025-10-08T23:41:28.798143Z",
    "is_labeled": true,
    "overlap": 1,
    "inner_id": 999,
    "total_annotations": 1,
    "cancelled_annotations": 0,
    "total_predictions": 0,
    "comment_count": 0,
    "unresolved_comment_count": 0,
    "last_comment_updated_at": null,
    "project": 1,
    "updated_by": null,
    "file_upload": null,
    "comment_authors": []
  },
  "was_cancelled": false,
  "ground_truth": false,
  "created_at": "2025-10-09T00:08:55.627074Z",
  "updated_at": "2025-10-09T00:08:55.627108Z",
  "draft_created_at": null,
  "lead_time": 0.666,
  "import_id": null,
  "last_action": null,
  "bulk_created": false,
  "project": 1,
  "updated_by": 1,
  "parent_prediction": null,
  "parent_annotation": null,
  "last_created_by": null
}
```

---

## 1. Create the directory structure for fine tuning

You will need a python virtual environment in which to install and use the Ultralytics software for fine tuning the YOLO model.

```bash
cd
python3 -m venv finetune
cd finetune
source bin/activate
mkdir scripts
cp ~/repos/common_platform/scripts/ls_to_yolo.py scripts/
cp ~/repos/common_platform/scripts/make_split_and_yaml.py scripts/
cp ~/repos/common_platform/scripts/prune_images_without_boxes.py scripts/
mkdir -p data/images_raw
cp ~/teleop_data/images/* data/images_raw/
cp -r ~/teleop_data/annotations data/
mkdir models
cp ~/repos/common_platform/models/yolo11n.pt models/
mkdir runs
pip install ultralytics
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

Then, split the data into training and validation sets
```bash
python3 scripts/make_split_and_yaml.py
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

Finally, update the data.yaml

---

## 3. Clean Up the Dataset

A secondary script removed images with **no bounding boxes**:

```bash
python3 scripts/prune_images_without_boxes.py --images-dir data/images_raw --annotations-dir data/annotations --delete
```

Result:

```
Found 271 stems with rectangles.
Images scanned: 1504  |  keep: 271  |  prune: 1233
```

After pruning, only labeled images remained in `data/images_raw/`. This is necessary to create a calibration dataset for the conversion to the .hef format that will work on the Hailo 8 AI Hat.

---

## 4. Edit the `data.yaml` File

The `data.yaml` file was created by `make_split_and_yaml.py`. It describes the dataset and class names:

```yaml
path: data
train: train/images
val: val/images
nc: 2
names:
- class_0
- class_1
```

**Edit the `data.yaml` file with the correct class names.**
```bash
nano data.yaml
```

```
names:
- Purple ball
- Green ball
```

---

## 5. Verify Ultralytics Installation

You can test on a sample dataset to confirm everything works:

```bash
yolo detect train model=yolo11n.pt data=coco8.yaml epochs=1 imgsz=640
```

---

## 6. Fine-Tune the YOLOv11n Model

**Note that this will take a VERY LONG time on the Raspberry Pi 5. It is better to run it on a computer with a good GPU and lots of memory.**
Run training on your custom data:

```bash
yolo train model=models/yolo11n.pt data=data.yaml epochs=100 imgsz=640 batch=16 project=runs name=y11n_finetune
```

Choosing a batch size
| Hardware                        | Recommended batch       |
| ------------------------------- | ----------------------- |
| **M1 / M2 / M3 MacBook Air**    | `batch=8`               |
| **M1 / M2 / M3 Pro (16 GB)**    | `batch=16`              |
| **M1 / M2 / M3 Max (32–64 GB)** | `batch=32`              |
| **Intel Mac (CPU only)**        | `batch=4` (or even `2`) |

Key arguments:
| Option | Description |
|---------|--------------|
| `model=yolo11n.pt` | Base pretrained YOLOv11n weights |
| `data=data.yaml` | Custom dataset definition |
| `epochs=100` | Train for 100 epochs |
| `imgsz=640` | Input image size |
| `batch=16` | Batch size (adjust to available RAM) |
| `device=mps` | **Add this if on Apple Silicon Mac to use Metal GPU acceleration** |
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
runs/train/y11n_finetune/
├── weights/
│   ├── best.pt
│   └── last.pt
├── results.csv
└── confusion_matrix.png
```

Use `best.pt` for inference or export.

---

## 9. Export the Model for Hailo Deployment

```bash
cd runs/y11n_finetune/weights
yolo export model=best.pt format=onnx imgsz=640 simplify=True nms=False
mv best.onnx yolov11n_finetune.onnx
scp yolov11n_finetune.onnx <USER>@<HOSTNAME>:~
```

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
