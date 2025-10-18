# YOLOv11n to Hailo-8 HEF Compilation Guide

This guide describes how to convert a **fine-tuned YOLOv11n model** (from Ultralytics) into a **Hailo Executable File (.hef)** for use on the Raspberry Pi AI Kit with the Hailo-8 accelerator.
**NOTE: These steps can only be completed on an Ubuntu computer with x86_64 architecture. It is best with a CUDA GPU.**
[Complete the steps in this document first](./YOLOv11n_Finetuning_Workflow.md)

---

## 0. Environment Setup

Before compiling models, you need to set up the Hailo development environment.

### Prerequisites

- **Ubuntu 20.04 or 22.04** (x86_64)
- **16GB+ RAM** (32GB recommended)
- **NVIDIA GPU with CUDA support** (optional but strongly recommended for faster compilation)
- **50GB+ free disk space**

Native installation provides better GPU access and is generally more reliable.

1. **Download Hailo Wheel Files**

   Visit the [Hailo Developer Zone](https://hailo.ai/developer-zone/) and download the required wheel files directly:

   - `hailo_dataflow_compiler-3.33.0-py3-none-linux_x86_64.whl`
   - `hailort-4.23.0-cp310-cp310-linux_x86_64.whl`
   - `hailo_model_zoo-2.17.0-py3-none-any.whl`

2. **Install Prerequisites**

   Install all required packages:

   ```bash
   sudo apt update
   sudo apt install -y python3.10 python3.10-venv python3.10-dev python3-pip \
     git wget build-essential python3-tk graphviz libgraphviz-dev
   ```

3. **Install CUDNN (Recommended for GPU Acceleration)**

   **Important:** CUDNN version must match your CUDA version. Check first:

   ```bash
   # Check your CUDA version
   nvcc --version
   nvidia-smi  # Look at "CUDA Version" in the output
   ```

   Download and install the matching CUDNN:

   ```bash
   # Download CUDNN from NVIDIA (requires free account)
   # Visit: https://developer.nvidia.com/cudnn-downloads
   # Select: CUDNN for CUDA 12.x, Linux x86_64, Ubuntu 22.04

   # Example installation (adjust version as needed):
   wget https://developer.download.nvidia.com/compute/cudnn/9.14.0/local_installers/cudnn-local-repo-ubuntu2204-9.14.0_1.0-1_amd64.deb
   sudo dpkg -i cudnn-local-repo-ubuntu2204-9.14.0_1.0-1_amd64.deb
   sudo cp /var/cudnn-local-repo-ubuntu2204-9.14.0/cudnn-*-keyring.gpg /usr/share/keyrings/
   sudo apt update
   
   # CRITICAL: Install CUDNN for YOUR CUDA version (e.g., cuda-12 for CUDA 12.x)
   # Check available packages:
   apt-cache search cudnn | grep cuda-
   
   # For CUDA 12.x:
   sudo apt install -y cudnn9-cuda-12 libcudnn9-cuda-12 libcudnn9-dev-cuda-12
   
   # For CUDA 11.x:
   # sudo apt install -y cudnn9-cuda-11 libcudnn9-cuda-11 libcudnn9-dev-cuda-11
 
   # Verify installation
   sudo ldconfig
   ldconfig -p | grep cudnn
   # Should show multiple libcudnn*.so files
   ```

   **Troubleshooting CUDNN Installation:**
   
   ```bash
   # Check if CUDNN was actually installed
   dpkg -l | grep cudnn
   
   # Check for version mismatch (common issue!)
   # If you see "cuda-13" but have CUDA 12.x, that's wrong!
   nvcc --version  # Your actual CUDA version
   dpkg -l | grep cudnn | grep cuda-  # What CUDNN thinks it's for
   
   # If there's a mismatch, remove wrong version and install correct one:
   # For CUDA 12.x with wrong CUDNN 13:
   sudo apt remove -y cudnn9-cuda-13* libcudnn9-cuda-13* libcudnn9-dev-cuda-13*
   sudo apt install -y cudnn9-cuda-12 libcudnn9-cuda-12 libcudnn9-dev-cuda-12
   
   # Refresh library cache
   sudo ldconfig
   
   # Verify again
   ldconfig -p | grep cudnn
   ```

   **If you still can't install CUDNN, you can use CPU-only mode:**
   
   ```bash
   # Force TensorFlow to use CPU instead of GPU
   export CUDA_VISIBLE_DEVICES=""
   
   # Add to your environment permanently
   echo 'export CUDA_VISIBLE_DEVICES=""' >> ~/hailo_env/bin/activate
   ```

   > **Note:** Without CUDNN, Hailo tools will error when trying to use GPU. Use `CUDA_VISIBLE_DEVICES=""` to force CPU mode. Compilation will be slower but still work.

4. **Create Clean Python Environment**

   **Recommended approach:** Create a fresh virtual environment and install only the wheel files:

   ```bash
   # Create a new virtual environment
   python3 -m venv hailo_venv
   cd hailo_venv/
   source bin/activate
   
   # Upgrade pip
   pip install --upgrade pip
   
   # Install Hailo wheel files (adjust paths as needed)
   pip install ../hailo_dataflow_compiler-3.33.0-py3-none-linux_x86_64.whl
   pip install ../hailort-4.23.0-cp310-cp310-linux_x86_64.whl
   pip install ../hailo_model_zoo-2.17.0-py3-none-any.whl
   ```

5. **Clone Hailo Model Zoo**

   ```bash
   cd ~
   git clone https://github.com/hailo-ai/hailo_model_zoo.git
   cd hailo_model_zoo
   ```

6. **Verify Installation**

   ```bash
   # Activate your environment if not done already
   cd ~/hailo_venv
   source bin/activate
   
   # Test the tools
   hailomz --help
   hailo compiler --help
   
   # Check that Python packages are installed
   pip list | grep hailo
   ```

### Recommended Workflow

For the best experience:
1. Use **native installation** if possible
2. Allocate a dedicated Ubuntu machine or partition
3. Ensure NVIDIA drivers are up to date (version 525+ for CUDA 12.x)
4. Use an SSD for faster calibration

---

## 1. Prepare Calibration Images

Hailo needs a small unlabeled image set (≈ 200–1000 images) from your dataset to calibrate the quantization step.

```bash
scp -r images_raw <USER>@<HOSTNAME>:~/hailo_ai_sw_suite/hailo_venv/
# Copy random training/validation images here
```

---

## 2. Create the yaml File

```bash
nano yolov11n_2cls.yaml
```

```text
base:
  - base/yolov8.yaml

postprocessing:
  device_pre_post_layers:
    nms: true
  hpp: true

network:
  network_name: yolov11n_2cls

paths:
  network_path:
    - /home/joe/scratch/hailo_venv/yolov11n_finetune.onnx

evaluation:
  classes: 2
  postprocess_config: /home/joe/scratch/hailo_venv/yolov11n_2cls.nms.json
```

---

## 3. Create the Model Script

The Hailo compiler needs to know the post-processing configuration.

Create `y11.alls`:

```bash
nano y11.alls
```

```text
normalization1 = normalization([0.0, 0.0, 0.0], [255.0, 255.0, 255.0])
nms_postprocess("/home/joe/scratch/hailo_venv/yolov11n_2cls.nms.json", meta_arch=yolov8, engine=cpu)
allocator_param(width_splitter_defuse=disabled)
```

---

## 4. Create the json File

```bash
nano yolov11n_2cls.nms.json
```

```text
{"nms_scores_th": 0.2, "nms_iou_th": 0.7, "image_dims": [640, 640], "max_proposals_per_class": 100, "classes": 2, "regression_length": 16, "background_removal": false, "background_removal_index": 0, "bbox_decoders": [{"name": "", "stride": 8, "reg_layer": "", "cls_layer": ""}, {"name": "", "stride": 16, "reg_layer": "", "cls_layer": ""}, {"name": "", "stride": 32, "reg_layer": "", "cls_layer": ""}]}
```

---

## 4. Run the Compilation

Activate your **Hailo-8 environment**.

```bash
hailomz compile --ckpt /home/joe/scratch/hailo_venv/yolov11n_finetune.onnx --calib-path images_raw --yaml /home/joe/scratch/hailo_venv/yolov11n_2cls.yaml --hw-arch hailo8  --model-script /home/joe/scratch/hailo_venv/y11.alls
```

### Key flags
| Option | Purpose |
|---------|----------|
| `--ckpt` | Path to your ONNX model |
| `--calib-path` | Directory of calibration images |
| `--yaml` | Network config (use `yolov8n.yaml` or `yolov11n.yaml` if present) |
| `--hw-arch hailo8` | Target chip (for Pi AI Hat / Hailo-8L) |
| `--model-script y11.alls` | Explicit post-processing |

After successful compilation, you’ll see:

```
Saved HAR to yolov11n.har
Saved HEF to yolov11n.hef
```

---

## 5. Deploy to Raspberry Pi AI Kit

1. Copy `yolov11n.hef` to the Pi:

   ```bash
   scp yolov11n.hef rcr@<HOSTNAME>:~
   ```

2. On the Pi, verify the device:

   ```bash
   hailortcli fw-control identify
   ```

3. Test inference:

   ```bash
   hailortcli run yolov11n.hef
   ```

For application-level use, load it with the Python or C++ `hailort` API.

---

## 6. Troubleshooting

### Environment Issues

| Symptom | Likely cause / fix |
|----------|--------------------|
| `No DNN in stream executor` or `InvalidArgumentError` with TensorFlow | CUDNN version mismatch or not installed. Check: `dpkg -l \| grep cudnn` vs `nvcc --version`. Must have matching versions (e.g., CUDNN cuda-12 for CUDA 12.x). Or use CPU mode: `export CUDA_VISIBLE_DEVICES=""` |
| `hailomz: command not found` | Activate your virtual environment: `source ~/hailo_venv/bin/activate` |
| `ImportError: cannot import name 'one_of' from 'pyparsing'` | Dependency conflict in installer environment. Use wheel-only installation (Section 5) instead of installer environment. |
| `ERROR: No matching distribution found for hailort` | HailoRT is not on PyPI. Use the wheel file from the Hailo installer. |
| `WARNING: CUDNN version should be 8.9 or higher, found ..` | Hailo installer detection issue. Verify CUDNN with `ldconfig -p \| grep cudnn`. If installed, proceed anyway. |
| `WARNING: Recommended package Node.js not found` | Using nvm instead of system-wide Node.js. Safe to ignore - nvm works fine. Verify with `node -v`. |
| `ERROR: Requirement libgraphviz-dev not found` | Install with `sudo apt install libgraphviz-dev` then re-run installer. |
| GPUs not visible in Docker | Install `nvidia-container-toolkit` and use `--gpus all` flag. Verify with `nvidia-smi` inside container. |
| `ImportError: libhailo.so` | Hailo SDK not installed or not in library path. Reinstall or check `LD_LIBRARY_PATH`. |
| Docker container exits immediately | Check Docker logs: `docker logs <container_id>`. May need to specify entrypoint or run interactively with `-it`. |
| Slow compilation without GPU | Normal. GPU acceleration speeds up optimization. Consider cloud GPU instances (AWS g4dn, GCP T4). |

### Compilation Issues

| Symptom | Likely cause / fix |
|----------|--------------------|
| ONNX has only 1 output (`output0`) instead of 6 | Model exported with `simplify=True`. Re-export with `simplify=False dynamic=False opset=11`. See Section 2. |
| `ValueError: None values not supported` during calibration | The post-process could not find reg/cls pairs — check the six output names and `y11.model`. |
| "invalid choice: 'hailo8'" | You're using the Hailo-15 SDK. Install the **Hailo-8 legacy AI Suite v3/v4**. |
| Poor accuracy | Provide ≥ 200 calibration images and use a realistic dataset. |
| No `.hef` produced | Check for earlier "Optimization" or "Statistics" exceptions; usually due to incorrect output mapping. |
| `CUDA out of memory` | Reduce batch size in calibration or use CPU mode. |
| Python version mismatch | Hailo requires Python 3.10. Use `python3.10 -m venv` to create environment. |

---

## 7. Summary

**Pipeline overview**

1. Set up Hailo development environment (native or Docker with GPU support).  
2. Fine-tune YOLOv11n with Ultralytics.  
3. Export to static ONNX.  
4. Prepare 200–1000 calibration images.  
5. Confirm 6 outputs (reg/cls pairs).  
6. Create `y11.model` for Hailo post-processing.  
7. Run `hailomz compile` with `--hw-arch hailo8`.  
8. Deploy the generated `.hef` to your Raspberry Pi AI Kit.

You now have a fully quantized and compiled YOLOv11n detector ready to run on the Hailo-8 accelerator.
