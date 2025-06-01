---
sidebar_position: 9
---

# Object Detection with OpenCV and CUDA

This tutorial covers setting up object detection capabilities on your FEI JetRacer using OpenCV with CUDA acceleration and YOLOv4-Tiny for real-time performance.

## Overview

We'll use:

- **OpenCV with CUDA**: For accelerated computer vision operations
- **YOLOv4-Tiny**: A lightweight object detection model optimized for embedded devices
- **cuDNN**: NVIDIA's deep learning acceleration library

## Note

**If you are using our pre-configured JetRacer image, some of this setup is already complete!**

**You can skip the OpenCV with CUDA installation steps below and go directly to [Installation Verification](#verify-installation).**

## Manual Installation

### Step 1: Install OpenCV with CUDA Support

We'll use the Qengineering installation script which automates the complex process of building OpenCV with CUDA and cuDNN support.

#### Download the Installation Script

```bash
cd ~
```

```bash
git clone https://github.com/Qengineering/Install-OpenCV-Jetson-Nano.git
```

```bash
cd Install-OpenCV-Jetson-Nano
```

#### Prepare Your System

Before running the installation script, ensure your system is ready:

```bash
sudo apt update && sudo apt upgrade -y
```

```bash
sudo apt autoremove -y
```

Free up disk space and increase swap:

```bash
sudo systemctl disable nvzramconfig
```

```bash
sudo fallocate -l 4G /var/swapfile
```

```bash
sudo chmod 600 /var/swapfile
```

```bash
sudo mkswap /var/swapfile
```

```bash
sudo swapon /var/swapfile
```

```bash
sudo swapon -s
```

#### Run the Installation Script

> **Warning**: This process can take 2-4 hours to complete. Ensure stable power supply and don't interrupt the process.

```bash
chmod +x ./OpenCV-4-11-0.sh
```

```bash
./OpenCV-4-11-0.sh
```

The script will:

- Install all necessary dependencies
- Download OpenCV source code
- Configure build with CUDA support
- Compile OpenCV (this takes the longest time)
- Install the compiled libraries

#### Verify Installation

After installation completes, verify OpenCV with CUDA support:

```bash
python3 -c "import cv2; print('OpenCV version:', cv2.__version__); print('CUDA devices:', cv2.cuda.getCudaEnabledDeviceCount())"
```

You should see output similar to:

```
OpenCV version: 4.11.1
CUDA devices: 1
```

### Step 2: Configure Environment

Ensure the OpenCV installation is properly configured in your DonkeyCar environment:

```bash
source ~/env/bin/activate
```

```bash
cd ~/mycar
```

Test OpenCV in your virtual environment:

```bash
python3 -c "import cv2; print('OpenCV loaded successfully with version:', cv2.__version__)"
```

## Step 3: Setup YOLOv4-Tiny

Now we'll add YOLOv4-Tiny object detection model to your DonkeyCar setup.

### Create Models Directory

```bash
cd ~/mycar
```

```bash
mkdir -p models
```

```bash
cd models
```

### Download YOLOv4-Tiny Files

Download the pre-trained YOLOv4-Tiny model files:

```bash
wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4-tiny.weights
```

```bash
wget https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4-tiny.cfg
```

```bash
wget https://raw.githubusercontent.com/AlexeyAB/darknet/master/data/coco.names
```

### Verify Model Files

Check that all required files are present:

```bash
ls -la ~/mycar/models/
```

You should see:

- `yolov4-tiny.weights` (approximately 23MB)
- `yolov4-tiny.cfg` (configuration file)
- `coco.names` (class labels file)

### Test Object Detection

Test by running the DonkeyCar web interface:

```bash
cd ~/mycar
```

```bash
python manage.py drive
```

You should see the object detection in the camera feed in the web interface.

## Performance Tips

### Optimize for Real-time Performance

1. **Resolution Tuning**: Balance between accuracy and speed by adjusting input resolution

2. **Detection Frequency**: Run detection every few frames rather than every frame

3. **ROI Processing**: Only process regions of interest in the image

## Troubleshooting

### OpenCV Installation Issues

If the installation script fails:

1. **Insufficient storage**: Ensure at least 16GB free space
2. **Memory issues**: Increase swap file size
3. **Power issues**: Use adequate power supply

### CUDA Not Working

If CUDA acceleration isn't working:

```bash
# Check CUDA installation
nvcc --version
```

```bash
# Verify cuDNN
cat /usr/local/cuda/include/cudnn_version.h | grep CUDNN_MAJOR -A 2
```

### Model Loading Errors

If YOLO models fail to load:

1. Verify file integrity:

   ```bash
   md5sum ~/mycar/models/yolov4-tiny.weights
   ```

2. Check file permissions:
   ```bash
   chmod 644 ~/mycar/models/*
   ```

## References

- [Qengineering OpenCV Installation Script](https://github.com/Qengineering/Install-OpenCV-Jetson-Nano)
- [YOLOv4-Tiny Paper](https://arxiv.org/abs/2004.10934)
- [OpenCV DNN Module Documentation](https://docs.opencv.org/master/d2/d58/tutorial_table_of_content_dnn.html)
- [NVIDIA Jetson Nano Developer Kit](https://developer.nvidia.com/embedded/jetson-nano-developer-kit)
