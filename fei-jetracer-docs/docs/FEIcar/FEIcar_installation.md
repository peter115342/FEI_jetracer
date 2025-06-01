---
sidebar_position: 8
---

# FEI DonkeyCar(FEIcar) Installation on Jetson Nano

This tutorial guides you through installing our customized version of DonkeyCar on the Jetson Nano for the FEI JetRacer project. This setup enables autonomous driving capabilities using machine learning and computer vision.

## Quick Start - Pre-configured Image (Recommended)

**If you want to skip the entire installation process**, we provide a pre-configured SD card image with everything already set up:

**Download Link**: [FEI JetRacer Complete Image](https://drive.google.com/file/d/1OVulgYBTdY4HOwtRhksuTYHidAfxd2wF/view?usp=sharing)

### What's Included in the Image

The pre-configured image contains:

- Ubuntu 18.04 with JetPack 4.5
- FEI DonkeyCar with mycar application pre-created
- OpenCV with CUDA and cuDNN support
- YOLOv4-tiny for object detection
- All dependencies and libraries pre-installed
- Ready-to-use configuration for JetRacer hardware

### Flashing the Image to SD Card

1. **Download the image** (approximately 8GB compressed)
2. **Extract the image** if it's compressed
3. **Flash to SD card** using one of these tools:

   **Option A: Raspberry Pi Imager (Recommended)**

   - Download [Raspberry Pi Imager](https://www.raspberrypi.org/software/)
   - Select "Use custom image" and choose the downloaded file
   - Select your SD card (32GB or larger)
   - Click "Write"

   **Option B: Balena Etcher**

   - Download [Balena Etcher](https://www.balena.io/etcher/)
   - Select the image file
   - Select your SD card
   - Click "Flash"

   **Option C: Command Line (Linux/macOS)**

   ```bash
   # Find your SD card device (be careful!)
   lsblk

   # Flash the image (replace /dev/sdX with your SD card device)
   sudo dd if=fei-jetracer-image.img of=/dev/sdX bs=4M status=progress
   ```

4. **Insert the SD card** into your Jetson Nano and boot

### First Boot Setup

After flashing and booting:

1. The system will automatically resize the filesystem
2. Default credentials:
   - Username: `jetson`
   - Password: `jetson`
3. The DonkeyCar environment is ready at `~/mycar`
4. Activate the virtual environment:
   ```bash
   source ~/env/bin/activate
   ```

**If you use this pre-configured image, you can skip the rest of this tutorial and the [Object Detection setup](/docs/FEIcar/object_detection) too.**

---

## Manual Installation (Advanced Users)

If you prefer to install everything manually or want to understand the process, continue with the steps below.

## Overview

DonkeyCar is an open-source platform for building and racing autonomous RC cars. Our version includes:

- Preparation for object detection with YOLOv4-tiny
- Improved web interface

> **Note**: Default DonkeyCar is already installed by default if you use the configured image, so we skip some steps in this tutorial.

## Step 1: Setup Virtual Environment

Create and activate a Python virtual environment to isolate the DonkeyCar installation:

```bash
pip3 install virtualenv
```

```bash
python3 -m virtualenv -p python3 env --system-site-packages
```

```bash
source env/bin/activate
```

> **Important**: All subsequent DonkeyCar operations must be performed within this virtual environment. Remember to run `source env/bin/activate` every time you open a new terminal session.

## Step 2: Install FEIcar

Clone and install our customized version of DonkeyCar:

```bash
git clone -b 4_5_1_FEI https://github.com/peter115342/FEIcar
```

```bash
cd donkeycar
```

```bash
pip install -e .[nano]
```

## Step 3: Create Your Car Application

Create a new DonkeyCar application instance:

```bash
donkey createcar --path ~/mycar
```

This command creates a new directory `~/mycar` with all the necessary files to run your autonomous car. In our FEIcar fork the camera is already pre-configured to work with our JetRacer kit.

## Verification

Test your installation by running the DonkeyCar web interface:

```bash
cd ~/mycar
```

```bash
python manage.py drive
```

Open a web browser and navigate to `http://<jetson-ip>:8887` to access the control interface.

> **Note**: Object detection functionality will only be available after completing the [Object Detection setup tutorial](/docs/FEIcar/object_detection). The basic DonkeyCar interface will work without object detection, but advanced features like YOLO object detection require additional setup.

## Troubleshooting

### Virtual Environment Issues

If you encounter import errors, ensure you're in the virtual environment:

```bash
source env/bin/activate
```

### Camera Not Detected

If the camera isn't working:

1. Check camera connection
2. Verify camera is enabled in Jetson settings
3. Test camera with:

```bash
nvgstcapture-1.0
```

### Memory Issues

If you encounter out-of-memory errors during installation:

1. Increase swap space
2. Close other applications
3. Install packages one at a time

### Permission Errors

For permission-related issues:

```bash
sudo chown -R $USER:$USER ~/mycar
```

## References

- [Original Waveshare DonkeyCar Tutorial](https://www.waveshare.com/wiki/DonkeyCar_for_JetRacer_ROS_Tutorial_I%3A_Install_Jetson_Nano)
- [DonkeyCar Official Documentation](http://docs.donkeycar.com/)
