---
sidebar_position: 8
---

# FEI DonkeyCar Installation on Jetson Nano

This tutorial guides you through installing our customized version of DonkeyCar on the Jetson Nano for the FEI JetRacer project. This setup enables autonomous driving capabilities using machine learning and computer vision.

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

## Step 2: Install FEI DonkeyCar

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
