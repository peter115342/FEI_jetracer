---
sidebar_position: 3
---

# Installing Jetson Nano Image

After [completing the assembly](/docs/assembly/assembly) of your JetRacer, the next step is to set up the software environment by installing the Jetson Nano image.

## Overview

The Jetson Nano requires a properly configured Ubuntu-based operating system with the necessary drivers and software packages for controlling the JetRacer components. This guide will walk you through:

1. Programming the Jetson Nano image to an SD card
2. Setting up Wi-Fi connectivity
3. Establishing remote access to your Jetson Nano
4. Verifying the installation

## Step 1: Flash Jetson Nano Image

> **Note:** If you received your JetRacer kit with a pre-flashed SD card, you can skip this step.

To install the operating system:

1. Obtain an SD card (minimum 64GB recommended)
2. Download the JetRacer ROS Image from [Google Drive](https://drive.google.com/file/d/16OBLRNlrZaSkhVcC4xJ6VugtChmZw1_B/view?usp=sharing)
3. Format your SD card using [SDFormatter](https://files.waveshare.com/upload/3/31/Panasonic_SDFormatter_%289%29.zip)
   > **Important:** Disconnect other storage devices to avoid formatting the wrong device
4. Use [Rufus](https://rufus.ie/en/) or [Balena Etcher](https://etcher.balena.io/) to flash the ISO onto the SD card
5. Safely eject the SD card from your computer

## Step 2: Connect Jetson Nano to Wi-Fi

You can connect your Jetson Nano to Wi-Fi either with a display or headlessly.

### Option A: With Display

1. Insert the SD card into the Jetson Nano's SD card slot (located on the back of the module)
2. Connect peripherals (monitor, keyboard, mouse) to the Jetson Nano
3. Power on the JetRacer using the power switch
4. Log in with default credentials:
   - Username: `jetson`
   - Password: `jetson`
5. Connect to Wi-Fi using the network manager in the Ubuntu interface

### Option B: Without Display (Headless Setup)

If you don't have access to a display:

1. Create a text file named `wpa_supplicant.conf` with the following content:

   ```
   country=US
   ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
   update_config=1

   network={
       ssid="YOUR_WIFI_NAME"
       psk="YOUR_WIFI_PASSWORD"
       key_mgmt=WPA-PSK
   }
   ```

2. Replace `YOUR_WIFI_NAME` and `YOUR_WIFI_PASSWORD` with your actual Wi-Fi credentials
3. Place this file in the root directory of the SD card
4. Insert the SD card into the Jetson Nano and power it on

## Step 3: Remote Access

### SSH Remote Login

1. Get your Jetson Nano's IP address from the OLED display on the mainboard check your router's connected devices
2. Connect via SSH from your computer terminal:
   ```bash
   ssh jetson@<IP-ADDRESS>
   ```
3. Enter the password when prompted (default: `jetson`)

## Step 4: Verify Installation

To verify that everything is working properly:

1. Log in to your Jetson Nano
2. Check system information:
   ```bash
   uname -a
   ```
   This should show Linux running on aarch64 architecture
3. Verify CUDA installation:
   ```bash
   nvcc --version
   ```
4. Check for ROS installation:
   ```bash
   rosversion -d
   ```

## Next Steps

Now that you have successfully installed and accessed your Jetson Nano, you're ready to proceed with installing the Ubuntu Virtual Machine on your host PC.

## Troubleshooting

- **Cannot connect to Wi-Fi**: Try using a wired Ethernet connection first, then set up Wi-Fi
- **SSH connection refused**: Ensure the Jetson Nano has fully booted (may take up to 2 minutes on first boot)
- **Performance issues**: The Jetson Nano can become hot during operation; make sure the heatsink fan is connected and running

## References

For more detailed instructions, refer to the [official Waveshare JetRacer ROS AI Kit Tutorial II](https://www.waveshare.com/wiki/JetRacer_ROS_AI_Kit_Tutorial_II%3A_Install_Jetson_Nano_Image).
