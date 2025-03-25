---
sidebar_position: 4
---

# Installing Ubuntu Virtual Machine on Host PC

After [setting up the Jetson Nano](/docs/installation/installation_jetson), the next step is to install an Ubuntu virtual machine on your host computer. This VM will serve as your development environment for programming and controlling the JetRacer.

## Overview

In this guide, you'll learn how to:

1. Download and install VMware
2. Download the pre-configured Ubuntu VM image
3. Import and configure the virtual machine
4. Connect the VM to your JetRacer

## Step 1: Install VMware

Since VMware got acquired by Broadcom the installation got a bit cumbersome and you have to create a Broadcom account

1. Go to [Broadcom Registration](https://profile.broadcom.com/web/registration) and make a new Broadcom account
2. During the registration you will be asked for a Job title, choose “other”
3. After registering you will be asked to build your profile. We aren’t going to do that, click: “I’ll do it later”.
4. After Registering go to this link:
   [VMware Workstation Player 16.2.2](https://support.broadcom.com/group/ecx/productfiles?subFamily=VMware%20Workstation%20Player&displayGroup=VMware%20Workstation%20Player%2016&release=16.2.2&os=&servicePk=208715&language=EN&freeDownloads=true)
   It will ask you to log in, when logging in use your e-mail address that you used when making your account as the username.
5. Download VMware Workstation 16.2.2 Player for Windows 64-bit Operating Systems, this version has been tested and proved to work with our VM.

## Step 2: Download the Ubuntu VM Image

1. Download the pre-configured Ubuntu VM image from [Google Drive](https://drive.google.com/file/d/1u98h-GooQYXppDZCbRxvvfh2Zw3-cUcq/view?usp=sharing)
2. Extract the downloaded .zip file to a location on your computer (preferably with at least 20GB of free space)

## Step 3: Import the VM into VMware

1. Open VMware Workstation Player
2. Click on the top left Player dropdown > File > Open
3. Browse to and select the extracted .vmx file
4. Click "Open"
5. If prompted about whether the virtual machine was moved or copied, select "I copied it"

## Step 4: Configure the Virtual Machine

### System Configuration

1. Select the imported VM and click "Edit virtual machine settings"
2. Under "Memory", allocate at least 4GB of RAM (8GB recommended)
3. Under "Processors", allocate at least 2 processor cores
4. Click "OK" to save the settings

### Network Configuration

1. Go to "Edit" → "Virtual Network Editor" (you may need admin rights)
2. Ensure Bridge networking is enabled for internet access (should be automatic)
3. Click "OK" to save the settings

## Step 5: Start and Login to the VM

1. Select the VM and click "Play virtual machine"
2. Wait for Ubuntu to boot up (this may take a few minutes on first boot)
3. Login with the default credentials:
   - Username: `jetson`
   - Password: `jetson`

## Troubleshooting

- **VM performance is slow**: Increase the allocated RAM and CPU cores in VM settings
- **Cannot connect to the internet**: Ensure the networking is set to Bridged(automatic)
- **VMware errors on startup**: Make sure virtualization is enabled on your PC

## Next Steps

Now that you have both your Jetson Nano and Ubuntu VM set up, you're ready to proceed with controlling your JetRacer's movements.

## References

For more detailed instructions, refer to the [official Waveshare JetRacer ROS AI Kit Tutorial III](https://www.waveshare.com/wiki/JetRacer_ROS_AI_Kit_Tutorial_III%3A_Install_Ubuntu_Virtual_Image).
