---
sidebar_position: 1
---

# FEI JetRacer Tutorial

Welcome to the **FEI JetRacer** documentation site. This guide will help you set up and use the JetRacer ROS AI Kit.

## About JetRacer

JetRacer is an AI-powered racing robot based on the NVIDIA Jetson Nano platform. It combines robotics, computer vision, and artificial intelligence to create an autonomous racing platform.

## What You'll Learn

In this tutorial series, you'll learn how to:

- Assemble the JetRacer ROS AI Kit
- Install and configure the Jetson Nano image
- Set up multi-machine communication
- Control the robot's movement
- Work with ROS topics and nodes
- Calibrate the robot's odometer
- Configure the camera and lidar

## Prerequisites

Before starting, make sure you have:

- JetRacer ROS AI Kit
- NVIDIA Jetson Nano
- MicroSD card (32GB or larger recommended)
- Power supply for Jetson Nano
- Computer with SSH client
- Basic knowledge of Linux commands

## Getting Started

Begin your journey with the JetRacer by following our step-by-step tutorials. Start with the [Assembly Guide](assembly) to build your robot.

## Robot Movement Control

Based on the Waveshare tutorial, here's how to control your JetRacer's movement:

1. First, add the serial user group to ensure proper permissions
2. Start the robot chassis control node
3. Use topic publishing for control
4. Control the robot movement with keyboard
5. Connect to gamepad control for easier operation

For detailed instructions, check out the section.

## Project Structure

This documentation is organized into the following sections:

1. **Assembly** - How to build your JetRacer
2. **Installation** - Setting up the Jetson Nano image
3. **Configuration** - Configuring multi-machine communication
4. **Movement Control** - Controlling the robot's movement
5. **ROS Topics** - Working with ROS nodes and topics
6. **Calibration** - Calibrating the robot's odometer
7. **Camera** - Setting up and using the camera
8. **Lidar** - Configuring and using the lidar

Let's get started with your JetRacer!

````

## Step 3: Create the Slovak intro.md file

After setting up your project, you'll need to create the Slovak translation. First, run:

```bash
npm run write-translations -- --locale sk
````

Then create the Slovak version of the intro:

```markdown:i18n/sk/docusaurus-plugin-content-docs/current/intro.md
---
sidebar_position: 1
---

# FEI JetRacer Tutoriál

Vitajte na dokumentačnej stránke **FEI JetRacer**. Táto príručka vám pomôže nastaviť a používať JetRacer ROS AI Kit.

## O JetRacer

JetRacer je pretekársky robot s umelou inteligenciou založený na platforme NVIDIA Jetson Nano. Kombinuje robotiku, počítačové videnie a umelú inteligenciu na vytvorenie autonómnej pretekárskej platformy.

## Čo sa naučíte

V tejto sérii tutoriálov sa naučíte:

- Zostaviť JetRacer ROS AI Kit
- Nainštalovať a nakonfigurovať obraz Jetson Nano
- Nastaviť komunikáciu medzi viacerými zariadeniami
- Ovládať pohyb robota
- Pracovať s témami a uzlami ROS
- Kalibrovať odometer robota
- Konfigurovať kameru a lidar

## Predpoklady

Pred začatím sa uistite, že máte:

- JetRacer ROS AI Kit
- NVIDIA Jetson Nano
- MicroSD kartu (odporúča sa 32GB alebo väčšia)
- Napájací zdroj pre Jetson Nano
- Počítač s SSH klientom
- Základné znalosti príkazov Linuxu

## Začíname

Začnite svoju cestu s JetRacerom pomocou našich podrobných tutoriálov. Začnite s [Príručkou na zostavenie](assembly) na zostavenie vášho robota.

## Ovládanie pohybu robota

Na základe tutoriálu od Waveshare, takto môžete ovládať pohyb vášho JetRacera:

1. Najprv pridajte skupinu sériového portu pre zabezpečenie správnych oprávnení
2. Spustite riadiaci uzol podvozku robota
3. Použite publikovanie tém na ovládanie
4. Ovládajte pohyb robota pomocou klávesnice
5. Pripojte sa ku gamepad ovládaniu pre jednoduchšiu prevádzku

Pre podrobné inštrukcie si pozrite sekciu.

## Štruktúra projektu

Táto dokumentácia je organizovaná do nasledujúcich sekcií:

1. **Zostavenie** - Ako zostaviť váš JetRacer
2. **Inštalácia** - Nastavenie obrazu Jetson Nano
3. **Konfigurácia** - Konfigurácia komunikácie medzi viacerými zariadeniami
4. **Ovládanie pohybu** - Ovládanie pohybu robota
5. **ROS Témy** - Práca s uzlami a témami ROS
6. **Kalibrácia** - Kalibrácia odometra robota
7. **Kamera** - Nastavenie a používanie kamery
8. **Lidar** - Konfigurácia a používanie lidaru

Poďme začať s vaším JetRacerom!
```

## Step 4: Create a Movement Control page based on the Waveshare tutorial

---

## sidebar_position: 4

# Robot Movement Control

This guide will walk you through controlling your JetRacer's movement using ROS.

## Step 1: Add serial User Group

First, check if the driver board is connected to Jetson Nano properly:

```bash
ls /dev
```

You should see `ttyACM0` and `ttyACM1` devices, where:

- `ttyACM0` is used to communicate with the microcontroller
- `ttyACM1` is used to communicate with the radar

Next, add the serial port user group to ensure proper permissions:

```bash
ls -l /dev/ttyACM*             # View the serial port user group as dialout
id -Gn                         # View the user group that the current user belongs to
sudo adduser jetbot dialout    # Add the current user to the dialout user group
```

Restart your Jetson Nano for the changes to take effect:

```bash
sudo reboot
```

## Step 2: Start the robot chassis control node

Connect to the robot via SSH and start the robot master node:

```bash
roscore
```

Starting the master node separately ensures it stays connected to the virtual machine even if the chassis node is closed.

Next, start the robot chassis node:

```bash
roslaunch jetracer jetracer.launch
```

## Step 3: Start Topic Publishing Node Control

To verify that multi-machine communication is working properly, open the Ubuntu virtual machine terminal and run:

```bash
rostopic list
```

If you see a list of topics, multi-device communication is successful.

## Step 4: Control the Robot Movement with Keyboard

You can control the robot using the keyboard with the teleop_twist_keyboard package:

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

This will display control instructions in the terminal. Use the following keys:

- `i`: Move forward
- `,`: Move backward
- `j`: Turn left
- `l`: Turn right
- `k`: Stop
- `q/z`: Increase/decrease max speeds by 10%
- `w/x`: Increase/decrease only linear speed by 10%
- `e/c`: Increase/decrease only angular speed by 10%

## Step 5: Connect the Virtual Machine to the Gamepad Control

For gamepad control, you'll need to install the joy package:

```bash
sudo apt-get install ros-noetic-joy
```

Connect your gamepad to the computer and check if it's recognized:

```bash
ls /dev/input/js0
```

Start the joy node:

```bash
rosrun joy joy_node
```

Then start the teleop node:

```bash
rosrun teleop_twist_joy teleop_node
```

Now you can control your JetRacer using the gamepad!

## Troubleshooting

If you encounter any issues:

1. Make sure all devices are on the same network
2. Check that the ROS_MASTER_URI and ROS_IP environment variables are set correctly
3. Verify that the serial port permissions are properly configured
4. Restart the ROS nodes if communication fails
