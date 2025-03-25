---
sidebar_position: 5
---

# Multi-Machine Communication Setup

After [installing Ubuntu VM on your host PC](/docs/installation/installation_host), the next step is to configure multi-machine communication between your host VM and the Jetson Nano. This allows your development computer to interact with the JetRacer remotely.

## Overview

ROS (Robot Operating System) is designed to work across multiple machines, allowing you to:

- Run resource-intensive computations on your more powerful host PC
- Control the JetRacer remotely
- Visualize sensor data and debug information on your development machine
- Deploy code seamlessly between development and robot environments

In this guide, you'll learn how to:

1. Configure network settings on both machines
2. Set up ROS environment variables
3. Test the multi-machine communication
4. Troubleshoot common issues

## Prerequisites

- Jetson Nano with ROS installed and connected to your network
- Ubuntu VM running on your host PC
- Both machines connected to the same network

## Step 1: Determine IP Addresses

First, you need to know the IP addresses of both machines:

### On Jetson Nano:

Look at the OLED display of the mainboard to see the IP (e.g., `192.168.1.100`).

### On Ubuntu VM:

```bash
ifconfig
```

Look for the `ens33` or similar interface and note the IP address (e.g., `192.168.1.200`).

## Step 2: Configure Hosts File on Both Machines

You'll need to edit the hosts file on both machines to enable hostname resolution:

### On Jetson Nano:

```bash
sudo nano /etc/hosts
```

Add the following lines:

```
127.0.0.1       localhost
127.0.1.1       jetson
192.168.1.100   jetson    # Your Jetson Nano's IP
192.168.1.200   ubuntu    # Your Ubuntu VM's IP
```

Save and exit (Ctrl+X, then Y, then Enter).

### On Ubuntu VM:

```bash
sudo nano /etc/hosts
```

Add the following lines:

```
127.0.0.1       localhost
127.0.1.1       ubuntu
192.168.1.100   jetson    # Your Jetson Nano's IP
192.168.1.200   ubuntu    # Your Ubuntu VM's IP
```

Save and exit.

## Step 3: Configure ROS Environment Variables

ROS needs to know which machine is the master and how to communicate between machines:

### On Jetson Nano (ROS Master):

Edit the `.bashrc` file:

```bash
nano ~/.bashrc
```

Add these lines at the end:

```bash
export ROS_MASTER_URI=http://jetson:11311
export ROS_HOSTNAME=jetson
```

Apply the changes:

```bash
source ~/.bashrc
```

### On Ubuntu VM:

Edit the `.bashrc` file:

```bash
nano ~/.bashrc
```

Add these lines at the end:

```bash
export ROS_MASTER_URI=http://jetson:11311
export ROS_HOSTNAME=ubuntu
```

Apply the changes:

```bash
source ~/.bashrc
```

## Step 4: Test the Communication

Now let's test if the multi-machine communication is working:

### On Jetson Nano:

Start the ROS master:

```bash
roscore
```

### On Ubuntu VM:

Check if you can see the ROS master:

```bash
rostopic list
```

If you see a list of topics (even if it's just `/rosout` and `/rosout_agg`), the communication is working!

## Step 5: Run a Test Example

Let's run a simple publisher/subscriber example to verify the communication:

### On Jetson Nano:

Start an example receiving node:

```bash
rosrun rospy_tutorials listener
```

### On Ubuntu VM:

Start an publishing receiving node:

```bash
rosrun rospy_tutorials talker
```

You should see "hello world" messages appearing repeatedly being received on the Jetson side.

## Troubleshooting

### Cannot Connect to ROS Master

1. Verify both machines can ping each other:

   ```bash
   ping jetson  # From Ubuntu VM
   ping ubuntu  # From Jetson Nano
   ```

2. Verify ROS environment variables:
   ```bash
   echo $ROS_MASTER_URI
   echo $ROS_HOSTNAME
   ```

### Connection Timeouts

1. Make sure both machines are on the same network
2. Check if the IP addresses have changed (common with DHCP)
3. Update the hosts file if IP addresses change

## References

For more detailed instructions, refer to the [official Waveshare JetRacer ROS AI Kit Tutorial IV](https://www.waveshare.com/wiki/JetRacer_ROS_AI_Kit_Tutorial_IV%3A_Configure_Multi-machine_Communication).
