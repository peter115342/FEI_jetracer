---
sidebar_position: 6
---

# Robot Movement Control

## Step 1: Add serial User Group

Enter the command ls /dev in jetson nano to check whether the driver board is connected to Jetson normally and whether ttyACM0 and ttyACM1 devices are found 【where ttyACM0 is used to communicate with the microcontroller, and ttyACM1 is used to communicate with the radar】.

```bash
ls /dev
```

Enter the following command to add the serial port user group, otherwise, you will not have permission to operate the serial port. Using the configured system has already added permissions, you can skip this step.

```bash
ls -l /dev/ttyACM*             #View the serial port user group as dialout
id -Gn                         #View the user group that the current user belongs to, the first one is the current user
sudo adduser jetbot dialout    #Add the current user to the user group dialout where the serial port is located
```

Enter the command "sudo reboot" to restart, the password is jetson 【Note: The above addition command can take effect only after restarting, and when there is no serial port permission, the robot chassis node will always report an error after booting up】.

```bash
sudo reboot
```

## Step 2: Start the robot chassis control node

Connect SSH to the robot to open the terminal and enter the following command to start the robot master node.

```bash
roscore
```

The following roslaunch command will also automatically start the master node, but the purpose of starting the master node alone is to keep connected to the virtual machine all the time. Otherwise, the master node will be automatically closed when the chassis node is closed, resulting in the virtual machine being disconnected.
Enter the following command to start the robot chassis node.

```bash
roslaunch jetracer jetracer.launch
```

Open the Ubuntu virtual machine terminal and enter the following command in the terminal to verify that the multi-machine communication connection is normal.

```bash
rostopic list
```

If this is the content you get after running the command,it means that the multi-device communication is successful:

- /cmd_vel is the topic about robot movement and is controlled by the robot.
- /imu is the robot IMU topic.
- /motor/\* indicates the actual encoded speed and set the speed of the left and right motors.
- /odom encodes the odometer for the robot.
- /odom_combined is the robot fusion odometer, which is obtained by combining the encoded odometer with IMU data.

## Step 3: Start Topic Publishing Node Control

You can publish topics through the Ubuntu virtual machine to control the movement of the robot.

```bash
rosrun rqt_publisher rqt_publisher
```

In the pop-up window, please select the topic /cmd_vel, and click "+" to create a new topic, /cmd_vel is the topic of controlling the motion of the robot, linear->x indicates the linear speed of the robot, and the range is -1.2m/s~1.2m/ s, angular->x represents the steering angle of the front tire of the robot, not the angular velocity, ranging from -0.6 radians to 0.6 radians.
Change the value of linear.x to 0.5, right-click and select publish selected Once to publish a topic, the robot will move forward 0.5 meters and then stop; if linear.x is changed to 0, angular. z is changed to 0.6, the robot will turn left maximum angle of motion.

Note: The topic we open here is cmd_vel, select Twist type, and select other topics, the car may not move.

## Step 4: Control the Robot Movement with Keyboard

Open a new terminal, run the keyboard control node to control the robot movement with the keyboard.

```bash
roslaunch jetracer keyboard.launch
```

After successful operation, an interface appears displaying the control mapping, and you can control the robot movement by pressing the keyboard.

## Step 5: Connect the Virtual Machine to the Gamepad Control

Connect the USB receiver of the gamepad to the host computer, and a box will pop up, select Connect to Virtual Machine -> Ubuntu JetRacer

Run the following command in the virtual machine to test whether the gamepad is connected normally(you should see a js0 input device).

```bash
ls /dev/input/
```

Test the input of the gamepad using the following command

```bash
jstest /dev/input/js0
```

Press different buttons on the gamepad, and see the corresponding values change.

Start the joystick node.

```bash
roslaunch jetracer joy.launch
```

Turn on the gamepad, press the HOME button and a red light will be on, then press and hold the L1 button in the upper left corner, and at the same time control the left joystick left and right directions to control the steering of the robot's servos, and the right joystick up and down to control the robot to move forward and backward. Release the L1 key, and the robot stops.

## Troubleshooting

### Robot Doesn't Move

1. Verify the battery is charged
2. Ensure the controller node is running on the Jetson Nano
3. Check for any error messages in the terminal

### Gamepad Not Recognized

1. Verify the gamepad is properly connected to the host PC
2. Ensure the VM has access to the USB device
3. Check if the correct joystick device is being used (e.g., /dev/input/js0)

## References

For more detailed instructions, refer to the [official Waveshare JetRacer ROS AI Kit Tutorial V: Robot Movement Control](https://www.waveshare.com/wiki/JetRacer_ROS_AI_Kit_Tutorial_V%3A_Robot_Movement_Control).
