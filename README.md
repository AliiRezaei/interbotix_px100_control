# Interbotix PX100 Control
px100 interbotix x-series arm dynamics and control in ROS
This package allows you to control the interbotix x-series arm manipulator spesefically PX100 arm.
this package writen in the ROS noetic distribution. for generating the arm dynamics, we used [sympybotics](https://github.com/cdsousa/SymPyBotics) with some modifications.
Follow the steps below to install and use the package:

## Installation

Before using the package, make sure you have installed the ROS noetic.

## Setting Permissions

In this section, you need to change your serial-port permission and activate read and write permissions. To do this, open a new terminal and enter the following command :

```bash
$ sudo chmod 666 /dev/ttyACM0
```

## Uploading Code

Upload the code to your Arduino board. You can use either servo_ros_publisher or servo_ros_subscriber. For the first case, upload servo_ros_subscriber to your Arduino board, and then run a node from rosserial_python package. This node is called serial_node.py. Open a new terminal and execute the following command:

```bash
$ rosrun rosserial_python serial_node.py /dev/ttyACM0
```

By running the above command, your connection with Arduino and ROS has been set up. Now, you can execute servo_pose_cmd node and send position commands for your servo motor. Open a new terminal and try the following command:

```bash
$ rosrun servo_control servo_pose_cmd
```

After executing this command, you should see your servo motor move.

Alternatively, you can upload another Arduino code called servo_ros_publisher. Similarly, connect your board and upload code from Arduino IDE. Now, check rostopics by running the following command:

```bash
$ rostopic list
```

You can visualize transformed data in the servo_pose topic by echoing it in the terminal with the following command :

```bash
$ rostopic echo /servo_pose
```

You will see the servo position printed in the terminal environment.

## Conclusion

With this package, you can easily control your servo motor using ROS commands. By following these simple steps, you can connect ROS and Arduino and control your servo motor's position.
