# Interbotix PX100 Control
px100 interbotix x-series arm dynamics and control in ROS
This package allows you to control the interbotix x-series arm manipulator spesefically PX100 arm.
this package writen in the ROS noetic distribution. for generating the arm dynamics, we used [sympybotics](https://github.com/cdsousa/SymPyBotics) with some modifications.
Follow the steps below to install and use the package:

## Installation

Before using the package, make sure you have installed the ROS noetic (or one of the ROS1 distributions).
also, you must install the interbotix ROS open source packages [see this link](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros1_packages.html). So clone the repository inside your ROS workspace by running the following command:

```bash 
$ git clone https://github.com/AliiRezaei/interbotix_px100_control.git
```
Its better clone the repository inside the `interbotix_ros_xsarms` directory 


## Usage

After properly install the requirements, open a new terminal and start `rosmater` by insert the command `roscore`.
in another terminal launch the PX100 rviz model by running the following command:

```bash 
$ roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=px100 use_sim:=true
```
Now, you can see the PX100 arm in rviz environment that is the sleep position. we develope two type of controllers for manipulate the arm inside it's addmisible workspace, the `PID` and `Adaptive` controllers. befor using the controllers, you must change the robot operating mode that's default mode is `position`. for using controllers you must change the operating mode to the `pwm` mode. for this purpose run the following command:

```bash 
$ rosservice call /px100/set_operating_modes "{cmd_type: 'group', name: 'arm', mode: 'pwm', profile_type: 'time', profile_velocity: 131, profile_acceleration: 25}" 
```
after properly change the operating mode, you are ready to use the `PID` or `Adaptive` controllers for controlling the PX100.

# PID Controller
open a new terminal and navigate it's where the repository is cloned. so change directory to the scripts folder by `cd scripts` command. now you can run the `main_pid.py` by gollowing command:

```bash
$ python3 main_pid.py
```
you see the robot start moving and go to the Home position with joints value $q = [0.0, 0.0, 0.0, 0.0]$ where all of the joints set in the zero. 




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
