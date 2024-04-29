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

After properly install the requirements, open a new terminal and start `rosmaster` by insert the command `roscore`.
in another terminal launch the PX100 rviz model by running the following command:

```bash 
$ roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=px100 use_sim:=true
```
Now, you can see the PX100 arm in rviz environment that is the sleep position. we develope two type of controllers for manipulate the arm inside it's addmisible workspace, the `PID` and `Adaptive` controllers. befor using the controllers, you must change the robot operating mode that's default mode is `position`. for using controllers you must change the operating mode to the `pwm` mode. for this purpose run the following command:

```bash 
$ rosservice call /px100/set_operating_modes "{cmd_type: 'group', name: 'arm', mode: 'pwm', profile_type: 'time', profile_velocity: 131, profile_acceleration: 25}" 
```
after properly change the operating mode, you are ready to use the `PID` or `Adaptive` controllers for controlling the PX100.

### PID Controller
open a new terminal and navigate it's where the repository is cloned. so change directory to the scripts folder by `cd scripts` command. now you can run the `main_pid.py` by gollowing command:

```bash
$ python3 main_pid.py
```
you see the robot start moving and go to the Home position with joints value $q = [0.0, 0.0, 0.0, 0.0]$ where all of the joints set in the zero. by typing the command `rostopic list` appeare the available topics. in the `/px100/commands/desired_joint_states` you can set the desired values for every joints of robot. laterly you can set the desirred joints position by following command:

```bash
$ rostopic pub -1 /px100/commands/desired_joint_states std_msgs/Float32MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [1.0, 0.5, 0.2, 0.1]" 

```
by using the above command, robot go the desired position : $q = [1.0, 0.5, 0.2, 0.1]$
