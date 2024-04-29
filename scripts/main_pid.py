from RobotController import RobotController
import rospy

def main():
    # RobotController object :
    robotController = RobotController()

    # control loop rate:
    rate = rospy.Rate(100)
    
    # control loop :
    while not rospy.is_shutdown():    
        ctrlSignal = robotController.pid_controller()
        robotController.ctrl_cmd.name = "arm"
        robotController.ctrl_cmd.cmd = ctrlSignal
        robotController.ctrl_cmd_pub.publish(robotController.ctrl_cmd)
        rospy.loginfo("Control Signals : \n" + str(ctrlSignal) + "\n")
        rate.sleep()

if __name__ == '__main__':
    main()