from RobotController import RobotController
import rospy
import numpy as np

def main():
    # RobotController object :
    robotController = RobotController()

    # initial estimation :
    theta_hat = np.ones((44, 1))

    # control loop rate:
    rate = rospy.Rate(100)
    
    # control loop :
    while not rospy.is_shutdown():    
        ctrlSignal, theta_hat = robotController.adaptive_controller(theta_hat)
        robotController.ctrl_cmd.name = "arm"
        robotController.ctrl_cmd.cmd = ctrlSignal
        robotController.ctrl_cmd_pub.publish(robotController.ctrl_cmd)
        rospy.loginfo("Control Signals : \n" + str(ctrlSignal) + "\n")
        rate.sleep()

if __name__ == '__main__':
    main()