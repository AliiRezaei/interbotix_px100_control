import numpy as np
from math import cos, sin, sqrt, inf
import rospy
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointGroupCommand


class RobotMotion:
    def __init__(self):

        self.L1 = 0.08945
        self.L2 = 0.1
        self.Lm = 0.035 
        self.L3 = 0.1
        self.L4 = 0.08605
        self.Lr = sqrt(self.L2**2 + self.Lm**2)

        self.q = np.zeros([7, 1])

        self.e = np.array([[0.0],
                           [0.0],
                           [0.0],
                           [0.0]])
        self.ie = np.array([[0.0],
                           [0.0],
                           [0.0],
                           [0.0]])

        self.joint_command = JointGroupCommand()
        self.joint_command.name = "arm"
        self.joint_command_pub = rospy.Publisher("px100/commands/joint_group", JointGroupCommand, queue_size=10)


        node_name = "motion_info_publisher"
        rospy.init_node(node_name, anonymous=True)

        topic_name = "px100/joint_states"
        rospy.Subscriber(topic_name, JointState, self.joint_state_callback)

    def joint_state_callback(self, msg):
        self.q = msg.position
   

    def get_homogeneous_transformation(self):
        H_shoulder_to_waist = self.rotation_around_z(self.q[0]) @ self.translation_around_z(self.L1) @ self.translation_around_x(0) @ self.rotation_around_x(-np.pi/2)

        H_elbow_to_shoulder = self.rotation_around_z(self.q[1]) @ self.translation_around_z(0) @ self.translation_around_x(self.Lr) @ self.rotation_around_x(0)

        H_wrist_to_elbow = self.rotation_around_z(self.q[2]) @ self.translation_around_z(0) @ self.translation_around_x(self.L3) @ self.rotation_around_x(0)

        H_gripper_to_wrist = self.rotation_around_z(self.q[3]) @ self.translation_around_z(0) @ self.translation_around_x(self.L4) @ self.rotation_around_x(0)

        return H_shoulder_to_waist, H_elbow_to_shoulder, H_wrist_to_elbow, H_gripper_to_wrist
    

    # Rotation Matrices :
    def rotation_around_x(self, theta):
        return np.matrix([[1,        0,            0,             0],
                          [0,   cos(theta),   -sin(theta),        0],
                          [0,   sin(theta),    cos(theta),        0],
                          [0,        0,            0,             1]])
    
    def rotation_around_y(self, theta):
        return np.matrix([[cos(theta),    0,   sin(theta),   0],
                          [     0,        1,         0,      0],
                          [-sin(theta),   0,   cos(theta),   0],
                          [     0,        0,         0,      1]])
    
    def rotation_around_z(self, theta):
        return np.matrix([[cos(theta),   -sin(theta),     0,    0],
                          [sin(theta),    cos(theta),     0,    0],
                          [     0,             0,         1,    0],
                          [     0,             0,         0,    1]])
    

    # Translation Matrices :
    def translation_around_x(self, x):
        return np.matrix([[1,   0,   0,   x],
                          [0,   1,   0,   0],
                          [0,   0,   1,   0],
                          [0,   0,   0,   1]])
    
    def translation_around_y(self, y):
        return np.matrix([[1,   0,   0,   0],
                          [0,   1,   0,   y],
                          [0,   0,   1,   0],
                          [0,   0,   0,   1]])
    
    def translation_around_z(self, z):
        return np.matrix([[1,   0,   0,   0],
                          [0,   1,   0,   0],
                          [0,   0,   1,   z],
                          [0,   0,   0,   1]])
    
    def pi_controller(self):
        P = 100.0 * np.eye(4, 4)
        I = 10.0 * np.eye(4, 4)

        q_d = np.array([0.0, 0.0, 0.0, 0.0])
        q_d = np.transpose(q_d)

        self.e = np.array(q_d - self.q[:4], dtype = float)
        self.ie = np.array(self.ie + self.e)
        # print(self.e)
        # print(self.ie)

        u = list(np.array(P @ self.e, dtype=float))
        return u
    
    # def get_body_jacobian(self):

    # def get_com_jacobian(self):

    # def get_mass_matrix(self):

    # def get_coriolis_accel(self):

    # def get_gravity_vector(self):


    
def main():
    robot = RobotMotion()
    while not rospy.is_shutdown():
        # H_shoulder_to_waist, H_elbow_to_shoulder, H_wrist_to_elbow, H_gripper_to_wrist = robot.get_homogeneous_transformation()

        # H_gripper_to_waist = H_shoulder_to_waist @ H_elbow_to_shoulder @ H_wrist_to_elbow @ H_gripper_to_wrist

        # print("\n Homogen Matrix from gripper to waist is : \n", H_gripper_to_waist)
        
        # rospy.spin()

        u = robot.pi_controller()
        # print(u)
        # print(type(u[0]))
        robot.joint_command.name = "arm"
        robot.joint_command.cmd = [u[0], u[1], u[2], u[3]]
        # robot.joint_command.cmd = [0.0, 0.0, 0.0, 0.0]
        robot.joint_command_pub.publish(robot.joint_command)


if __name__ == '__main__':
    main()