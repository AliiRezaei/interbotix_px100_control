import numpy as np
import sympy as sym
import re
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from interbotix_xs_msgs.msg import JointGroupCommand
import time

class RobotMotion:
    def __init__(self):

        # px100 links length :
        self.L1 = 0.08945
        self.L2 = 0.1
        self.Lm = 0.035 
        self.L3 = 0.1
        self.L4 = 0.08605
        self.Lr = np.sqrt(self.L2**2 + self.Lm**2)

        # init joints actual and desired angles :
        self.q = np.zeros((1, 4))[0]                # actual  angels
        # self.q_d = np.array([1.0, 0.0, -0.1, -0.2]) # desired angels
        self.q_d = np.array([0.0, 0.0, 0.0, 0.0]) # desired angels

        # init joints actual and desired velocities :
        self.dq = np.zeros((1, 4))[0]                # actual  velocities
        self.dq_d = np.array([0.0, 0.0, 0.0, 0.0]) # desired velocities

        # init tracking error vector :
        self.e = np.zeros((1, 4))[0]      # actual   error vector
        self.e_prev = np.zeros((1, 4))[0] # previous error vector

        # integral and derivative of error vector :
        self.ie = np.zeros((1, 4))[0]  # integral   error vector
        self.de = np.zeros((1, 4))[0]  # derivative error vector

        # declare control command type and its publisher :
        self.ctrl_cmd = JointGroupCommand()
        self.ctrl_cmd_pub = rospy.Publisher("px100/commands/joint_group", JointGroupCommand, queue_size=1)

        # init node :
        node_name = "motion_info_publisher"
        rospy.init_node(node_name, anonymous=True)

        # subscribe "px100/joint_states" for update actual joints angels :
        rospy.Subscriber("px100/joint_states", JointState, self.joint_states_callback)

        # subscribe "px100/desired_joint_states" for update desired joints angels :
        rospy.Subscriber("px100/desired_joint_states", Float32MultiArray, self.desired_joint_states_callback)

        # init now time and previous time (used in controller):
        self.t_now  = rospy.get_time() # time --> now
        self.t_prev = rospy.get_time() # time --> previous (last step)

    def joint_states_callback(self, msg):
        # update joints angle :
        self.q = np.array(msg.position[:4])
        # self.dq = np.array(msg.velocity[:4])

    def desired_joint_states_callback(self, msg):
        # update desired joints angle and :
        self.q_d = np.array(msg.data)
   

    def get_homogeneous_transformation(self):
        H_shoulder_to_waist = self.rotation_around_z(self.q[0]) @ self.translation_about_z(self.L1) @ self.translation_about_x(0) @ self.rotation_around_x(-np.pi/2)

        H_elbow_to_shoulder = self.rotation_around_z(self.q[1]) @ self.translation_about_z(0) @ self.translation_about_x(self.Lr) @ self.rotation_around_x(0)

        H_wrist_to_elbow = self.rotation_around_z(self.q[2]) @ self.translation_about_z(0) @ self.translation_about_x(self.L3) @ self.rotation_around_x(0)

        H_gripper_to_wrist = self.rotation_around_z(self.q[3]) @ self.translation_about_z(0) @ self.translation_about_x(self.L4) @ self.rotation_around_x(0)

        return H_shoulder_to_waist, H_elbow_to_shoulder, H_wrist_to_elbow, H_gripper_to_wrist
    

    # Rotation Matrices :
    def rotation_around_x(self, theta, out_type = 'np'):
        if out_type == 'np':
            return np.matrix([[1,         0,                0,              0],
                              [0,   np.cos(theta),   -np.sin(theta),        0],
                              [0,   np.sin(theta),    np.cos(theta),        0],
                              [0,         0,                0,              1]])
        elif out_type == 'sym':
            return sym.Matrix([[1,          0,                0,               0],
                               [0,   sym.cos(theta),   -sym.sin(theta),        0],
                               [0,   sym.sin(theta),    sym.cos(theta),        0],
                               [0,          0,                0,               1]])
    
    def rotation_around_y(self, theta, out_type = 'np'):
        if out_type == 'np':
            return np.matrix([[np.cos(theta),    0,   np.sin(theta),   0],
                              [       0,         1,          0,        0],
                              [-np.sin(theta),   0,   np.cos(theta),   0],
                              [       0,         0,          0,        1]])
        elif out_type == 'sym':
            return sym.Matrix([[sym.cos(theta),    0,   sym.sin(theta),   0],
                               [       0,          1,          0,         0],
                               [-sym.sin(theta),   0,   sym.cos(theta),   0],
                               [       0,          0,          0,         1]])

    def rotation_around_z(self, theta, out_type = 'np'):
        if out_type == 'np':
            return np.matrix([[np.cos(theta),   -np.sin(theta),     0,    0],
                              [np.sin(theta),    np.cos(theta),     0,    0],
                              [      0,               0,            1,    0],
                              [      0,               0,            0,    1]])
        elif out_type == 'sym':
            return sym.Matrix([[sym.cos(theta),   -sym.sin(theta),     0,    0],
                               [sym.sin(theta),    sym.cos(theta),     0,    0],
                               [      0,                0,             1,    0],
                               [      0,                0,             0,    1]])
    

    # Translation Matrices :
    def translation_about_x(self, x, out_type = 'np'):
        if out_type == 'np':
            return np.matrix([[1,   0,   0,   x],
                              [0,   1,   0,   0],
                              [0,   0,   1,   0],
                              [0,   0,   0,   1]])
        elif out_type == 'sym':
            return sym.Matrix([[1,   0,   0,   x],
                               [0,   1,   0,   0],
                               [0,   0,   1,   0],
                               [0,   0,   0,   1]])
        else :
            print('output type error (use np or sym)')
    
    def translation_about_y(self, y, out_type = 'np'):
        if out_type == 'np':
            return np.matrix([[1,   0,   0,   0],
                              [0,   1,   0,   y],
                              [0,   0,   1,   0],
                              [0,   0,   0,   1]])
        elif out_type == 'sym':
            return sym.Matrix([[1,   0,   0,   0],
                               [0,   1,   0,   y],
                               [0,   0,   1,   0],
                               [0,   0,   0,   1]])
    
    def translation_about_z(self, z, out_type = 'np'):
        if out_type == 'np':
            return np.matrix([[1,   0,   0,   0],
                              [0,   1,   0,   0],
                              [0,   0,   1,   z],
                              [0,   0,   0,   1]])
        elif out_type == 'sym':
            return sym.Matrix([[1,   0,   0,   0],
                               [0,   1,   0,   0],
                               [0,   0,   1,   z],
                               [0,   0,   0,   1]])
    
    
    def pid_controller(self):
        # PID gains :
        kp = 100
        ki = 10
        kd = 20

        # update now time :
        self.t_now = rospy.get_time()

        # calc time step :
        dt = self.t_now - self.t_prev

        # calc PID terms :
        self.e = self.q_d - self.q
        self.ie = (self.e + self.ie) * dt
        self.de = (self.e - self.e_prev) / dt

        # assigning now vars to the prev vars :
        self.t_prev = self.t_now
        self.e_prev = self.e
        
        u = kp * self.e + ki * self.ie + kd * self.de

        # maxU = 10
        # for i in range(0, len(u)):
        #     if abs(u[i]) > maxU:
        #         u[i] = np.sign(u[i]) * maxU
        
        return u
    
    def Regressor(self, q, dq, ddq):
        # Read the content of the text file
        with open('rne_dynamics/test_dynamics.txt', 'r') as file:
            content = file.read()

        # Extract the matrix expression from the content using regex
        matrix_expression = re.search(r'Y = np\.matrix\((.+)\)', content).group(1)


        # Define the vector q dq ddq
        q = q
        dq = dq
        ddq = ddq


        # Evaluate the matrix expression to get the numpy matrix
        Y = np.matrix(eval(matrix_expression))
        # print(Y)
        return Y

    

    def adaptive_controller(self, theta_hat):
        q_tilde  = self.q  - self.q_d
        dq_tilde = self.dq - self.dq_d
        ddq = np.zeros((1, 4))[0]
        Lambda = 20.0 * np.eye(4)
        Kd     = 50.0 * np.eye(4)
        Gamma  = 0.1 * np.eye(48)

        # theta_hat = np.transpose(theta_hat)

        # s = dq_tilde + Lambda @ q_tilde
        s = dq_tilde + Lambda @ q_tilde
        Y = self.Regressor(self.q, self.dq, ddq)
        
        
        dtheta_hat = - Gamma * np.transpose(Y) @ s
        # print(dtheta_hat)
        # update now time :
        self.t_now = rospy.get_time()

        # calc time step :
        dt = self.t_now - self.t_prev

        theta_hat = dtheta_hat * dt + theta_hat
        # print(theta_hat)
        
        # assigning now vars to the prev vars :
        self.t_prev = self.t_now
    
        u = Y @ np.transpose(theta_hat) - np.transpose(Kd @ s)[0]
        # print(Y @ np.transpose(theta_hat) - np.transpose(Kd @ s)[0])
        # print(theta_hat.shape)
        
        return u, theta_hat

    # def get_body_jacobian(self):

    # def get_com_jacobian(self):

    # def get_mass_matrix(self):

    # def get_coriolis_accel(self):

    # def get_gravity_vector(self):

class RobotDynamics:
    def __init__(self, robotMotion):
        
        self.robotMotion = robotMotion
        
        # robot symbolic params (joints angle and velocity) :
        # self.q1, self.q2, self.q3, self.q4 = sym.symbols('q1, q2, q3, q4', real=True)
        # self.dq1, self.dq2, self.dq3, self.dq4 = sym.symbols('dq1, dq2, dq3, dq4', real=True)

        # links center of mass : (default)
        self.Lc1 = self.robotMotion.L1 / 2
        self.Lc2 = self.robotMotion.L2 / 2
        self.Lc3 = self.robotMotion.L3 / 2
        self.Lc4 = self.robotMotion.L4 / 2



    def get_com_jacobian(self, no_com):
        # no_com --> witch one of links? (number of link) 1, 2, 3 or 4
        if no_com == 1:
            H_com_1 = self.robotMotion.rotation_around_z(self.q1, 'sym') @ self.robotMotion.translation_about_z(self.Lc1, 'sym') @ self.robotMotion.translation_about_x(0, 'sym') @ self.robotMotion.rotation_around_x(-np.pi/2, 'sym')
            H = sym.simplify(H_com_1)
            R = H[:3, :3]
            z_i = R[:, -1]
        elif no_com == 2:
            H_1     = self.robotMotion.rotation_around_z(self.q1, 'sym') @ self.robotMotion.translation_about_z(self.robotMotion.L1, 'sym') @ self.robotMotion.translation_about_x(0, 'sym') @ self.robotMotion.rotation_around_x(-np.pi/2, 'sym')
            H_com_2 = self.robotMotion.rotation_around_z(self.q2, 'sym') @ self.robotMotion.translation_about_z(0, 'sym') @ self.robotMotion.translation_about_x(self.Lc2, 'sym') @ self.robotMotion.rotation_around_x(0, 'sym')
            H = sym.simplify(H_1 @ H_com_2)
            R = H[:3, :3]
            z_i = R[:, -1]
        elif no_com == 3:
            H_1     = self.robotMotion.rotation_around_z(self.q1, 'sym') @ self.robotMotion.translation_about_z(self.robotMotion.L1, 'sym') @ self.robotMotion.translation_about_x(0, 'sym') @ self.robotMotion.rotation_around_x(-np.pi/2, 'sym')
            H_2     = self.robotMotion.rotation_around_z(self.q2, 'sym') @ self.robotMotion.translation_about_z(0, 'sym') @ self.robotMotion.translation_about_x(self.robotMotion.L2, 'sym') @ self.robotMotion.rotation_around_x(0, 'sym')
            H_com_3 = self.robotMotion.rotation_around_z(self.q3, 'sym') @ self.robotMotion.translation_about_z(0, 'sym') @ self.robotMotion.translation_about_x(self.Lc3, 'sym') @ self.robotMotion.rotation_around_x(0, 'sym')
            H = sym.simplify(H_1 @ H_2 @ H_com_3)
            R = H[:3, :3]
            z_i = R[:, -1]
        elif no_com == 4:
            H_1     = self.robotMotion.rotation_around_z(self.q1, 'sym') @ self.robotMotion.translation_about_z(self.robotMotion.L1, 'sym') @ self.robotMotion.translation_about_x(0, 'sym') @ self.robotMotion.rotation_around_x(-np.pi/2, 'sym')
            H_2     = self.robotMotion.rotation_around_z(self.q2, 'sym') @ self.robotMotion.translation_about_z(0, 'sym') @ self.robotMotion.translation_about_x(self.robotMotion.L2, 'sym') @ self.robotMotion.rotation_around_x(0, 'sym')
            H_3     = self.robotMotion.rotation_around_z(self.q3, 'sym') @ self.robotMotion.translation_about_z(0, 'sym') @ self.robotMotion.translation_about_x(self.robotMotion.L3, 'sym') @ self.robotMotion.rotation_around_x(0, 'sym')
            H_com_4 = self.robotMotion.rotation_around_z(self.q4, 'sym') @ self.robotMotion.translation_about_z(0, 'sym') @ self.robotMotion.translation_about_x(self.Lc4, 'sym') @ self.robotMotion.rotation_around_x(0, 'sym')
            H = sym.simplify(H_1 @ H_2 @ H_3 @ H_com_4)
            R = H[:3, :3]
            z_i = R[:, -1]
        else :
            print('Something Wrong!')

        fq = H[:3, -1]
        Jv = fq.jacobian([self.q1, self.q2, self.q3, self.q4])
        print(Jv)
        return H

    def get_body_jacobian(self):
        H_1 = self.robotMotion.rotation_around_z(self.q1, 'sym') @ self.robotMotion.translation_about_z(self.robotMotion.L1, 'sym') @ self.robotMotion.translation_about_x(0, 'sym') @ self.robotMotion.rotation_around_x(-np.pi/2, 'sym')
        H_2 = self.robotMotion.rotation_around_z(self.q2, 'sym') @ self.robotMotion.translation_about_z(0, 'sym') @ self.robotMotion.translation_about_x(self.robotMotion.L2, 'sym') @ self.robotMotion.rotation_around_x(0, 'sym')
        H_3 = self.robotMotion.rotation_around_z(self.q3, 'sym') @ self.robotMotion.translation_about_z(0, 'sym') @ self.robotMotion.translation_about_x(self.robotMotion.L3, 'sym') @ self.robotMotion.rotation_around_x(0, 'sym')
        H_4 = self.robotMotion.rotation_around_z(self.q4, 'sym') @ self.robotMotion.translation_about_z(0, 'sym') @ self.robotMotion.translation_about_x(self.robotMotion.L4, 'sym') @ self.robotMotion.rotation_around_x(0, 'sym')
        H   = sym.simplify(H_1 @ H_2 @ H_3 @ H_4)
        return H


    
def main():
    robot = RobotMotion()
    # robotDynamics = RobotDynamics(robot)
    # robotDynamics.get_com_jacobian(4)
    # tmp = sym.Matrix(robotDynamics.robotMotion.translation_about_z(robotDynamics.Lc1))
    # print(tmp@tmp)
    # tmp = robot.translation_about_x(10, 'sym')
    # print(tmp)


    theta_hat = np.transpose(np.ones((1, 48))[0])

    start_time = rospy.get_time()

    while not rospy.is_shutdown():
        # H_shoulder_to_waist, H_elbow_to_shoulder, H_wrist_to_elbow, H_gripper_to_wrist = robot.get_homogeneous_transformation()

        # H_gripper_to_waist = H_shoulder_to_waist @ H_elbow_to_shoulder @ H_wrist_to_elbow @ H_gripper_to_wrist

        # print("\n Homogen Matrix from gripper to waist is : \n", H_gripper_to_waist)
        
        if rospy.get_time() - start_time < 10.0:
            u = robot.pid_controller()
        # print(rospy.Time.now())
        else:
            u, theta_hat = robot.adaptive_controller(theta_hat)
        robot.ctrl_cmd.name = "arm"
        robot.ctrl_cmd.cmd = u
        robot.ctrl_cmd_pub.publish(robot.ctrl_cmd)
        # rospy.loginfo("Tracking error : \n" + str(robot.e) + "\n")
        rospy.loginfo("Control Signals : \n" + str(u) + "\n")
        # rate.sleep()

if __name__ == '__main__':
    main()