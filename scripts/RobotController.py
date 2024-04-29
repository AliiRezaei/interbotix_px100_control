import numpy as np
import re
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from interbotix_xs_msgs.msg import JointGroupCommand

class RobotController:
    def __init__(self):
        # init joints actual and desired angles :
        self.q = np.zeros((4, 1))      # actual  angels
        self.q_d = np.zeros((4, 1))    # desired angels
        self.q_prev = np.zeros((4, 1)) # previous joints angle

        # init joints actual and desired velocities :
        self.dq = np.zeros((4, 1))     # actual  velocities
        self.dq_d = np.zeros((4, 1))   # desired velocities

        # init tracking error vector :
        self.e = np.zeros((4, 1))      # actual   error vector
        self.e_prev = np.zeros((4, 1)) # previous error vector

        # integral and derivative of error vector :
        self.ie = np.zeros((4, 1))     # integral   error vector
        self.de = np.zeros((4, 1))     # derivative erro4 v1ct
        
        # declare control command type and It's publisher :
        self.ctrl_cmd = JointGroupCommand()
        self.ctrl_cmd_pub = rospy.Publisher("px100/commands/joint_group", JointGroupCommand, queue_size=1)

        # init node :
        node_name = "px100_control_node"
        rospy.init_node(node_name, anonymous=True)

        # subscribe "px100/joint_states" for update actual joints angels :
        rospy.Subscriber("px100/joint_states", JointState, self.joint_states_callback)

        # subscribe "px100/desired_joint_states" for update desired joints angels :
        rospy.Subscriber("px100/commands/desired_joint_states", Float32MultiArray, self.desired_joint_states_callback)

        # init current time and previous time (used in controller):
        self.t_now  = rospy.get_time() # time --> current
        self.t_prev = rospy.get_time() # time --> previous (last step)


    def joint_states_callback(self, msg):
        # update joints angle :
        self.q[:, 0] = np.array(msg.position[:4])
        # self.dq = np.array(msg.velocity[:4])

    def desired_joint_states_callback(self, msg):
        # update desired joints angle and :
        self.q_d[:, 0] = np.array(msg.data)


    def pid_controller(self, kp=100, ki=10, kd=20):
        # default PID gains :
        # kp = 100, ki = 10, kd = 20

        # update current time :
        self.t_now = rospy.get_time()

        # calc time step :
        dt = self.t_now - self.t_prev

        # calc PID terms :
        self.e = self.q_d - self.q
        self.ie = (self.e + self.ie) * dt
        self.de = (self.e - self.e_prev) / dt

        # assigning current vars to the prev vars :
        self.t_prev = self.t_now
        self.e_prev = self.e
        
        # control law :
        u = kp * self.e + ki * self.ie + kd * self.de

        # applying control signal constraints : 
        # assumption : - maxU <= u <= + maxU
        maxU = 50
        for i in range(0, len(u)):
            if abs(u[i]) > maxU:
                u[i] = np.sign(u[i]) * maxU
        
        return u
    
    def Regressor(self, q, dq, ddq):
        # read the content of the text file (where regressor matrix saved):
        with open('regressor_matrix.txt', 'r') as file:
            content = file.read()

        # extract the matrix expression from the content using re :
        matrix_expression = re.search(r'Y = np\.matrix\((.+)\)', content).group(1)

        # define the vector q dq ddq
        q   = q
        dq  = dq
        ddq = ddq

        # evaluate the matrix expression to get the numpy matrix
        Y = np.matrix(eval(matrix_expression))
        
        return Y
    

    def adaptive_controller(self, theta_hat):
        # joints acceleration :
        ddq = np.zeros((4, 1))

        # controller params :
        Lambda = 30.0 * np.eye(4)
        Kd     = 15.0 * np.eye(4)
        Gamma  = 0.05 * np.eye(44)

        # extract robot states :
        q = self.q
        dq = self.dq

        # get current time :
        self.t_now = rospy.get_time()

        # calc time step :
        dt = self.t_now - self.t_prev

        # joints velocity :
        self.dq = (q  - self.q_prev) / dt
        
        # pos and vel errors :
        q_tilde  = q  - self.q_d
        dq_tilde = dq - self.dq_d

        # calc regressor true val :
        s = dq_tilde + Lambda @ q_tilde
        Y = self.Regressor(q, dq, ddq)
        
        # adaptation law :
        dtheta_hat = - Gamma @ np.transpose(Y) @ s
        
        # calc numerical integral of adaptation law :
        theta_hat = dtheta_hat * dt + theta_hat 
        
        # assigning current vars to the prev vars :
        self.t_prev = self.t_now
        self.q_prev = q
    
        # control law :
        u = Y @ theta_hat - Kd @ s

        # applying control signal constraints : 
        # assumption : - maxU <= u <= + maxU
        maxU = 50
        for i in range(0, len(u)):
            if abs(u[i]) > maxU:
                u[i] = np.sign(u[i]) * maxU
        
        return u, theta_hat
