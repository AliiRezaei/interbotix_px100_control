import sympy
import numpy as np
from robotdef import RobotDef
from geometry import Geometry
from regressor import regressor
import re

# PX100 Properties :
L1 = 0.08945                # links length
L2 = 0.1                    # ...
Lm = 0.035                  # ...
L3 = 0.1                    # ...
L4 = 0.08605                # ...
Lr = np.sqrt(L2**2 + Lm**2) # ...



m_base_link_frame = 0.395887
ixx=0.0010650000; iyy=0.0003332000; izz=0.0012080000; ixy=-0.0000130300; ixz=0.0000018614; iyz=0.0000409200
I_base_link_frame = np.matrix([[ixx, ixy, ixz],[ixy, iyy, iyz],[ixz, iyz, izz]])

m_shoulder_link = 0.072587
ixx=0.0000231000; iyy=0.0000253500; izz=0.0000144200; ixy=0.0000000003; ixz=0.0000001606; iyz=-0.0000000206
I_shoulder_link = np.matrix([[ixx, ixy, ixz],[ixy, iyy, iyz],[ixz, iyz, izz]])

m_upper_arm_link = 0.082923
ixx=0.0000800600; iyy=0.0000745900; izz=0.0000368500; ixy=-0.0000002144; ixz=0.0000002982; iyz=0.0000165700
I_upper_arm_link = np.matrix([[ixx, ixy, ixz],[ixy, iyy, iyz],[ixz, iyz, izz]])

m_forearm_link = 0.073058
ixx=0.0000533800; iyy=0.0000165300; izz=0.0000603500; ixy=-0.0000003073; ixz=0.0000000000; iyz=0.0000000000
I_forearm_link = np.matrix([[ixx, ixy, ixz],[ixy, iyy, iyz],[ixz, iyz, izz]])

m_gripper_link = 0.069929
ixx=0.0000226800; iyy=0.0000204400; izz=0.0000197400; ixy=0.0000000000; ixz=0.0000000000; iyz=0.0000008485
I_gripper_link = np.matrix([[ixx, ixy, ixz],[ixy, iyy, iyz],[ixz, iyz, izz]])

m_ee_arm_link = 0.001
ixx=0.001; iyy=0.001; izz=0.001; ixy=0; ixz=0; iyz=0
I_ee_arm_link = np.matrix([[ixx, ixy, ixz],[ixy, iyy, iyz],[ixz, iyz, izz]])

m_gripper_prop_link = 0.00434
ixx=0.0000005923; iyy=0.0000011156; izz=0.0000005743; ixy=0.0000000000; ixz=0.0000003195; iyz=-0.0000000004
I_gripper_prop_link = np.matrix([[ixx, ixy, ixz],[ixy, iyy, iyz],[ixz, iyz, izz]])

m_ar_tag_link = 0.016507
ixx=0.000003084; ixy=0; ixz=0; iyy=0.000003084; iyz=0; izz=0.000006059
I_ar_tag_link = np.matrix([[ixx, ixy, ixz],[ixy, iyy, iyz],[ixz, iyz, izz]])

m_gripper_bar_link = 0.034199
ixx=0.0000074125; iyy=0.0000284300; izz=0.0000286000; ixy=-0.0000000008; ixz=-0.0000000006; iyz=-0.0000013889
I_gripper_bar_link = np.matrix([[ixx, ixy, ixz],[ixy, iyy, iyz],[ixz, iyz, izz]])

m_fingers_link = 0.001
ixx=0.001; iyy=0.001; izz=0.001; ixy=0; ixz=0; iyz=0
I_fingers_link = np.matrix([[ixx, ixy, ixz],[ixy, iyy, iyz],[ixz, iyz, izz]])

m_left_finger_link = 0.016246
ixx=0.0000047310; iyy=0.0000015506; izz=0.0000037467; ixy=-0.0000004560; ixz=0.0000000000; iyz=0.0000000000
I_left_finger_link = np.matrix([[ixx, ixy, ixz],[ixy, iyy, iyz],[ixz, iyz, izz]])

m_right_finger_link = 0.016246
ixx=0.0000047310; iyy=0.0000015506; izz=0.0000037467; ixy=0.0000004560; ixz=0.0000000000; iyz=0.0000000000
I_right_finger_link = np.matrix([[ixx, ixy, ixz],[ixy, iyy, iyz],[ixz, iyz, izz]])

m_ee_gripper_link = 0.001
ixx=0.001; iyy=0.001; izz=0.001; ixy=0; ixz=0; iyz=0
I_ee_gripper_link = np.matrix([[ixx, ixy, ixz],[ixy, iyy, iyz],[ixz, iyz, izz]])

m_waist = m_base_link_frame + m_shoulder_link
I_waist = I_base_link_frame + I_shoulder_link

m_shoulder = m_shoulder_link + m_upper_arm_link
I_shoulder = I_shoulder_link + I_upper_arm_link

m_elbow = m_upper_arm_link + m_forearm_link
I_elbow = I_upper_arm_link + I_forearm_link

m_payload = m_ee_arm_link + m_gripper_prop_link + m_gripper_bar_link + m_fingers_link + m_left_finger_link + m_right_finger_link + m_ee_gripper_link
I_payload = I_ee_arm_link + I_gripper_prop_link + I_gripper_bar_link + I_fingers_link + I_left_finger_link + I_right_finger_link + I_ee_gripper_link

m_wrist_angle = m_forearm_link + m_gripper_link
I_wrist_angle = I_forearm_link + I_gripper_link

# m_wrist_angle = m_forearm_link + m_gripper_link + m_payload
# I_wrist_angle = I_forearm_link + I_gripper_link + I_payload



# Denavit-Hartenberg (alpha, a, d, theta)
dh = [('-pi/2',      0,    L1,    'q'),
      (    '0',     Lr,     0,    'q'),
      (    '0',     L3,     0,    'q'),
      (    '0',     L4,     0,    'q')]
  

# Robot Definition :
rbtdef = RobotDef('px100', dh, dh_convention = 'standard')
rbtdef.frictionmodel = {'Coulomb', 'viscous'} 
rbtdef.gravityacc = sympy.Matrix([0.0, 0.0, -9.81])


# Dynamics Properties of Robot :
rbtdef.m = np.array([m_waist, m_shoulder, m_elbow, m_wrist_angle]) # links mass
rbtdef.m = [I_waist, I_shoulder, I_elbow, I_wrist_angle] # links moment of inertia

# Geometry :
geom = Geometry(rbtdef, ifunc=None)

# Regressor :
reg = regressor(rbtdef, geom, ifunc=None)


text_reg = open(r'test_dynamics.txt', 'w')
str_reg = str(reg)
text_reg.truncate(0) # clear text file
text_reg.write(str_reg)
text_reg.close()


print(reg.shape)

file_path = 'test_dynamics.txt'

with open(file_path, 'r') as file:
    file_content = file.read()

file_content = file_content.replace('Matrix', 'Y = np.matrix')
file_content = file_content.replace('sin', 'np.sin')
file_content = file_content.replace('sign', 'np.sin')
file_content = file_content.replace('cos', 'np.cos')

file_content = file_content.replace('\ddot{q}_1', 'ddq[0]')
file_content = file_content.replace('\ddot{q}_2', 'ddq[1]')
file_content = file_content.replace('\ddot{q}_3', 'ddq[2]')
file_content = file_content.replace('\ddot{q}_4', 'ddq[3]')

file_content = file_content.replace('\dot{q}_1', 'dq[0]')
file_content = file_content.replace('\dot{q}_2', 'dq[1]')
file_content = file_content.replace('\dot{q}_3', 'dq[2]')
file_content = file_content.replace('\dot{q}_4', 'dq[3]')

file_content = file_content.replace('q1', 'q[0]')
file_content = file_content.replace('q2', 'q[1]')
file_content = file_content.replace('q3', 'q[2]')
file_content = file_content.replace('q4', 'q[3]')

with open(file_path, 'w') as file:
    file.write(file_content)

print('Regressor Generated Successfully!')