import sympy
import numpy as np
from RobotDefinition import RobotDef
from RobotGeometry import Geometry
from RobotDynamics import RicursiveNewtonEuler as RNE

# PX100 Properties :
L1 = 0.08945                # links length
L2 = 0.1                    # ...
Lm = 0.035                  # ...
L3 = 0.1                    # ...
L4 = 0.08605                # ...
Lr = np.sqrt(L2**2 + Lm**2) # ...



m_base_link_frame = 0.395887
xyz_base_link_frame = [-0.0332053000, 0.0008915770, 0.0211913000]
fc_base_link_frame = 0.1
ixx=0.0010650000; iyy=0.0003332000; izz=0.0012080000; ixy=-0.0000130300; ixz=0.0000018614; iyz=0.0000409200
I_base_link_frame = [ixx, ixy, ixz, iyy, iyz, izz]

m_shoulder_link = 0.072587
xyz_shoulder_link = [0.0000111169, -0.0003605640, 0.0284598000]
fc_shoulder_link = 0.1
ixx=0.0000231000; iyy=0.0000253500; izz=0.0000144200; ixy=0.0000000003; ixz=0.0000001606; iyz=-0.0000000206
I_shoulder_link = [ixx, ixy, ixz, iyy, iyz, izz]

m_upper_arm_link = 0.082923
xyz_upper_arm_link = [0.0161976963, -0.0002929352, 0.0877230000]
fc_upper_arm_link = 0.1
ixx=0.0000800600; iyy=0.0000745900; izz=0.0000368500; ixy=-0.0000002144; ixz=0.0000002982; iyz=0.0000165700
I_upper_arm_link = [ixx, ixy, ixz, iyy, iyz, izz]

m_forearm_link = 0.073058
xyz_forearm_link = [0.0773720000, -0.0003324882, 0.0000000000]
fc_forearm_link = 0.1
ixx=0.0000533800; iyy=0.0000165300; izz=0.0000603500; ixy=-0.0000003073; ixz=0.0000000000; iyz=0.0000000000
I_forearm_link = [ixx, ixy, ixz, iyy, iyz, izz]

m_gripper_link = 0.069929
xyz_gripper_link = [0.0446910000, 0.0000000000, 0.0113540000]
fc_gripper_link = 0.1
ixx=0.0000226800; iyy=0.0000204400; izz=0.0000197400; ixy=0.0000000000; ixz=0.0000000000; iyz=0.0000008485
I_gripper_link = [ixx, ixy, ixz, iyy, iyz, izz]


# Denavit-Hartenberg (alpha, a, d, theta)
dh = [('-pi/2',      0,    L1,    'q'),
      (    '0',     Lr,     0,    'q'),
      (    '0',     L3,     0,    'q'),
      (    '0',     L4,     0,    'q')]
  

# Robot Definition :
rbtdef = RobotDef('px100', dh, dh_convention = 'standard')
rbtdef.frictionmodel = {'Coulomb'} 
rbtdef.gravityacc = sympy.Matrix([0.0, 0.0, -9.81])


# Dynamics Properties of Robot :
rbtdef.m = [m_base_link_frame, m_shoulder_link, m_upper_arm_link, m_forearm_link] # links mass

rbtdef.Le = [I_base_link_frame, I_shoulder_link, I_upper_arm_link, I_forearm_link]

rbtdef.l = [xyz_base_link_frame, xyz_shoulder_link, xyz_upper_arm_link, xyz_forearm_link]

rbtdef.fc = [fc_base_link_frame, fc_shoulder_link, fc_upper_arm_link, fc_forearm_link]



# Geometry :
geom = Geometry(rbtdef, ifunc=None)

# Regressor :
rne = RNE()
reg = rne.regressor(rbtdef, geom, ifunc=None)

# Save reg in regressor_matrix.txt :
text_reg = open(r'regressor_matrix.txt', 'w')
text_reg.truncate(0) # clear text file
text_reg.write(str(reg))
text_reg.close()


with open('regressor_matrix.txt', 'r') as file:
    file_content = file.read()

file_content = file_content.replace('Matrix', 'Y = np.matrix')
file_content = file_content.replace('sin'   , 'np.sin')
file_content = file_content.replace('sign'  , 'np.sign')
file_content = file_content.replace('cos'   , 'np.cos')


with open('regressor_matrix.txt', 'w') as file:
    file.write(file_content)

print('Regressor Generated Successfully!')