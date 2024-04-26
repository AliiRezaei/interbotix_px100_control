import sympy
import numpy as np
from robotdef import RobotDef
from geometry import Geometry
from regressor import regressor
import re

# px100 links length :
L1 = 0.08945
L2 = 0.1
Lm = 0.035 
L3 = 0.1
L4 = 0.08605
Lr = np.sqrt(L2**2 + Lm**2)


# (alpha, a, d, theta)
dh = [('-pi/2',      0,    L1,    'q'),
      (    '0',     Lr,     0,    'q'),
      (    '0',     L3,     0,    'q'),
      (    '0',     L4,     0,    'q')]
  


# dh = [('-pi/2',      0,    L1,    'q'),
#       (    '0',     L2,     0,    '0'),
#       (    '0',     Lm,     0,    'q'),
#       (    '0',     L3,     0,    'q'),
#       (    '0',     L4,     0,    'q')]

rbtdef = RobotDef('px100', dh, dh_convention = 'standard')

rbtdef.frictionmodel = {'Coulomb', 'viscous'} # options are None or a combination of 'Coulomb', 'viscous' and 'offset'
rbtdef.gravityacc = sympy.Matrix([0.0, 0.0, -9.81]) # optional, this is the default value

geom = Geometry(rbtdef, ifunc=None)
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

print("Text replaced successfully!")




# # Read the content of the text file
# with open(file_path, 'r') as file:
#     content = file.read()

# # Extract the matrix expression from the content using regex
# matrix_expression = re.search(r'Y = np\.matrix\((.+)\)', content).group(1)


# # Define the vector q dq ddq
# q = np.array([1, 2, 3, 4])
# dq = np.array([1, 2, 3, 4])
# ddq = np.array([1, 2, 3, 4])


# # Evaluate the matrix expression to get the numpy matrix
# Y = eval(matrix_expression)
# print(Y)