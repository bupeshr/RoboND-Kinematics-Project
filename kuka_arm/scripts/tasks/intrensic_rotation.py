from sympy import symbols, cos, sin, pi, sqrt
from sympy.matrices import Matrix as m

### Create symbols for joint variables
q1, q2 = symbols('q1:3')

# Create a symbolic matrix representing an intrinsic sequence of rotations 
  # about the Y and then Z axes. Let the rotation about the Y axis be described
  # by q1 and the rotation about Z by q2. 

R_y = m([[ cos(q1), 0, sin(q1)],
        [       0, 1,       0],
        [-sin(q1), 0, cos(q1)]])

R_z = m([[ cos(q2), -sin(q2), 0],
        [ sin(q2),  cos(q2), 0],
        [       0,        0, 1]])
####### TO DO ########
# Replace R_y and R_z with the appropriate (symbolic) elementary rotation matrices 
  # and then compute YZ_intrinsic. 
# R_y = 1
# R_z = 1
YZ_intrinsic_sym = R_y * R_z

####### TO DO ########
# Numerically evaluate YZ_intrinsic assuming:
   # q1 = 45 degrees and q2 = 60 degrees. 
   # NOTE: Trigonometric functions in Python assume the input is in radians!  

rad = 180/pi #deg to rad
deg = pi/180 #rad to deg

YZ_intrinsic_num = YZ_intrinsic_sym.evalf(subs={q1: 45*deg, q2: 60*deg})
print(YZ_intrinsic_num)