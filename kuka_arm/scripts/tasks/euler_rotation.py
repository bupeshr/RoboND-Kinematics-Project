#!/usr/bin/env python
import numpy as np
from sympy.matrices import Matrix
from sympy import symbols, atan2, sqrt


# Fixed Axis X-Y-Z Rotation Matrix
R_XYZ = Matrix([[ 0.353553390593274, -0.306186217847897, 0.883883476483184],
            [ 0.353553390593274,  0.918558653543692, 0.176776695296637],
            [-0.866025403784439,               0.25, 0.433012701892219]])

######## TO DO ##########
# Calculate the Euler angles that produces a rotation equivalent to R (above)

r31 = R_XYZ[2,0]
r21 = R_XYZ[1,0]
r11 = R_XYZ[0,0]

r32 = R_XYZ[2,1]
r22 = R_XYZ[1,1]
r12 = R_XYZ[0,1]

r33 = R_XYZ[2,2]
r23 = R_XYZ[1,2]
r13 = R_XYZ[0,2]

# NOTE: Be sure your answer has units of DEGREES!
deg = np.pi/180
beta = atan2(-r31, sqrt(r21**2 + r11**2))*deg
alpha  = atan2(r21,r11)*deg # rotation about Y-axis
gamma = atan2(r31,r33)*deg # rotation about X-axis

print('--------------------------')
print(alpha)
print('--------------------------')
print(beta)
print('--------------------------')
print(gamma)
print('--------------------------')
