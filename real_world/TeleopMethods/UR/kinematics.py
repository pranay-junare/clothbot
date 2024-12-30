"""
This forward computation is based on the base_link
the result is the end-effector's pose, not the wrist_3_link
"""
import numpy as np

from math import cos as cos
from math import sin as sin
from math import sqrt as sqrt
import tf.transformations as tf
global mat

# UR5e dh parameters
# This can be find here:
# https://www.universal-robots.com/how-tos-and-faqs/faq/ur-faq/parameters-for-calculations-of-kinematics-and-dynamics/
# or in the ur_e_description package
d1 =  0.163
a2 = -0.425
a3 = -0.392
d4 =  0.127
d5 =  0.1
d6 =  0.1


def forward(q):
    s = [sin(q[0]), sin(q[1]), sin(q[2]), sin(q[3]), sin(q[4]), sin(q[5])]
    c = [cos(q[0]), cos(q[1]), cos(q[2]), cos(q[3]), cos(q[4]), cos(q[5])]

    q23 = q[1]+q[2]
    q234 = q[1]+q[2]+q[3]

    s23 = sin(q23)
    c23 = cos(q23)
    s234 = sin(q234)
    c234 = cos(q234)
    T = np.matrix(np.identity(4))
    T[0, 0] = c234*c[0]*s[4] - c[4]*s[0]
    T[0, 1] = c[5]*(s[0]*s[4] + c234*c[0]*c[4]) - s234*c[0]*s[5]
    T[0, 2] = -s[5]*(s[0]*s[4] + c234*c[0]*c[4]) - s234*c[0]*c[5]
    T[0, 3] = d6*c234*c[0]*s[4] - a3*c23*c[0] - a2*c[0]*c[1] - d6*c[4]*s[0] - d5*s234*c[0] - d4*s[0]
    T[1, 0] = c[0]*c[4] + c234*s[0]*s[4]
    T[1, 1] = -c[5]*(c[0]*s[4] - c234*c[4]*s[0]) - s234*s[0]*s[5]
    T[1, 2] = s[5]*(c[0]*s[4] - c234*c[4]*s[0]) - s234*c[5]*s[0]
    T[1, 3] = d6*(c[0]*c[4] + c234*s[0]*s[4]) + d4*c[0] - a3*c23*s[0] - a2*c[1]*s[0] - d5*s234*s[0]
    T[2, 0] = -s234*s[4]
    T[2, 1] = -c234*s[5] - s234*c[4]*c[5]
    T[2, 2] = s234*c[4]*s[5] - c234*c[5]
    T[2, 3] = d1 + a3*s23 + a2*s[1] - d5*(c23*c[3] - s23*s[3]) - d6*s[4]*(c23*s[3] + s23*c[3])
    T[3, 0] = 0.0
    T[3, 1] = 0.0
    T[3, 2] = 0.0
    T[3, 3] = 1.0
    return T

def forward_6(q):
    mat = forward(q)
    xyz = mat[:3, 3].A1
    rpy = tf.euler_from_matrix(mat, 'sxyz')
    return xyz, rpy