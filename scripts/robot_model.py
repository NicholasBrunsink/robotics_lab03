import math as m
import numpy as np

def dh_transformation(theta, a, d, alpha):
    return np.array(
        [[m.cos(theta), -m.sin(theta)*m.cos(alpha),   m.sin(theta)*m.sin(alpha),   a*m.cos(theta)],
         [m.sin(theta),  m.cos(theta)*m.cos(alpha),  -m.cos(theta)*m.sin(alpha),   a*m.sin(theta)],
         [0,             m.sin(alpha),                m.cos(alpha),                d],
         [0,             0,                           0,                           1]])

def kinematic_chain(dhParams):
    TMatrix = np.array([
        [1,0,0,0],
        [0,1,0,0],
        [0,0,1,0],
        [0,0,0,1]]
    )

    for joint in range(len(dhParams)):
        TMatrix = np.matmul(TMatrix, dh_transformation(dhParams[joint][0],
                                                        dhParams[joint][1], 
                                                        dhParams[joint][2], 
                                                        dhParams[joint][3]))
    return TMatrix

def get_pos(H):
    return np.array([H[0][3], H[1][3], H[2][3]])

def get_rot(H):
    roll = m.atan2(H[2][1], H[2][2])
    pitch = m.atan2(-H[2][0], m.sqrt(H[2][1]**2 + H[2][2]**2))
    yaw = m.atan2(H[1][0], H[0][0])

    return np.array([roll, pitch, yaw])