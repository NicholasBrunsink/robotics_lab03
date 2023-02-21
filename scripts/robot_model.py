# Import math and numpy for matrix and math operations
import math as m
import numpy as np

def dh_transformation(theta, a, d, alpha):
    '''
    Generate a homogenous transformation matrix based on a single frame
    from iputted Denavit-Hartenberg parameters using the matrix below.
    |  cos(theta), -sin(theta)*cos(alpha),   sin(theta)*m.sin(alpha),   a*cos(theta)  |
    |  sin(theta),  cos(theta)*cos(alpha),  -cos(theta)*m.sin(alpha),   a*sin(theta)  |
    |  0,           sin(alpha),              cos(alpha),                d             |
    |  0,           0,                       0,                         1             |
    
    Parameters:
        theta - angle to be rotated
        a - link length
        alpha - twist angle between z axis of current frame and previous frame.
        d - link offset from previous link
    Returns:
        a homogenous transformation matrix based on the parameters.
    '''
    return np.array(
        [[m.cos(theta), -m.sin(theta)*m.cos(alpha),   m.sin(theta)*m.sin(alpha),   a*m.cos(theta)],
         [m.sin(theta),  m.cos(theta)*m.cos(alpha),  -m.cos(theta)*m.sin(alpha),   a*m.sin(theta)],
         [0,             m.sin(alpha),                m.cos(alpha),                d],
         [0,             0,                           0,                           1]])

def kinematic_chain(dhParams):
    '''
    Calculates the homogenous transformation matrix for each frame
    Parameters:
        dhParams - a list containing the Denavit-Hartenberg parameters 
            for each frame of a robot.
    Returns:
        TMatrix - the final transformation computed from every frame
    '''
    # Initializing a 4x4 identity matrix to build off of
    TMatrix = np.array([
        [1,0,0,0],
        [0,1,0,0],
        [0,0,1,0],
        [0,0,0,1]]
    )

    # Iterating through each frame in dhParams
    for joint in range(len(dhParams)):
        # Calculate the frame's tranformation matrix based on dhParams and the previous TMatrix using the current frame.
        TMatrix = np.matmul(TMatrix, dh_transformation(dhParams[joint][0],
                                                        dhParams[joint][1], 
                                                        dhParams[joint][2], 
                                                        dhParams[joint][3]))
    return TMatrix

def get_pos(H):
    '''Returns the x, y, and z coordinates from an inputted 4x4 Homogenous transformation matrix'''
    return np.array([H[0][3], H[1][3], H[2][3]])

def get_rot(H):
    '''Returns the roll, pitch, and yaw rotation calculated from an inputted 4x4 Homogenous transformation matrix'''
    # Calculate roll
    roll = m.atan2(H[2][1], H[2][2])
    # Calculate pitch
    pitch = m.atan2(-H[2][0], m.sqrt(H[2][1]**2 + H[2][2]**2))
    # Calculate yaw
    yaw = m.atan2(H[1][0], H[0][0])

    return np.array([roll, pitch, yaw])
