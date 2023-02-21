# import solution for problem 1
import robot_model
# import numpy and math for matrix and math functionality
import numpy as np
import math as m

np.set_printoptions(precision = 2, suppress = True)

def moveBot(params):
    '''
    Calculates the position and rotation of an n-DOF robot 
    as defined by Denavit-Hartenberg parameters inputted.
    Parameter:
        params - list of parameters for a given robot, contains theta, a, alpha, and d for each frame.
    Returns:
        None 
    '''
    # Calculating DH matrix using function from problem 1
    dhMatrix = robot_model.kinematic_chain(params)

    # Extracting the position information from the DH matrix using function form problem 1
    pos = robot_model.get_pos(dhMatrix)
    # Extracting the rotation information from the DH matrix using function form problem 1
    rot = robot_model.get_rot(dhMatrix)

    print(f'x = {pos[0]:.2f} \tmeters\ny = {pos[1]:.2f} \tmeters\nz = {pos[2]:.2f} \tmeters')
    print(f'roll = {rot[0]:.2f} \tradians\npitch = {rot[1]:.2f} \tradians\nyaw = {rot[2]:.2f} \tradians')

if __name__ == "__main__":

    # Defining parameters for problem A
    # 2 DOF robot arm defined in lectures
    paramsA = np.array([
        [m.pi/2, 1, 0, 0],
        [m.pi/2, 1, 0, 0]
    ])

    # Defining parameters for problem B part 1
    # 6 DOF robot arm defined in lectures
    paramsB1 = np.array([
        [0,     0,         0.1625,     m.pi/2],
        [0,    -0.425,     0,          0],
        [0,    -0.3922,    0,          0],
        [0,     0,         0.133,      m.pi/2],
        [0,     0,         0.0997,    -m.pi/2],
        [0,     0,         0.0996,     0]
    ])

    # Defining parameters for problem B part 2
    # 6 DOF robot arm defined in lectures
    paramsB2 = np.array([
        [0,         0,         0.1625,     m.pi/2],
        [-m.pi/2,  -0.425,     0,          0],
        [0,        -0.3922,    0,          0],
        [0,         0,         0.133,      m.pi/2],
        [0,         0,         0.0997,    -m.pi/2],
        [0,         0,         0.0996,     0]
    ])

    print("\nPart A position and rotation:")
    moveBot(paramsA)
    print("\nPart B case 1 position and rotation:")
    moveBot(paramsB1)
    print("\nPart B case 2 position and rotation:")
    moveBot(paramsB2)

    