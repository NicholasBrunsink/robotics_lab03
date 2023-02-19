import robot_model
import numpy as np
import math as m

np.set_printoptions(precision = 2, suppress = True)

def moveBot(params):
    dhMatrix = robot_model.kinematic_chain(params)
    pos = robot_model.get_pos(dhMatrix)
    rot = robot_model.get_rot(dhMatrix)

    print(f'x = {pos[0]:.2f} \tmeters\ny = {pos[1]:.2f} \tmeters\nz = {pos[2]:.2f} \tmeters')
    print(f'roll = {rot[0]:.2f} \tradians\npitch = {rot[1]:.2f} \tradians\nyaw = {rot[2]:.2f} \tradians')

if __name__ == "__main__":

    paramsA = np.array([
        [m.pi/2, 1, 0, 0],
        [m.pi/2, 1, 0, 0]
    ])

    paramsB1 = np.array([
        [0,     0,         0.1625,     m.pi/2],
        [0,    -0.425,     0,          0],
        [0,    -0.3922,    0,          0],
        [0,     0,         0.133,      m.pi/2],
        [0,     0,         0.0997,    -m.pi/2],
        [0,     0,         0.0996,     0]
    ])

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

    