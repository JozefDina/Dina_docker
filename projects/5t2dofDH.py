import pybullet as p
import pybullet_data
import numpy as np
import os
import time

def dh_transformation(theta, a, d, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([[ct,   -st*ct,     st*sa,      a*ct],
                     [st,   ct*ca,      -ct*sa,     a*st],
                     [0,    sa,         ca,                d],
                     [0,    0,          0,                 1]])

def draw_frame(T, length = 0.2):
    origin = T[:3, 3]
    x_axis = origin + T[:3, 0] * length
    y_axis = origin + T[:3, 1] * length
    z_axis = origin + T[:3, 2] * length

    p.addUserDebugLine(origin, x_axis, [1, 0, 0], 2)
    p.addUserDebugLine(origin, y_axis, [0, 1, 0], 2)
    p.addUserDebugLine(origin, z_axis, [0, 0, 1], 2)

if __name__ == "__main__":

    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    robot_id = p.loadURDF("urdf/2dof_planar3.urdf", basePosition = [0,0,0], useFixedBase = True)

    joint_angles = [np.pi/8, np.pi/4]

    dh_params = [[1,0,0], #joint 1
                 [1,0,0]]   #joint 2
    
    num_dof = 2

    T = np.eye(4)
    T_list = np.zeros((num_dof, 4, 4))
    for i in range(num_dof):
        T_i = dh_transformation(joint_angles[i], *dh_params[i])
        T = T @ T_i
        T_list[i] = T

    fk_position_dh = T[:3, 3]

    for i in range(num_dof):
        p. resetJointState(robot_id, i, joint_angles[i])

    for _ in range(100):
        p.stepSimulation()
        time.sleep(1/240)

    ee_state = p.getLinkState(robot_id, 2, computeForwardKinematics = True)
    fk_position_pybullet = np.array(ee_state[4])

    green_sphere = p.createVisualShape(p.GEOM_SPHERE, radius = 0.05, rgbaColor= [0,1,0,1])
    red_sphere = p.createVisualShape(p.GEOM_SPHERE, radius = 0.05, rgbaColor= [1,0,0,1])

    p.createMultiBody(baseMass = 0, baseVisualShapeIndex = green_sphere, basePosition = fk_position_dh)
    p.createMultiBody(baseMass = 0, baseVisualShapeIndex = red_sphere, basePosition = fk_position_pybullet)

    print(f"DH Computed Position: {fk_position_dh}")
    print(f"Pybullet Computed Position: {fk_position_pybullet}")

    while True:
        p.stepSimulation()
        time.sleep(1/240)