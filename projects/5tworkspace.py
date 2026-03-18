import pybullet as p
import pybullet_data
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

def transformation_matrix(theta, translation):
    ct, st = np.cos(theta), np.sin(theta)
    tx = tz = translation
    return np.array([[ct,   0,  st, tx*st],
                     [0,    1,  0,   0],
                     [-st,  0,  ct, tz*ct],
                     [0,    0,  0,  1]])
def compute_fk(joint_angles):
    L1, L2 = 1, 1
    theta1, theta2 = joint_angles

    T_W_Base = np.eye(4)
    T_W_Base[2,3] = 0.05

    T1 = transformation_matrix(theta1, L1)
    T_link1 = T_W_Base @ T1

    T2 = transformation_matrix(theta2, L2)
    T_link2 = T_link1 @ T2

    return T_link2[0, 3], 0, T_link2[2, 3]

def draw_workspace():
    L1, L2 = 1, 1

    theta1_range = np.linspace(0, 1/2*np.pi, 50)
    theta2_range = np.linspace(0, 1/2*np.pi, 50)

    point = []
    color = []

    for theta1 in theta1_range:
        for theta2 in theta2_range:
            x, y, z = compute_fk([theta1, theta2])
            point.append([x, y, z])
            color.append([0, 0, 1])

    p.addUserDebugPoints(point, color, pointSize = 2)

if __name__ == "__main__":
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.8)
    p.loadURDF("plane.urdf")

    robot_id = p.loadURDF("urdf/2dof_planar_robot.urdf", basePosition = [0, 0, 0], useFixedBase = True)

    joint_indices = [1, 2]
    joint_angle1 = np.linspace(0, 1/2*np.pi,50)
    joint_angle2 = np.linspace(0, 1/2*np.pi,50)

    draw_workspace()
    num_step = 1000
    for _ in range(num_step):
        for angle1 in joint_angle1:
            for angle2 in joint_angle2:
                angles = [angle1, angle2]
                for i, angle in zip(joint_indices, angles):
                    p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition = angle)
                    p.stepSimulation()
                    time.sleep(0.01)

    p.dissconect()
