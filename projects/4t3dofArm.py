import pybullet as pb
import pybullet_data
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

def decompose_homogenous_matrix(T):
    translation = T[:3, 3] #zavolame vsetky 3 riadky a potom treti stlpec
    rotation_matrix = T[:3, :3] #zavolame vsetky 3 riadky a potom vsetky stlpce
    euler_angles = R. from_matrix(rotation_matrix).as_euler('xyz')
    quaterion = pb.getQuaternionFromEuler(euler_angles)
    return translation, quaterion

if __name__ == "__main__":
    pb.connect(pb.GUI)
    pb.setGravity(0, 0, -9.8)

    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_id = pb.loadURDF("plane.urdf")

    robot_id = pb.loadURDF("urdf/3dof_planar_robot.urdf", basePosition = [0,0,0], useFixedBase=True)

    joint1_idx = 1
    joint2_idx = 2
    joint3_idx = 3

    theta1 = -np.pi/4
    theta2 = np.pi/4
    theta3 = np.pi/18

    pb.setJointMotorControl2(robot_id, joint1_idx, controlMode = pb.POSITION_CONTROL, targetPosition = theta1)
    pb.setJointMotorControl2(robot_id, joint2_idx, controlMode = pb.POSITION_CONTROL, targetPosition = theta2)
    pb.setJointMotorControl2(robot_id, joint3_idx, controlMode = pb.POSITION_CONTROL, targetPosition = theta3)


    L1 = 1
    L2 = 2
    L3 = 3

    T_W_Base = np.eye(4)
    T_W_Base[2,3]=0.05

    T1 = np.array([
        [ np.cos(theta1),             0, np.sin(theta1), L1*np.sin(theta1)],
        [              0,             1,              0,                 0],
        [-np.sin(theta1),             0, np.cos(theta1), L1*np.cos(theta1)],
        [              0,             0,              0,                 1]
    ])

    T2 = np.array([
        [ np.cos(theta2),             0, np.sin(theta2), L2*np.sin(theta2)],
        [              0,             1,              0,                 0],
        [-np.sin(theta2),             0, np.cos(theta2), L2*np.cos(theta2)],
        [              0,             0,              0,                 1]
    ])

    T3 = np.array([
        [ np.cos(theta3),             0, np.sin(theta3), L3*np.sin(theta3)],
        [              0,             1,              0,                 0],
        [-np.sin(theta3),             0, np.cos(theta3), L3*np.cos(theta3)],
        [              0,             0,              0,                 1]
    ])

    T_link1 = T_W_Base @ T1
    T_link2 = T_link1 @ T2
    T_link3 = T_link2 @ T3

    link1_position, link1_orientation = decompose_homogenous_matrix(T_link1)
    link2_position, link2_orientation = decompose_homogenous_matrix(T_link2)
    link3_position, link3_orientation = decompose_homogenous_matrix(T_link3)


    print("Link 1 position", link1_position)
    print("Link 2 position", link2_position)
    print("Link 3 position", link3_position)


    sphere_colision_shape = pb.createCollisionShape(shapeType=pb.GEOM_SPHERE, radius = 0.05)
    sphere_visual_shape = pb.createVisualShape(shapeType=pb.GEOM_SPHERE, radius = 0.05, rgbaColor = [0,0,1,1])
    pb.createMultiBody(baseMass=0, baseCollisionShapeIndex = sphere_colision_shape, baseVisualShapeIndex = sphere_visual_shape, basePosition = link3_position)

    axis_length = 0.2
    num_steps = 10000
    for t in range(num_steps):
        pb.addUserDebugLine(link1_position, link1_position + T_link1[:3, 0] * axis_length, [1, 0, 0], lineWidth = 3, lifeTime = 1.0)
        pb.addUserDebugLine(link1_position, link1_position + T_link1[:3, 1] * axis_length, [0, 1, 0], lineWidth = 3, lifeTime = 1.0)
        pb.addUserDebugLine(link1_position, link1_position + T_link1[:3, 2] * axis_length, [0, 0, 1], lineWidth = 3, lifeTime = 1.0)
        
        pb.addUserDebugLine(link2_position, link2_position + T_link2[:3, 0] * axis_length, [1, 0, 0], lineWidth = 3, lifeTime = 1.0)
        pb.addUserDebugLine(link2_position, link2_position + T_link2[:3, 1] * axis_length, [0, 1, 0], lineWidth = 3, lifeTime = 1.0)
        pb.addUserDebugLine(link2_position, link2_position + T_link2[:3, 2] * axis_length, [0, 0, 1], lineWidth = 3, lifeTime = 1.0)
        
        pb.addUserDebugLine(link3_position, link3_position + T_link3[:3, 0] * axis_length, [1, 0, 0], lineWidth = 3, lifeTime = 1.0)
        pb.addUserDebugLine(link3_position, link3_position + T_link3[:3, 1] * axis_length, [0, 1, 0], lineWidth = 3, lifeTime = 1.0)
        pb.addUserDebugLine(link3_position, link3_position + T_link3[:3, 2] * axis_length, [0, 0, 1], lineWidth = 3, lifeTime = 1.0)
        pb.stepSimulation()
        time.sleep(1/240)
    pb.disconnect()