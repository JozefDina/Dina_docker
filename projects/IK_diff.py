import pybullet as p
import pybullet_data
import time
import numpy as np
import matplotlib.pyplot as plt

def marker(pos, colour):
    shape = p.createVisualShape(p.GEOM_SPHERE, radius = 0.005, rgbaColor = colour)
    p.createMultiBody(baseVisualShapeIndex = shape, basePosition = pos)

def init_pos(robot_id, first_pose, orientation):
    tol = 1e-3
    ee_index = 6

    for _ in range(100):
        joint_angles = p.calculateInverseKinematics(robot_id, ee_index, first_pose, orientation)

        for i, j in enumerate(joint_indices):
            p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL, targetPosition = joint_angles[i])

            p.stepSimulation()
            time.sleep(1/240)

            state = p.getLinkState(robot_id, ee_index)
            actual_pos = np.array(state[4])
            err = np.linalg.norm(actual_pos - first_pose)

            if err<tol:
                print(f"Robot reached init position")
                break

if __name__ == "__main__":
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    kuka_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase = True)
    ee_index = 6
    num_joints = p.getNumJoints(kuka_id)
    joint_indices = list(range(num_joints))

    start = np.array([0.5, 0.0, 0.6])
    end = np.array([0.6, 0.2, 0.9])
    num_steps = 100

    positions = np.linspace(start, end, num_steps)
    orientation = p.getQuaternionFromEuler([0, 0, 0])

    for pos in positions:
        marker(pos, [1, 0, 0, 1])

    first_pose = positions[0]
    init_pos(kuka_id, first_pose, orientation)

    pos_errors = []
    ori_errors = []

    for pos in positions:
        joint_angles = p.calculateInverseKinematics(kuka_id, ee_index, pos, orientation)

        for i,j in enumerate(joint_indices):
            p.setJointMotorControl2(kuka_id, j, p.POSITION_CONTROL, targetPosition = joint_angles[i])

        p.stepSimulation()
        time.sleep(1/240)

        state = p.getLinkState(kuka_id, ee_index)
        actual_pos = np.array(state[4])
        actual_ori = np.array(state[5])

        marker(actual_pos, [0, 0, 1, 1])

        pos_error = np.linalg.norm(pos - actual_pos)
        dot = np.dot(orientation, actual_ori)
        dot = np.clip(dot, -1.0, 1.0)
        ori_errors_deg = 2*np.arccos(abs(dot)) * (180/np.pi)

        pos_errors.append(pos_error)
        ori_errors.append(ori_errors_deg)

    input("Press Enter")
    p.disconnect()

    plt.figure(figsize=(10,4))
    plt.subplot(1,2,1)
    plt.plot(pos_errors, label = 'Position error (m)')
    plt.title("End efector position error")
    plt.grid(True)
    plt.xlabel('Step')
    plt.ylabel("m")
    
    plt.subplot(1,2,2)
    plt.plot(ori_errors, label = 'Orentation error (deg)')
    plt.title("End efector orientation error")
    plt.grid(True)
    plt.xlabel('Step')
    plt.ylabel("deg")
    
    plt.tight_layout()
    plt.show()


