import pybullet as p
import pybullet_data
import time
import numpy as np
import matplotlib.pyplot as plt


def init_pos(robot_id, first_pose):
    tol = 1e-3
    ee_index = 6

    for _ in range(100):
        joint_angles = p.calculateInverseKinematics(robot_id, ee_index, first_pose)

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

def compute_differential_ik(robot_id, joint_indices, ee_link_index, desired_ee_velocity):
    joint_states = p.getJointStates(robot_id, joint_indices)
    q = [s[0] for s in joint_states]
    dq_zero = [0.0] * len(joint_indices)

    local_pos = [0, 0, 0]

    jac_t, _ = p.calculateJacobian(robot_id, ee_link_index, local_pos, q, dq_zero, dq_zero)

    J = np.array(jac_t)
    J_pinv = np.linalg.pinv(J)

    dq = J_pinv @ desired_ee_velocity

    return dq

if __name__ == "__main__":

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(0.01)

    plane_id = p.loadURDF("plane.urdf")
    kuka_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase = True)

    joint_indices = [i for i in range(7)]
    ee_link_index = 6

    center_x = 0.5
    center_y = 0.0

    radius = 0.1

    dq_log = []
    desired_velocity_log = []
    actual_velocity_log = []
    time_log = []
    ee_velocity_sim_log = []
    prev_ee_pos = None

    first_pose = np.array([0.6, 0.0, 0.7])

    init_pos(kuka_id, first_pose)

    for step in range(1000):
        t = step * 0.01

        vx = -radius * np.sin(t)
        vy = radius * np.cos(t)
        vz = 0.0
        desired_velocity = np.array([vx, vy, vz])

        dq = compute_differential_ik(kuka_id, joint_indices, ee_link_index, desired_velocity)

        dq_log.append(dq.copy())
        desired_velocity_log.append(desired_velocity.copy())
        time_log.append(t)

        state = p.getLinkState(kuka_id, ee_link_index, computeForwardKinematics = True)
        ee_pos = state[4]
        state = p.getLinkState(kuka_id, ee_link_index, computeLinkVelocity = 1)
        ee_velocity_sim = np.array(state[6])
        ee_velocity_sim_log.append(ee_velocity_sim.copy())

        if prev_ee_pos is not None:
            p.addUserDebugLine(prev_ee_pos, ee_pos, lineColorRGB = [1,0,0], lineWidth = 2, lifeTime = 0)
        prev_ee_pos = ee_pos

        for i, vel in zip (joint_indices, dq):
            p.setJointMotorControl2(kuka_id,i, controlMode = p.VELOCITY_CONTROL, targetVelocity = vel)

        p.stepSimulation()
        time.sleep(0.01)
    p.disconnect()

    desired_velocity_log = np.array(desired_velocity_log)
    ee_velocity_sim_log = np.array(ee_velocity_sim_log)
    dq_log = np.array(dq_log)

    plt.figure(figsize = (10, 6))
    labels = ['vx' , 'vy', 'vz']

    for i in range(3):
        plt.plot(time_log, desired_velocity_log[:,i],'--',label = f"Desired{labels[i]}")
        plt.plot(time_log, ee_velocity_sim_log[:,i],'--',label = f"Angular{labels[i]}")
        plt.plot(time_log, dq_log[:,i],'--',label = f"Angular velocity{labels[i]}")

    plt.xlabel('Time (s)')
    plt.ylabel('End efector velocity')
    plt.title('Desired and actual velocities')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()
