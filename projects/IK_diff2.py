import pybullet as p
import pybullet_data
import numpy as np
import time
import matplotlib.pyplot as plt


"""
DIFFERENTIAL IK

Cieľ:
→ nechcem ísť len do jedného bodu
→ chcem, aby sa robot hýbal (napr. po kružnici)

Hlavná myšlienka:
desired_velocity = ako sa má hýbať koniec robota
dq = ako sa majú hýbať kĺby

Vzťah:
x_dot = J * q_dot
q_dot = J_pinv * x_dot
"""


def init_pos(robot_id, joint_indices, ee_link_index, target):
    """
    nastaví robota do počiatočnej pozície pomocou klasickej IK
    """
    joint_angles = p.calculateInverseKinematics(robot_id, ee_link_index, target)

    for i, joint_idx in enumerate(joint_indices):
        p.resetJointState(robot_id, joint_idx, joint_angles[i])


def compute_differential_ik(robot_id, joint_indices, ee_link_index, desired_ee_velocity):
    """
    Vstup:
    → desired_ee_velocity = požadovaná rýchlosť konca robota

    Výstup:
    → dq = rýchlosti kĺbov
    """

    # aktuálne uhly kĺbov
    joint_states = p.getJointStates(robot_id, joint_indices)
    q = [s[0] for s in joint_states]

    # nulové rýchlosti (potrebné pre PyBullet)
    dq_zero = [0.0] * len(joint_indices)

    # Jacobian (lineárna časť)
    # hovorí, ako pohyb kĺbov ovplyvní pohyb konca robota
    jac_t, _ = p.calculateJacobian(
        robot_id,
        ee_link_index,
        [0, 0, 0],
        q,
        dq_zero,
        dq_zero
    )

    J = np.array(jac_t)

    # pseudoinverzia Jacobianu
    J_pinv = np.linalg.pinv(J)

    # NAJDÔLEŽITEJŠÍ RIADOK:
    # dq = rýchlosti kĺbov
    # teda ako sa majú hýbať jointy, aby sa robot hýbal požadovanou rýchlosťou
    dq = J_pinv @ desired_ee_velocity

    return dq


if __name__ == "__main__":

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    p.loadURDF("plane.urdf")

    # načítanie robota
    kuka_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

    # index konca robota
    ee_link_index = 6

    # vyber pohyblivých jointov
    joint_indices = [
        i for i in range(p.getNumJoints(kuka_id))
        if p.getJointInfo(kuka_id, i)[2] != p.JOINT_FIXED
    ]

    # počiatočný bod
    first_pose = np.array([0.6, 0.0, 0.7])

    # nastav robot do počiatočnej pozície
    init_pos(kuka_id, joint_indices, ee_link_index, first_pose)

    # parametre kružnice
    center_x = 0.6
    center_y = 0.0
    radius = 0.1

    # logovanie pre grafy
    dq_log = []
    desired_velocity_log = []
    ee_velocity_sim_log = []

    prev_pos = None

    for step in range(1000):

        t = step * 0.01

        # požadovaná rýchlosť → kružnica
        vx = -radius * np.sin(t)
        vy = radius * np.cos(t)
        vz = 0.0

        desired_velocity = np.array([vx, vy, vz])

        # vypočítaj rýchlosti kĺbov
        dq = compute_differential_ik(
            kuka_id,
            joint_indices,
            ee_link_index,
            desired_velocity
        )

        # ulož si hodnoty (pre graf)
        dq_log.append(dq)
        desired_velocity_log.append(desired_velocity)

        # aplikuj rýchlosti na robot
        for i, vel in zip(joint_indices, dq):
            p.setJointMotorControl2(
                kuka_id,
                i,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=vel
            )

        p.stepSimulation()
        time.sleep(1 / 240)

        # aktuálna pozícia robota
        state = p.getLinkState(kuka_id, ee_link_index, computeLinkVelocity=1)
        ee_pos = np.array(state[4])

        # skutočná rýchlosť zo simulácie
        ee_velocity_sim = np.array(state[6])
        ee_velocity_sim_log.append(ee_velocity_sim)

        # kreslenie trajektórie
        if prev_pos is not None:
            p.addUserDebugLine(prev_pos, ee_pos, [0, 0, 1], 2, 2)

        prev_pos = ee_pos

    """
    Grafy:
    - desired_velocity = čo chceme
    - ee_velocity_sim = čo robot robí
    - dq = rýchlosti kĺbov
    """

    dq_log = np.array(dq_log)
    desired_velocity_log = np.array(desired_velocity_log)
    ee_velocity_sim_log = np.array(ee_velocity_sim_log)

    plt.figure()
    plt.title("Desired vs Actual EE velocity")
    plt.plot(desired_velocity_log[:, 0], label="vx desired")
    plt.plot(ee_velocity_sim_log[:, 0], label="vx actual")
    plt.legend()

    plt.figure()
    plt.title("Joint velocities (dq)")
    for i in range(dq_log.shape[1]):
        plt.plot(dq_log[:, i], label=f"joint {i}")
    plt.legend()

    plt.show()

    input("Press Enter to exit...")
    p.disconnect()


"""
CHEAT SHEET:

IK:
→ idem do bodu
→ error = target - current

DIFF IK:
→ idem rýchlosťou
→ dq = J_pinv @ desired_velocity

desired_velocity:
→ ako sa má hýbať koniec robota

dq:
→ ako sa majú hýbať kĺby

Jacobian:
→ preklad medzi jointmi a pohybom robota
"""
