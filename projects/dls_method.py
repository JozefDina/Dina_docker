import pybullet as p
import pybullet_data
import numpy as np
import time

# ------------------------------------------------------------
# NUMERICKÁ INVERSE KINEMATICS pomocou DLS
# DLS = Damped Least Squares
#
# Cieľ:
# dostať end-effector robota do zadanej pozície target_pos
#
# Myšlienka:
# 1. zistím aktuálnu pozíciu konca robota
# 2. spočítam chybu = kam chcem ísť - kde som teraz
# 3. cez Jacobian vypočítam, ako zmeniť uhly kĺbov
# 4. opakujem, kým chyba nie je malá
# ------------------------------------------------------------
def numerical_ik(robot_id, end_efector_index, target_pos, joint_indices,
                 max_iters=100, threshold=1e-3, alpha=0.1, damping=0.1):
    
    # cyklus - robot sa bude postupne približovať k cieľu
    for _ in range(max_iters):

        # zistenie aktuálnych uhlov kĺbov
        # p.getJointState(...)[0] = aktuálny uhol kĺbu
        joint_states = [p.getJointState(robot_id, i)[0] for i in joint_indices]

        # zistenie aktuálnej pozície konca robota
        # link_state[4] = world position end-effectora
        link_state = p.getLinkState(robot_id, end_efector_index, computeForwardKinematics=True)
        current_pos = np.array(link_state[4])

        # chyba = kam chcem ísť - kde som teraz
        error = np.array(target_pos) - current_pos

        # ak je chyba už malá, končíme
        if np.linalg.norm(error) < threshold:
            break

        # nulové rýchlosti a zrýchlenia - PyBullet ich chce pri calculateJacobian
        zero_vec = [0.0] * len(joint_indices)

        # výpočet Jacobianu
        # J_lin = lineárna časť Jacobianu (vzťah medzi pohybom kĺbov a pohybom pozície)
        # localPosition = [0,0,0] znamená, že berieme priamo origin end-effector frame-u
        J_lin, _ = p.calculateJacobian(
            robot_id,
            end_efector_index,
            localPosition=[0, 0, 0],
            objPositions=joint_states,
            objVelocities=zero_vec,
            objAccelerations=zero_vec,
        )

        # prevedenie na numpy pole
        J = np.array(J_lin)

        # transponovaný Jacobian
        JT = J.T

        # DLS vzorec:
        # d_theta = alpha * J^T * inv(J*J^T + lambda^2 * I) * error
        #
        # d_theta = o koľko upravíme uhly kĺbov
        # alpha = krok učenia / rýchlosť priblíženia
        # damping = stabilizácia, aby to neblblo pri singularitách
        d_theta = alpha * JT @ np.linalg.inv(J @ JT + damping**2 * np.eye(3)) @ error

        # aktualizácia uhlov kĺbov
        # každý joint posunieme o príslušnú hodnotu z d_theta
        for i, joint_idx in enumerate(joint_indices):
            p.resetJointState(robot_id, joint_idx, joint_states[i] + d_theta[i])
            time.sleep(1 / 240)

        # posun simulácie o jeden krok
        p.stepSimulation()


if __name__ == "__main__":
    # pripojenie do PyBullet GUI
    p.connect(p.GUI)

    # cesta k základným PyBullet dátam
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # gravitácia
    p.setGravity(0, 0, -9.8)

    # načítanie roviny
    p.loadURDF("plane.urdf")

    # načítanie KUKA robota
    # useFixedBase=True = základňa robota je pevná
    kuka_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

    # index koncového efektora
    end_efector_index = 6

    # cieľová pozícia, kam má robot ísť
    # toto je jedna z hlavných vecí, ktoré budeš na zápočte meniť
    target_position = [0.7, 0.0, 0.5]

    # vyberieme iba pohyblivé jointy
    # pevné jointy nechceme
    movable_joints = [
        i for i in range(p.getNumJoints(kuka_id))
        if p.getJointInfo(kuka_id, i)[2] != p.JOINT_FIXED
    ]

    # zavolanie numerickej IK
    numerical_ik(kuka_id, end_efector_index, target_position, movable_joints)

    # konečná pozícia po dobehnutí algoritmu
    final_pos = p.getLinkState(kuka_id, end_efector_index)[4]

    # výpis výsledkov
    print("Target:", np.round(target_position, 3))
    print("Achieved:", np.round(final_pos, 3))
    print("Final error:", np.round(np.linalg.norm(np.array(target_position) - np.array(final_pos)), 6))

    # počkaj na enter, aby sa okno hneď nezavrelo
    input("Press enter to disconnect ...")

    # odpojenie od PyBullet
    p.disconnect()
