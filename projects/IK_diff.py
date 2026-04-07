import pybullet as p
import pybullet_data
import time
import numpy as np
import matplotlib.pyplot as plt

"""
IK_diff.py

Tento kód robí trajektóriu cez BODY.
Nejde cez Jacobian a rýchlosti ako IK_diff2.py,
ale takto:

1. vytvorím body trajektórie medzi start a end
2. pre každý bod zavolám klasickú IK
3. robot ide postupne po bodoch

Toto je vhodné, keď je zadanie:
→ choď zo start do end
→ sprav čiaru / trajektóriu cez body
→ porovnaj desired a actual path
"""

def marker(pos, colour):
    # nakreslí malú guľku v priestore
    # používa sa na zobrazenie:
    # - desired trajektórie
    # - actual trajektórie
    shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.005, rgbaColor=colour)
    p.createMultiBody(baseVisualShapeIndex=shape, basePosition=pos)

def init_pos(robot_id, first_pose, orientation):
    """
    Nastaví robota do počiatočnej pozície.
    first_pose = prvý bod trajektórie
    orientation = požadovaná orientácia konca robota
    """
    tol = 1e-3
    ee_index = 6

    for _ in range(100):
        # klasická IK:
        # z pozície + orientácie vypočítaj uhly kĺbov
        joint_angles = p.calculateInverseKinematics(robot_id, ee_index, first_pose, orientation)

        # nastav vypočítané uhly na robot
        for i, j in enumerate(joint_indices):
            p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL, targetPosition=joint_angles[i])

            p.stepSimulation()
            time.sleep(1 / 240)

            # zisti aktuálnu pozíciu konca robota
            state = p.getLinkState(robot_id, ee_index)
            actual_pos = np.array(state[4])

            # chyba medzi požadovanou a aktuálnou pozíciou
            err = np.linalg.norm(actual_pos - first_pose)

            if err < tol:
                print("Robot reached init position")
                break

if __name__ == "__main__":
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # načítanie robota
    kuka_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

    # index end-effectora
    ee_index = 6

    # počet jointov robota
    num_joints = p.getNumJoints(kuka_id)

    # zoznam joint indexov
    joint_indices = list(range(num_joints))

    # začiatok a koniec trajektórie
    # toto je jedna z hlavných vecí, ktoré budeš meniť
    start = np.array([0.5, 0.0, 0.6])
    end = np.array([0.6, 0.2, 0.9])

    # počet bodov trajektórie
    num_steps = 100

    # vytvorí body medzi start a end
    positions = np.linspace(start, end, num_steps)

    # požadovaná orientácia end-effectora
    orientation = p.getQuaternionFromEuler([0, 0, 0])

    # červené body = požadovaná trajektória
    for pos in positions:
        marker(pos, [1, 0, 0, 1])

    # robot najprv nastavím do prvého bodu trajektórie
    first_pose = positions[0]
    init_pos(kuka_id, first_pose, orientation)

    # polia na chyby
    pos_errors = []
    ori_errors = []

    # prejdi všetky body trajektórie
    for pos in positions:
        # pre každý bod vypočítaj joint angles
        joint_angles = p.calculateInverseKinematics(kuka_id, ee_index, pos, orientation)

        # nastav jointy
        for i, j in enumerate(joint_indices):
            p.setJointMotorControl2(kuka_id, j, p.POSITION_CONTROL, targetPosition=joint_angles[i])

        p.stepSimulation()
        time.sleep(1 / 240)

        # zisti skutočnú pozíciu a orientáciu
        state = p.getLinkState(kuka_id, ee_index)
        actual_pos = np.array(state[4])
        actual_ori = np.array(state[5])

        # modré body = skutočná trajektória
        marker(actual_pos, [0, 0, 1, 1])

        # pozičná chyba
        pos_error = np.linalg.norm(pos - actual_pos)

        # orientačná chyba
        dot = np.dot(orientation, actual_ori)
        dot = np.clip(dot, -1.0, 1.0)
        ori_errors_deg = 2 * np.arccos(abs(dot)) * (180 / np.pi)

        pos_errors.append(pos_error)
        ori_errors.append(ori_errors_deg)

    input("Press Enter")
    p.disconnect()

    # graf pozičnej a orientačnej chyby
    plt.figure(figsize=(10, 4))

    plt.subplot(1, 2, 1)
    plt.plot(pos_errors, label='Position error (m)')
    plt.title("End effector position error")
    plt.grid(True)
    plt.xlabel('Step')
    plt.ylabel("m")

    plt.subplot(1, 2, 2)
    plt.plot(ori_errors, label='Orientation error (deg)')
    plt.title("End effector orientation error")
    plt.grid(True)
    plt.xlabel('Step')
    plt.ylabel("deg")

    plt.tight_layout()
    plt.show()

"""
CHEAT:
start, end = trajektória
positions = body medzi start a end
calculateInverseKinematics(...) = pre každý bod nájdem joint angles

Červené body = desired trajektória
Modré body = actual trajektória
"""
    plt.tight_layout()
    plt.show()


