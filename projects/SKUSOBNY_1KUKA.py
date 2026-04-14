import pybullet as p
import pybullet_data
import time
import numpy as np
import matplotlib.pyplot as plt

"""
Úloha 1 – Trajektória v task space (KUKA)

Zadanie:
- použi KUKA robota
- vygeneruj trajektóriu medzi bodmi:
    start = [0.5, 0.0, 0.5]
    end   = [1.5, 0.0, 1.0]
- lineárna interpolácia v task space
- minimálne 50 bodov
- pre každý bod vypočítaj IK
- vykresli trajektóriu robota
"""

# ------------------------------------------------------------
# Pomocné funkcie
# ------------------------------------------------------------

def marker(pos, colour, radius=0.01):
    """Nakreslí malú guľku do priestoru."""
    shape = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=radius,
        rgbaColor=colour
    )
    p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=shape,
        basePosition=pos
    )

def init_pos(robot_id, joint_indices, ee_index, first_pose):
    """
    Nastaví robota do prvého bodu trajektórie pomocou IK.
    """
    joint_angles = p.calculateInverseKinematics(robot_id, ee_index, first_pose)

    for i, joint_idx in enumerate(joint_indices):
        p.resetJointState(robot_id, joint_idx, joint_angles[i])

    p.stepSimulation()
    time.sleep(1 / 60)

# ------------------------------------------------------------
# Hlavný program
# ------------------------------------------------------------

if __name__ == "__main__":
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # prostredie
    p.loadURDF("plane.urdf")

    # KUKA robot
    robot_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

    # index end-effectora
    ee_index = 6

    # pohyblivé jointy robota
    joint_indices = [
        i for i in range(p.getNumJoints(robot_id))
        if p.getJointInfo(robot_id, i)[2] != p.JOINT_FIXED
    ]

    # --------------------------------------------------------
    # Trajektória
    # --------------------------------------------------------
    start = np.array([0.5, 0.0, 0.5])
    end   = np.array([1.5, 0.0, 1.0])

    num_points = 60   # minimálne 50
    desired_positions = np.linspace(start, end, num_points)

    # vykresli desired body (červené)
    for pos in desired_positions:
        marker(pos, [1, 0, 0, 1], radius=0.012)

    # nastav robota do prvého bodu
    first_pose = desired_positions[0]
    init_pos(robot_id, joint_indices, ee_index, first_pose)

    actual_positions = []
    errors = []
    prev_actual = None

    # --------------------------------------------------------
    # Pre každý bod trajektórie vypočítaj IK a nastav robot
    # --------------------------------------------------------
    for pos in desired_positions:
        # IK pre KUKA
        joint_angles = p.calculateInverseKinematics(robot_id, ee_index, pos)

        # nastav jointy
        for i, joint_idx in enumerate(joint_indices):
            p.resetJointState(robot_id, joint_idx, joint_angles[i])

        p.stepSimulation()
        time.sleep(1 / 30)

        # aktuálna pozícia end-effectora
        state = p.getLinkState(robot_id, ee_index, computeForwardKinematics=True)
        actual_pos = np.array(state[4])
        actual_positions.append(actual_pos)

        # modré body = actual trajektória
        marker(actual_pos, [0, 0, 1, 1], radius=0.008)

        # spoj actual trajektóriu čiarou
        if prev_actual is not None:
            p.addUserDebugLine(prev_actual, actual_pos, [0, 0, 1], 2, 0)

        prev_actual = actual_pos

        # chyba
        err = np.linalg.norm(pos - actual_pos)
        errors.append(err)

    actual_positions = np.array(actual_positions)
    errors = np.array(errors)

    print("Počet bodov trajektórie:", num_points)
    print("Počiatočný bod:", start)
    print("Koncový bod:", end)
    print("Priemerná chyba:", np.mean(errors))
    print("Maximálna chyba:", np.max(errors))

    input("Stlač Enter pre zobrazenie grafu...")
    p.disconnect()

    # graf chyby po trajektórii
    plt.figure(figsize=(8, 4))
    plt.plot(errors, label="Chyba polohy")
    plt.title("Chyba sledovania trajektórie")
    plt.xlabel("Bod trajektórie")
    plt.ylabel("Chyba [m]")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()