import pybullet as p
import pybullet_data
import time
import numpy as np
import matplotlib.pyplot as plt

"""
Úloha 1 – Trajektória v task space

Zadanie:
- použi 2DOF robota
- vygeneruj trajektóriu medzi bodmi:
    start = (0.5, 0.5)
    end   = (1.5, 1.0)
- lineárna interpolácia v task space (rovina XZ)
- minimálne 50 bodov
- pre každý bod vypočítaj IK
- vykresli trajektóriu robota

Poznámka:
Robot pracuje v rovine XZ, preto body zapisujeme v 3D ako:
[x, 0, z]
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

def analytical_ik_2dof(x, z, L1=1.0, L2=1.0, base_z=0.05, elbow_up=False):
    """
    Analytická IK pre 2DOF planar robot v rovine XZ.

    Geometria sedí na tvoje FK kódy:
    x = L1*sin(theta1) + L2*sin(theta1 + theta2)
    z = base_z + L1*cos(theta1) + L2*cos(theta1 + theta2)

    Vstup:
    - x, z ... cieľový bod v rovine XZ
    - L1, L2 ... dĺžky článkov
    - base_z ... výška základne
    - elbow_up ... výber vetvy riešenia

    Výstup:
    - theta1, theta2
    """

    # prechod do lokálneho systému základne
    z_local = z - base_z

    # vzdialenosť od základne k cieľu
    r2 = x**2 + z_local**2

    # kosínová veta pre theta2
    cos_theta2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)

    # orezanie kvôli numerike
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)

    # dve možné riešenia pre theta2
    sin_theta2 = np.sqrt(1.0 - cos_theta2**2)
    if elbow_up:
        sin_theta2 = -sin_theta2

    theta2 = np.arctan2(sin_theta2, cos_theta2)

    # dopočet theta1
    k1 = L1 + L2 * cos_theta2
    k2 = L2 * sin_theta2

    # pozor: u teba je theta meraný od osi z, nie od osi x
    theta1 = np.arctan2(x, z_local) - np.arctan2(k2, k1)

    return theta1, theta2

def fk_2dof(theta1, theta2, L1=1.0, L2=1.0, base_z=0.05):
    """
    Forward kinematics pre kontrolu výsledku.
    """
    x = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    z = base_z + L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    return np.array([x, 0.0, z])

# ------------------------------------------------------------
# Hlavný program
# ------------------------------------------------------------

if __name__ == "__main__":
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # prostredie
    p.loadURDF("plane.urdf")

    # 2DOF robot
    robot_id = p.loadURDF(
        "urdf/2dof_planar_robot.urdf",
        basePosition=[0, 0, 0],
        useFixedBase=True
    )

    # jointy podľa tvojich kódov
    joint_indices = [1, 2]

    # parametre robota podľa tvojich 2DOF FK príkladov
    L1 = 1.0
    L2 = 1.0
    base_z = 0.05

    # --------------------------------------------------------
    
    # start = (0.5, 0.5)
    # end   = (1.5, 1.0)
    #
    # do PyBullet priestoru to dávame ako [x, 0, z]
    # --------------------------------------------------------
    start_2d = np.array([0.25, 0.75])   # (x, z)
    end_2d   = np.array([1.22, 1.0])   # (x, z)

    num_points = 30   # minimálne 50
    traj_2d = np.linspace(start_2d, end_2d, num_points)

    # desired trajektória v 3D
    desired_positions = [np.array([pt[0], 0.0, pt[1]]) for pt in traj_2d]

    # vykresli desired body (červené)
    for pos in desired_positions:
        marker(pos, [1, 0, 0, 1], radius=0.012)

    actual_positions = []
    errors = []

    prev_actual = None

    # --------------------------------------------------------
    # Pre každý bod trajektórie vypočítaj IK a nastav robot
    # --------------------------------------------------------
    for pos in desired_positions:
        x = pos[0]
        z = pos[2]

        # analytická IK
        theta1, theta2 = analytical_ik_2dof(
            x, z,
            L1=L1,
            L2=L2,
            base_z=base_z,
            elbow_up=False
        )

        # nastav jointy okamžite
        p.resetJointState(robot_id, joint_indices[0], theta1)
        p.resetJointState(robot_id, joint_indices[1], theta2)

        p.stepSimulation()
        time.sleep(1 / 30)

        # skutočná FK pozícia z analytických uhlov
        actual_pos = fk_2dof(theta1, theta2, L1=L1, L2=L2, base_z=base_z)
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
    print("Priemerná chyba:", np.mean(errors))
    print("Maximálna chyba:", np.max(errors))

    input("Press Enter to close...")
    p.disconnect()

    # graf chyby po trajektórii
    plt.figure(figsize=(8, 4))
    plt.plot(errors, label="Position error")
    plt.title("Trajectory tracking error")
    plt.xlabel("Bod trajektórie")
    plt.ylabel("Chyba [m]")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

'''
Použil som 2DOF robota v rovine XZ. 
Medzi počiatočným a koncovým bodom som vytvoril lineárnu trajektóriu cez 30 bodov. 
Pre každý bod som analyticky vypočítal inverse kinematics, teda uhly oboch kĺbov. 
Tieto uhly som nastavil robotu a vykreslil som požadovanú aj skutočnú trajektóriu. 
Na konci bolo vidieť, ako sa robot pohyboval po zadanej dráhe.
'''