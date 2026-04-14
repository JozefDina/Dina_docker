import numpy as np
import matplotlib.pyplot as plt

"""
Úloha 2 – Porovnanie IK metód

Použijeme 2DOF planar robot v rovine XZ.

Budeme porovnávať:
1. analytickú IK
2. numerickú IK (pseudoinverzia Jacobianu)

Pre 10 náhodných bodov porovnáme:
- chybu (distance error)
- počet iterácií
"""

# ------------------------------------------------------------
# Parametre robota
# ------------------------------------------------------------

L1 = 1.0
L2 = 1.0
base_z = 0.05


# ------------------------------------------------------------
# Forward kinematics
# ------------------------------------------------------------

def fk_2dof(theta1, theta2, L1=1.0, L2=1.0, base_z=0.05):
    """
    FK pre 2DOF robot v rovine XZ.

    x = L1*sin(theta1) + L2*sin(theta1 + theta2)
    z = base_z + L1*cos(theta1) + L2*cos(theta1 + theta2)
    """
    x = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    z = base_z + L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    return np.array([x, z])


# ------------------------------------------------------------
# Analytická IK
# ------------------------------------------------------------

def analytical_ik_2dof(x, z, L1=1.0, L2=1.0, base_z=0.05, elbow_up=False):
    """
    Analytická IK pre 2DOF robot.
    """
    z_local = z - base_z
    r2 = x**2 + z_local**2

    cos_theta2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)

    sin_theta2 = np.sqrt(1.0 - cos_theta2**2)
    if elbow_up:
        sin_theta2 = -sin_theta2

    theta2 = np.arctan2(sin_theta2, cos_theta2)

    k1 = L1 + L2 * cos_theta2
    k2 = L2 * sin_theta2

    theta1 = np.arctan2(x, z_local) - np.arctan2(k2, k1)

    return np.array([theta1, theta2])


# ------------------------------------------------------------
# Jacobian
# ------------------------------------------------------------

def jacobian_2dof(theta1, theta2, L1=1.0, L2=1.0):
    """
    Jacobian pre 2DOF robot v rovine XZ.

    x = L1*sin(t1) + L2*sin(t1+t2)
    z = base_z + L1*cos(t1) + L2*cos(t1+t2)
    """
    dx_dt1 = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    dx_dt2 = L2 * np.cos(theta1 + theta2)

    dz_dt1 = -L1 * np.sin(theta1) - L2 * np.sin(theta1 + theta2)
    dz_dt2 = -L2 * np.sin(theta1 + theta2)

    J = np.array([
        [dx_dt1, dx_dt2],
        [dz_dt1, dz_dt2]
    ])

    return J


# ------------------------------------------------------------
# Numerická IK cez pseudoinverziu
# ------------------------------------------------------------

def numerical_ik_pinv(target, q_init=None, max_iters=200, tol=1e-4, alpha=0.5):
    """
    Numerická IK:
    q_{k+1} = q_k + alpha * J_pinv * error
    """
    if q_init is None:
        q = np.array([0.1, 0.1], dtype=float)
    else:
        q = np.array(q_init, dtype=float)

    for it in range(1, max_iters + 1):
        current = fk_2dof(q[0], q[1], L1=L1, L2=L2, base_z=base_z)
        error = target - current

        if np.linalg.norm(error) < tol:
            return q, np.linalg.norm(error), it

        J = jacobian_2dof(q[0], q[1], L1=L1, L2=L2)
        J_pinv = np.linalg.pinv(J)

        dq = alpha * (J_pinv @ error)
        q = q + dq

    # ak sa nestihne trafiť do tol
    current = fk_2dof(q[0], q[1], L1=L1, L2=L2, base_z=base_z)
    error = target - current
    return q, np.linalg.norm(error), max_iters


# ------------------------------------------------------------
# Generovanie náhodných dosiahnuteľných bodov
# ------------------------------------------------------------

def generate_random_reachable_points(n=10):
    """
    Najjednoduchší spôsob:
    vygenerujem náhodné uhly, cez FK dostanem body.
    Tak mám istotu, že sú dosiahnuteľné.
    """
    points = []

    for _ in range(n):
        theta1 = np.random.uniform(-np.pi / 2, np.pi / 2)
        theta2 = np.random.uniform(-np.pi / 2, np.pi / 2)

        point = fk_2dof(theta1, theta2, L1=L1, L2=L2, base_z=base_z)
        points.append(point)

    return np.array(points)


# ------------------------------------------------------------
# Hlavný program
# ------------------------------------------------------------

import pybullet as p
import pybullet_data
import time

if __name__ == "__main__":
    

    # -----------------------------
    # PYBULLET - vizualizácia robota
    # -----------------------------
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    p.loadURDF("plane.urdf")

    robot_id = p.loadURDF(
        "urdf/2dof_planar_robot.urdf",
        basePosition=[0, 0, 0],
        useFixedBase=True
    )

    joint_indices = [1, 2]

    # -----------------------------
    # Generovanie bodov
    # -----------------------------
    targets = generate_random_reachable_points(10)

    analytical_errors = []
    numerical_errors = []
    analytical_iters = []
    numerical_iters = []

    print("Porovnanie analytickej a numerickej IK")
    print("=" * 60)

    # -----------------------------
    # Pre každý bod
    # -----------------------------
    for i, target in enumerate(targets, start=1):
        x, z = target

        # -------------------------
        # analytická IK
        # -------------------------
        q_analytical = analytical_ik_2dof(x, z, L1=L1, L2=L2, base_z=base_z)
        pos_analytical = fk_2dof(q_analytical[0], q_analytical[1], L1=L1, L2=L2, base_z=base_z)
        err_analytical = np.linalg.norm(target - pos_analytical)

        analytical_errors.append(err_analytical)
        analytical_iters.append(1)

        # -------------------------
        # numerická IK
        # -------------------------
        q_numerical, err_numerical, iters_numerical = numerical_ik_pinv(
            target=target,
            q_init=[0.1, 0.1]
        )

        numerical_errors.append(err_numerical)
        numerical_iters.append(iters_numerical)

        print(f"Bod {i}: {np.round(target, 4)}")

        # -------------------------
        # ZOBRAZENIE ROBOTA
        # -------------------------

        # analytické riešenie (červené)
        p.resetJointState(robot_id, joint_indices[0], q_analytical[0])
        p.resetJointState(robot_id, joint_indices[1], q_analytical[1])

        for _ in range(50):
            p.stepSimulation()
            time.sleep(1/240)

        # numerické riešenie (modré)
        p.resetJointState(robot_id, joint_indices[0], q_numerical[0])
        p.resetJointState(robot_id, joint_indices[1], q_numerical[1])

        for _ in range(50):
            p.stepSimulation()
            time.sleep(1/240)

    print("\nRobot zobrazený. Pozri si pohyb.")
    input("👉 Stlač Enter pre grafy...")

    p.disconnect()

    # -----------------------------
    # GRAFY
    # -----------------------------
    analytical_errors = np.array(analytical_errors)
    numerical_errors = np.array(numerical_errors)
    analytical_iters = np.array(analytical_iters)
    numerical_iters = np.array(numerical_iters)

    import matplotlib.pyplot as plt

    plt.figure(figsize=(10, 4))

    plt.subplot(1, 2, 1)
    plt.plot(analytical_errors, 'o-', label='Analytická IK')
    plt.plot(numerical_errors, 's-', label='Numerická IK')
    plt.title("Chyba")
    plt.grid(True)
    plt.legend()

    plt.subplot(1, 2, 2)
    plt.plot(analytical_iters, 'o-', label='Analytická IK')
    plt.plot(numerical_iters, 's-', label='Numerická IK')
    plt.title("Počet iterácií")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()

    print("\nZhrnutie:")
    print(f"Priemerná chyba analytickej IK: {np.mean(analytical_errors):.8f}")
    print(f"Priemerná chyba numerickej IK:  {np.mean(numerical_errors):.8f}")
    print(f"Priemerný počet iterácií analytickej IK: {np.mean(analytical_iters):.2f}")
    print(f"Priemerný počet iterácií numerickej IK:  {np.mean(numerical_iters):.2f}")

'''
Na riešenie som použil 2DOF robota v rovine XZ.
Najprv som vygeneroval 10 náhodných dosiahnuteľných bodov pomocou forward kinematics. 
Pre každý bod som vypočítal analytickú IK pomocou vzorcov a numerickú IK pomocou pseudoinverzie 
Jacobianu. Pri numerickej metóde som v iteráciách počítal chybu a upravoval uhly kĺbov, 
kým sa robot nepriblížil k cieľu. Nakoniec som porovnal chybu a počet iterácií, 
pričom analytická IK bola presná a rýchla, zatiaľ čo numerická mala malú chybu a viac iterácií.
'''