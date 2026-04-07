import pybullet as p
import pybullet_data
import numpy as np
import time

"""
5tworkspace.py

Tento kód kreslí WORKSPACE robota.

Workspace =
všetky body, kam sa vie end-effector dostať.

Postup:
1. zvolím rozsahy uhlov theta1 a theta2
2. prejdem všetky kombinácie uhlov
3. cez FK vypočítam pozíciu end-effectora
4. vykreslím všetky body

Dôležité:
- dvojitý for loop = skúšam všetky kombinácie 2 kĺbov
- keby bol 1 loop, dostanem len krivku, nie celý workspace
"""

def transformation_matrix(theta, translation):
    """
    transformačná matica jedného článku
    theta = uhol kĺbu
    translation = dĺžka článku
    """
    ct, st = np.cos(theta), np.sin(theta)
    tx = tz = translation

    return np.array([
        [ct,   0,  st, tx * st],
        [0,    1,  0,  0],
        [-st,  0,  ct, tz * ct],
        [0,    0,  0,  1]
    ])

def compute_fk(joint_angles):
    """
    FK pre 2DOF robota
    joint_angles = [theta1, theta2]

    výstup:
    x, y, z pozícia konca robota
    """
    L1, L2 = 1, 1
    theta1, theta2 = joint_angles

    # transformácia základne
    T_W_Base = np.eye(4)
    T_W_Base[2, 3] = 0.05

    # transformácia prvého článku
    T1 = transformation_matrix(theta1, L1)
    T_link1 = T_W_Base @ T1

    # transformácia druhého článku
    T2 = transformation_matrix(theta2, L2)
    T_link2 = T_link1 @ T2

    # vrátim pozíciu end-effectora
    return T_link2[0, 3], 0, T_link2[2, 3]

def draw_workspace():
    """
    vykreslí workspace pomocou všetkých kombinácií uhlov
    """
    L1, L2 = 1, 1

    # rozsahy uhlov
    theta1_range = np.linspace(0, 1/2 * np.pi, 50)
    theta2_range = np.linspace(0, 1/2 * np.pi, 50)

    points = []
    colors = []

    # prejdi všetky kombinácie theta1 a theta2
    for theta1 in theta1_range:
        for theta2 in theta2_range:
            x, y, z = compute_fk([theta1, theta2])
            points.append([x, y, z])
            colors.append([0, 0, 1])

    # vykreslenie workspace bodov
    p.addUserDebugPoints(points, colors, pointSize=2)

if __name__ == "__main__":
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    p.loadURDF("plane.urdf")

    # načítanie 2DOF robota
    robot_id = p.loadURDF("urdf/2dof_planar_robot.urdf", basePosition=[0, 0, 0], useFixedBase=True)

    # indexy jointov
    joint_indices = [1, 2]

    # toto sa používa na animáciu pohybu robota
    # nie priamo na výpočet workspace, ale na ukázanie pohybu
    joint_angle1 = np.linspace(0, 1/2 * np.pi, 50)
    joint_angle2 = np.linspace(0, 1/2 * np.pi, 50)

    # vykresli workspace
    draw_workspace()

    # animácia pohybu robota
    num_step = 1000
    for _ in range(num_step):
        for angle1 in joint_angle1:
            for angle2 in joint_angle2:
                angles = [angle1, angle2]

                for i, angle in zip(joint_indices, angles):
                    p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=angle)

                p.stepSimulation()
                time.sleep(0.01)

    # v pôvodnom súbore je preklep dissconect()
    p.disconnect()

"""
CHEAT:
workspace = všetky body kam robot dosiahne

theta1_range, theta2_range = rozsahy uhlov
for theta1:
    for theta2:
        FK(theta1, theta2)

→ dostanem všetky možné pozície end-effectora
"""
