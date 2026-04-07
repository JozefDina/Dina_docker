import pybullet as p
import pybullet_data
import numpy as np
import time

"""
5t2dofDH.py

Tento kód používa DH parametre na výpočet forward kinematics.

Cieľ:
→ z uhlov kĺbov a DH parametrov vypočítať pozíciu konca robota
→ porovnať výsledok s PyBullet

DH úloha býva typicky:
- dostaneš theta, a, d, alpha
- skladáš transformačné matice
- na konci porovnáš pozíciu end-effectora
"""

def dh_transformation(theta, a, d, alpha):
    """
    DH transformačná matica

    Vstup:
    theta = uhol kĺbu
    a     = posun po osi x
    d     = posun po osi z
    alpha = rotácia medzi osami

    Výstup:
    4x4 homogénna transformačná matica
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [ct,   -st * ct,   st * sa,   a * ct],
        [st,    ct * ca,  -ct * sa,   a * st],
        [0,     sa,        ca,        d],
        [0,     0,         0,         1]
    ])

def draw_frame(T, length=0.2):
    """
    vykreslí súradnicový frame:
    červená = x
    zelená  = y
    modrá   = z
    """
    origin = T[:3, 3]
    x_axis = origin + T[:3, 0] * length
    y_axis = origin + T[:3, 1] * length
    z_axis = origin + T[:3, 2] * length

    p.addUserDebugLine(origin, x_axis, [1, 0, 0], 2)
    p.addUserDebugLine(origin, y_axis, [0, 1, 0], 2)
    p.addUserDebugLine(origin, z_axis, [0, 0, 1], 2)

if __name__ == "__main__":

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # načítanie 2DOF robota
    robot_id = p.loadURDF("urdf/2dof_planar3.urdf", basePosition=[0, 0, 0], useFixedBase=True)

    # uhly kĺbov
    joint_angles = [np.pi / 8, np.pi / 4]

    # DH parametre pre jednotlivé jointy
    # formát: [a, d, alpha]
    dh_params = [
        [1, 0, 0],   # joint 1
        [1, 0, 0]    # joint 2
    ]

    num_dof = 2

    # T = aktuálna celková transformácia
    T = np.eye(4)

    # sem si budem ukladať transformácie jednotlivých článkov
    T_list = np.zeros((num_dof, 4, 4))

    # FORWARD KINEMATICS cez DH
    for i in range(num_dof):
        T_i = dh_transformation(joint_angles[i], *dh_params[i])

        # skladanie transformácií
        T = T @ T_i
        T_list[i] = T

        # ak chceš, môžeš si aj vykresliť frame každého článku
        draw_frame(T)

    # DH výsledok = pozícia end-effectora
    fk_position_dh = T[:3, 3]

    # nastav uhly jointov v PyBullet modeli
    for i in range(num_dof):
        p.resetJointState(robot_id, i, joint_angles[i])

    # nechaj simuláciu prebehnúť
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1 / 240)

    # zistenie skutočnej pozície end-effectora z PyBullet
    ee_state = p.getLinkState(robot_id, 2, computeForwardKinematics=True)
    fk_position_pybullet = np.array(ee_state[4])

    # zelená guľa = DH výsledok
    green_sphere = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[0, 1, 0, 1])

    # červená guľa = PyBullet výsledok
    red_sphere = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 1])

    p.createMultiBody(baseMass=0, baseVisualShapeIndex=green_sphere, basePosition=fk_position_dh)
    p.createMultiBody(baseMass=0, baseVisualShapeIndex=red_sphere, basePosition=fk_position_pybullet)

    print(f"DH Computed Position: {fk_position_dh}")
    print(f"PyBullet Computed Position: {fk_position_pybullet}")
    print(f"Difference: {fk_position_dh - fk_position_pybullet}")

    while True:
        p.stepSimulation()
        time.sleep(1 / 240)

"""


DH:
→ mám tabuľku parametrov
→ pre každý joint spravím T_i
→ všetko skladám:
T = T @ T_i

pozícia end-effectora:
T[:3, 3]

ak sa porovnáva s PyBullet:
DH výsledok vs getLinkState(...)
"""
