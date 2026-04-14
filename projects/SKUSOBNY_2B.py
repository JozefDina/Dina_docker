import pybullet as p
import pybullet_data
import numpy as np
import time

"""
Úloha 2 – DH vs PyBullet

Zadanie:
- zvoľ vlastné uhly kĺbov
- vypočítaj FK cez DH
- vypočítaj FK cez PyBullet
- porovnaj výsledné pozície
- vypíš rozdiel
"""

def dh_transformation(theta, a, d, alpha):
    """
    DH transformačná matica
    theta = uhol kĺbu
    a, d, alpha = DH parametre
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [ct,   -st * ca,   st * sa,   a * ct],
        [st,    ct * ca,  -ct * sa,   a * st],
        [0,     sa,        ca,        d],
        [0,     0,         0,         1]
    ])

def draw_frame(T, length=0.2):
    """
    vykreslí frame:
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

    # vlastné uhly kĺbov
    joint_angles = [np.pi / 6, np.pi / 4]

    # DH parametre [a, d, alpha]
    dh_params = [
        [1, 0, 0],   # joint 1
        [1, 0, 0]    # joint 2
    ]

    num_dof = 2

    # celková transformácia
    T = np.eye(4)

    # uloženie frame-ov článkov
    T_list = np.zeros((num_dof, 4, 4))

    # FK cez DH
    for i in range(num_dof):
        T_i = dh_transformation(joint_angles[i], *dh_params[i])
        T = T @ T_i
        T_list[i] = T

        # vykresli frame každého článku
        draw_frame(T)

    # výsledná DH pozícia end-effectora
    fk_position_dh = T[:3, 3]

    # nastav tie isté uhly robotu v PyBullet
    for i in range(num_dof):
        p.resetJointState(robot_id, i, joint_angles[i])

    # nechaj simuláciu chvíľu bežať
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1 / 240)

    # pozícia end-effectora z PyBullet
    ee_state = p.getLinkState(robot_id, 2, computeForwardKinematics=True)
    fk_position_pybullet = np.array(ee_state[4])

    # vizualizácia výsledkov
    green_sphere = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[0, 1, 0, 1])
    red_sphere = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 1])

    # zelená = DH výsledok
    p.createMultiBody(baseMass=0, baseVisualShapeIndex=green_sphere, basePosition=fk_position_dh)

    # červená = PyBullet výsledok
    p.createMultiBody(baseMass=0, baseVisualShapeIndex=red_sphere, basePosition=fk_position_pybullet)

    # výpis
    print("Zvolené uhly kĺbov:", joint_angles)

    print("Pozícia koncového efektora (DH):", fk_position_dh)
    print("Pozícia koncového efektora (PyBullet):", fk_position_pybullet)

    print("Rozdiel pozícií:", fk_position_dh - fk_position_pybullet)

    print("Veľkosť chyby (vzdialenosť):",
        np.linalg.norm(fk_position_dh - fk_position_pybullet))
    input("Press Enter to close...")
    p.disconnect()

'''
Použil som 2DOF robota a zvolil som vlastné uhly kĺbov. 
Najprv som pomocou DH parametrov vypočítal forward kinematics a získal polohu konca robota. 
Potom som tie isté uhly nastavil robotu v PyBullet a cez getLinkState som získal jeho polohu. 
Nakoniec som vypísal rozdiel medzi DH výpočtom a PyBullet výpočtom. 
Rozdiel môže vzniknúť kvôli odlišnej definícii frame-ov alebo numerickým nepresnostiam.
'''