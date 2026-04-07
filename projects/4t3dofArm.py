import pybullet as pb
import pybullet_data
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

# theta1, theta2, theta3 = uhly kĺbov
# L1, L2, L3 = dĺžky článkov
# T1, T2, T3 = transformácie jednotlivých článkov
# T_link1, T_link2, T_link3 = výsledné transformácie voči svetu
# link3_position = pozícia konca robota
# @ = maticové násobenie

# Táto funkcia zoberie homogénnu transformačnú maticu T
# a vytiahne z nej:
# 1. pozíciu (translation)
# 2. orientáciu vo forme quaternionu
def decompose_homogenous_matrix(T):
    # pozícia je posledný stĺpec matice bez posledného riadku
    translation = T[:3, 3]

    # rotačná matica je ľavý horný blok 3x3
    rotation_matrix = T[:3, :3]

    # scipy prevedie rotačnú maticu na Eulerove uhly
    euler_angles = R.from_matrix(rotation_matrix).as_euler('xyz')

    # PyBullet chce orientáciu často ako quaternion
    quaterion = pb.getQuaternionFromEuler(euler_angles)

    return translation, quaterion


if __name__ == "__main__":
    # pripojenie do PyBullet GUI
    pb.connect(pb.GUI)

    # nastavenie gravitácie
    pb.setGravity(0, 0, -9.8)

    # pridanie cesty k základným PyBullet súborom
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())

    # načítanie roviny
    plane_id = pb.loadURDF("plane.urdf")

    # načítanie robota z URDF
    # useFixedBase=True znamená, že základňa robota je pevná
    robot_id = pb.loadURDF("urdf/3dof_planar_robot.urdf", basePosition=[0, 0, 0], useFixedBase=True)

    # indexy kĺbov robota
    # tieto indexy závisia od URDF modelu
    joint1_idx = 1
    joint2_idx = 2
    joint3_idx = 3

    # uhly jednotlivých kĺbov
    # tieto hodnoty môžeš meniť a sledovať, ako sa zmení poloha robota
    theta1 = -np.pi / 4
    theta2 = np.pi / 4
    theta3 = np.pi / 18

    # nastavíme robotu cieľové uhly kĺbov cez POSITION_CONTROL
    pb.setJointMotorControl2(robot_id, joint1_idx, controlMode=pb.POSITION_CONTROL, targetPosition=theta1)
    pb.setJointMotorControl2(robot_id, joint2_idx, controlMode=pb.POSITION_CONTROL, targetPosition=theta2)
    pb.setJointMotorControl2(robot_id, joint3_idx, controlMode=pb.POSITION_CONTROL, targetPosition=theta3)

    # dĺžky jednotlivých článkov robota
    L1 = 1
    L2 = 2
    L3 = 3

    # transformácia zo svetového frame do základne robota
    # tu je základňa len mierne posunutá v osi z
    T_W_Base = np.eye(4)
    T_W_Base[2, 3] = 0.05

    # transformačná matica prvého článku
    # obsahuje:
    # - rotáciu podľa theta1
    # - posun o dĺžku L1
    T1 = np.array([
        [ np.cos(theta1), 0, np.sin(theta1), L1 * np.sin(theta1)],
        [              0, 1,              0,                  0],
        [-np.sin(theta1), 0, np.cos(theta1), L1 * np.cos(theta1)],
        [              0, 0,              0,                  1]
    ])

    # transformačná matica druhého článku
    T2 = np.array([
        [ np.cos(theta2), 0, np.sin(theta2), L2 * np.sin(theta2)],
        [              0, 1,              0,                  0],
        [-np.sin(theta2), 0, np.cos(theta2), L2 * np.cos(theta2)],
        [              0, 0,              0,                  1]
    ])

    # transformačná matica tretieho článku
    T3 = np.array([
        [ np.cos(theta3), 0, np.sin(theta3), L3 * np.sin(theta3)],
        [              0, 1,              0,                  0],
        [-np.sin(theta3), 0, np.cos(theta3), L3 * np.cos(theta3)],
        [              0, 0,              0,                  1]
    ])

    # FORWARD KINEMATICS:
    # skladáme transformácie postupne od základne po koniec robota
    T_link1 = T_W_Base @ T1
    T_link2 = T_link1 @ T2
    T_link3 = T_link2 @ T3

    # z transformačných matíc vytiahneme pozície a orientácie
    link1_position, link1_orientation = decompose_homogenous_matrix(T_link1)
    link2_position, link2_orientation = decompose_homogenous_matrix(T_link2)
    link3_position, link3_orientation = decompose_homogenous_matrix(T_link3)

    # výpis pozícií jednotlivých článkov
    print("Link 1 position", link1_position)
    print("Link 2 position", link2_position)
    print("Link 3 position", link3_position)

    # vytvorenie malej gule v pozícii koncového efektora
    # collision shape = fyzikálny tvar
    sphere_colision_shape = pb.createCollisionShape(shapeType=pb.GEOM_SPHERE, radius=0.05)

    # visual shape = vizuálny tvar
    sphere_visual_shape = pb.createVisualShape(
        shapeType=pb.GEOM_SPHERE,
        radius=0.05,
        rgbaColor=[0, 0, 1, 1]
    )

    # vytvorenie objektu v scéne na pozícii konca robota
    pb.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=sphere_colision_shape,
        baseVisualShapeIndex=sphere_visual_shape,
        basePosition=link3_position
    )

    # dĺžka vykreslených osí frame-u
    axis_length = 0.2

    # počet krokov simulácie
    num_steps = 10000

    for t in range(num_steps):
        # vykreslenie osí frame-u pre link1
        # červená = x os
        # zelená = y os
        # modrá = z os
        pb.addUserDebugLine(
            link1_position,
            link1_position + T_link1[:3, 0] * axis_length,
            [1, 0, 0],
            lineWidth=3,
            lifeTime=1.0
        )
        pb.addUserDebugLine(
            link1_position,
            link1_position + T_link1[:3, 1] * axis_length,
            [0, 1, 0],
            lineWidth=3,
            lifeTime=1.0
        )
        pb.addUserDebugLine(
            link1_position,
            link1_position + T_link1[:3, 2] * axis_length,
            [0, 0, 1],
            lineWidth=3,
            lifeTime=1.0
        )

        # vykreslenie osí frame-u pre link2
        pb.addUserDebugLine(
            link2_position,
            link2_position + T_link2[:3, 0] * axis_length,
            [1, 0, 0],
            lineWidth=3,
            lifeTime=1.0
        )
        pb.addUserDebugLine(
            link2_position,
            link2_position + T_link2[:3, 1] * axis_length,
            [0, 1, 0],
            lineWidth=3,
            lifeTime=1.0
        )
        pb.addUserDebugLine(
            link2_position,
            link2_position + T_link2[:3, 2] * axis_length,
            [0, 0, 1],
            lineWidth=3,
            lifeTime=1.0
        )

        # vykreslenie osí frame-u pre link3
        pb.addUserDebugLine(
            link3_position,
            link3_position + T_link3[:3, 0] * axis_length,
            [1, 0, 0],
            lineWidth=3,
            lifeTime=1.0
        )
        pb.addUserDebugLine(
            link3_position,
            link3_position + T_link3[:3, 1] * axis_length,
            [0, 1, 0],
            lineWidth=3,
            lifeTime=1.0
        )
        pb.addUserDebugLine(
            link3_position,
            link3_position + T_link3[:3, 2] * axis_length,
            [0, 0, 1],
            lineWidth=3,
            lifeTime=1.0
        )

        # posun simulácie o jeden krok
        pb.stepSimulation()

        # malé oneskorenie, aby sa simulácia neprehrávala príliš rýchlo
        time.sleep(1 / 240)

    # odpojenie od PyBullet
    pb.disconnect()
