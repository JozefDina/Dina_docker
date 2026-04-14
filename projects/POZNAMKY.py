"""
========================================================
 PREHĽAD PODĽA TYPU ROBOTA
========================================================

PRÍKLADY:

1. KUKA ide do bodu [0.4, 0.2, 0.6]
-> dls_method.py
-> zmením target_position

2. 2DOF ide zo start do end
-> analytická trajektória 2DOF
-> zmením start_2d, end_2d, num_points

3. KUKA robí kružnicu
-> IK_diff2.py
-> zmením radius

4. 2DOF workspace
-> 5tworkspace.py
-> zmením theta1_range, theta2_range

5. DH vs PyBullet
-> 5t2dofDH.py
-> zmením joint_angles

6. PCC
-> pcc.py
-> zmením L1, L2, L3
//////////////////////////////////////////////////////////////
CIEĽ:
- rýchlo zistiť, ktorý kód použiť
- vedieť čo zmeniť
- rozlíšiť KUKA vs 2DOF

--------------------------------------------------------
1. TRAJEKTÓRIA MEDZI BODMI (lineárna interpolácia)
--------------------------------------------------------

Typ úlohy:
- trajektória v task space
- start -> end
- pre každý bod IK
- vykresli trajektóriu

Keď je KUKA robot:
- otvor: IK_diff.py
- zmeň:
    start = [...]
    end = [...]
    num_steps = ...
- princíp:
    positions = np.linspace(start, end, num_steps)
    pre každý bod vypočítam IK
- výstup:
    červené body = desired trajektória
    modré body = actual trajektória

Keď je 2DOF robot:
- použi 2DOF analytickú IK verziu trajektórie
- body zadávaj ako:
    [x, 0, z]
- zmeň:
    start_2d = [x, z]
    end_2d = [x, z]
    num_points = ...
- princíp:
    lineárna interpolácia
    pre každý bod analytická IK
    nastavím jointy
- pozor:
    2DOF ide v rovine XZ, nie XY


--------------------------------------------------------
2. ROBOT MÁ ÍSŤ DO BODU
--------------------------------------------------------

Typ úlohy:
- inverse kinematics
- target position
- error = target - current

Keď je KUKA robot:
- otvor: dls_method.py
- zmeň:
    target_position = [x, y, z]
- prípadne:
    alpha
    damping
- výstup:
    Target
    Achieved
    Final error

Keď je 2DOF robot:
- najlepšie analytická IK
- ak chcú numerickú, použi Jacobian/pseudoinverziu pre 2DOF
- zmeň:
    target = [x, z]
- pozor:
    2DOF riešiš v rovine XZ


--------------------------------------------------------
3. KRUŽNICA / PLYNULÝ POHYB / VELOCITY
--------------------------------------------------------

Typ úlohy:
- differential IK
- desired velocity
- dq = J_pinv @ desired_velocity

Keď je KUKA robot:
- otvor: IK_diff2.py
- zmeň:
    radius = ...
    prípadne vx, vy, vz
- kružnica:
    vx = -radius * np.sin(t)
    vy =  radius * np.cos(t)
    vz = 0.0
- keď chceš pohyb hore:
    vz = 0.05

Keď je 2DOF robot:
- diff IK by sa dal spraviť tiež, ale v našich materiáloch je dôležitejší KUKA
- pri 2DOF sa skôr čaká:
    trajektória cez body
    analytická IK
- ak by predsa chcel velocity:
    treba 2DOF Jacobian


--------------------------------------------------------
4. WORKSPACE
--------------------------------------------------------

Typ úlohy:
- všetky body, kam robot dosiahne
- skúšanie všetkých kombinácií uhlov

Keď je 2DOF robot:
- otvor: 5tworkspace.py
- zmeň:
    theta1_range = np.linspace(...)
    theta2_range = np.linspace(...)
- princíp:
    for theta1:
        for theta2:
            FK -> bod
- výstup:
    mrak bodov

Keď je KUKA robot:
- toto ste priamo nerobili ako hlavný template
- na zápočet skôr čakám workspace pre 2DOF


--------------------------------------------------------
5. DH vs PYBULLET
--------------------------------------------------------

Typ úlohy:
- forward kinematics cez DH
- porovnanie so simuláciou

Keď je 2DOF robot:
- otvor: 5t2dofDH.py
- zmeň:
    joint_angles = [...]
- princíp:
    T = T @ T_i
    fk_position_dh = T[:3, 3]
    fk_position_pybullet = getLinkState(...)
- výstup:
    Pozícia koncového efektora (DH)
    Pozícia koncového efektora (PyBullet)
    Rozdiel pozícií
    Veľkosť chyby

Keď je KUKA / Panda / 7DOF robot:
- otvor: 5t7dofDH.py
- zmeň:
    joint_angles = [...]
- princíp rovnaký
- iba viac jointov


--------------------------------------------------------
6. FORWARD KINEMATICS (ručné matice)
--------------------------------------------------------

Typ úlohy:
- zadané uhly
- vypočítaj pozíciu konca robota
- zobraz frame-y

Keď je 3DOF robot:
- otvor: 4t3dofArm.py
- zmeň:
    theta1
    theta2
    theta3
    prípadne L1, L2, L3
- princíp:
    T_link1 = T_base @ T1
    T_link2 = T_link1 @ T2
    T_link3 = T_link2 @ T3
- výsledok:
    Link 3 position

Keď je 2DOF robot:
- podobná logika ako 4t2dofArm.py
- len menší počet matíc


--------------------------------------------------------
7. ANALYTICKÁ IK vs NUMERICKÁ IK
--------------------------------------------------------

Typ úlohy:
- porovnanie dvoch metód
- chyba a počet iterácií

Keď je 2DOF robot:
- použi 2DOF compare kód
- analytická IK:
    priamy výpočet theta1, theta2
- numerická IK:
    pseudoinverzia Jacobianu
- výstup:
    graf chyby
    graf počtu iterácií

Keď je KUKA robot:
- analytická IK nie je taká jednoduchá
- preto je lepší 2DOF robot
- na takúto úlohu určite vyber 2DOF


--------------------------------------------------------
8. CONTINUUM ROBOT / PCC
--------------------------------------------------------

Typ úlohy:
- continuum robot
- PCC model
- L1, L2, L3
- vykresli rôzne tvary

Keď je PCC / continuum robot:
- otvor: pcc.py
- prípadne pokročilejšie: IK_pcc.py
- zmeň:
    náhodné L1, L2, L3
- princíp:
    z L1, L2, L3 vypočítam:
        kappa
        phi
        L
    potom forward kinematics
    potom vykreslenie tvaru

Keď je KUKA alebo 2DOF:
- PCC sa nepoužíva


========================================================
RÝCHLE ROZHODNUTIE NA ZÁPOČTE
========================================================

Keď vidím:
- "choď do bodu"
    -> KUKA: dls_method.py
    -> 2DOF: analytická IK alebo 2DOF numerická IK

- "trajektória medzi bodmi"
    -> KUKA: IK_diff.py
    -> 2DOF: analytická trajektória cez body

- "kružnica / velocity / plynulý pohyb"
    -> KUKA: IK_diff2.py

- "workspace"
    -> 2DOF: 5tworkspace.py

- "DH vs PyBullet"
    -> 2DOF: 5t2dofDH.py
    -> 7DOF: 5t7dofDH.py

- "vypočítaj pozíciu z uhlov"
    -> 4t3dofArm.py alebo 4t2dofArm.py

- "continuum / PCC / L1 L2 L3"
    -> pcc.py alebo IK_pcc.py


========================================================
ČO NAJČASTEJŠIE MENÍM
========================================================

KUKA:
- target_position
- start, end
- num_steps
- radius
- vx, vy, vz

2DOF:
- start_2d, end_2d
- num_points
- theta1_range, theta2_range
- joint_angles
- target [x, z]

DH:
- joint_angles

PCC:
- L1, L2, L3


========================================================
NAJDÔLEŽITEJŠIE VETY
========================================================

FK:
- mám uhly -> chcem pozíciu

IK:
- mám pozíciu -> chcem uhly

diff IK:
- mám rýchlosť -> chcem rýchlosti kĺbov

workspace:
- skúšam všetky kombinácie uhlov

DH:
- skladám transformačné matice z tabuľky

PCC:
- z L1, L2, L3 získam tvar robota
"""