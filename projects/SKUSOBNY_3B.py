import numpy as np
import matplotlib.pyplot as plt

"""
Úloha 3B – Continuum robot (PCC)

Zadanie:
- vygeneruj 5 náhodných kombinácií (L1, L2, L3)
- vykresli 5 rôznych tvarov robota

Použitý je PCC model.
"""

# parametre modelu
r = 0.01   # polomer uloženia aktuátorov od stredu
L0 = 0.2   # približná základná dĺžka

def actuator_to_pcc(L1, L2, L3):
    """
    Prevod z dĺžok aktuátorov na PCC parametre:
    phi   = smer ohybu
    kappa = zakrivenie
    L     = priemerná dĺžka segmentu
    """
    phi = np.arctan2(np.sqrt(3) * (L2 - L3), 2 * L1 - L2 - L3)
    kappa = (2 / (3 * r)) * np.sqrt((L1 - L2)**2 + (L2 - L3)**2 + (L3 - L1)**2)
    L = (L1 + L2 + L3) / 3
    return kappa, phi, L

def pcc_forward_kinematics(kappa, phi, L, num_points=80):
    """
    Vypočíta body continuum robota podľa PCC modelu.
    """
    s = np.linspace(0, L, num_points)

    if abs(kappa) < 1e-6:
        x = np.zeros_like(s)
        z = s
    else:
        x = (1 / kappa) * (1 - np.cos(kappa * s))
        z = (1 / kappa) * np.sin(kappa * s)

    X = x * np.cos(phi)
    Y = x * np.sin(phi)
    Z = z

    return X, Y, Z

# -------------------------------
# Hlavný program
# -------------------------------
np.random.seed(42)   # ak chceš zakaždým iné tvary, tento riadok vymaž

fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(projection='3d')

for i in range(5):
    # náhodné kombinácie dĺžok aktuátorov
    L1 = np.random.uniform(0.15, 0.25)
    L2 = np.random.uniform(0.15, 0.25)
    L3 = np.random.uniform(0.15, 0.25)

    kappa, phi, L = actuator_to_pcc(L1, L2, L3)
    X, Y, Z = pcc_forward_kinematics(kappa, phi, L)

    ax.plot(X, Y, Z, linewidth=3, label=f"Tvar {i+1}")
    ax.scatter(X[-1], Y[-1], Z[-1], s=40)

    print(f"Kombinácia {i+1}:")
    print(f"  L1 = {L1:.4f}, L2 = {L2:.4f}, L3 = {L3:.4f}")
    print(f"  kappa = {kappa:.4f}, phi = {phi:.4f}, L = {L:.4f}")
    print("-" * 40)

ax.set_title("Continuum robot – PCC model")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.legend()
plt.tight_layout()
plt.show()

'''
Použil som PCC model continuum robota. Vygeneroval som 5 náhodných kombinácií 
dĺžok L1, L2, L3. Z týchto hodnôt som vypočítal parametre kappa, phi a L, ktoré určujú 
zakrivenie a smer ohybu robota. Potom som pomocou PCC forward kinematics vypočítal body 
robota a vykreslil 5 rôznych tvarov. 
Každá kombinácia dĺžok vytvorila iný ohnutý tvar.
'''