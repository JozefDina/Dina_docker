import numpy as np
import matplotlib.pyplot as plt

"""
Úloha 3 – Continuum robot (PCC)

- vygeneruj 5 náhodných kombinácií (L1, L2, L3)
- vykresli 5 rôznych tvarov robota
"""

# PCC model
def pcc_shape(L1, L2, L3, num_points=50):

    # jednoduchý model zakrivenia
    L = (L1 + L2 + L3) / 3

    kappa = (L1 - L3) * 2      # zakrivenie
    phi = (L2 - L1) * 2        # smer

    s = np.linspace(0, L, num_points)

    if abs(kappa) < 1e-6:
        x = np.zeros_like(s)
        z = s
    else:
        x = (1 / kappa) * (1 - np.cos(kappa * s))
        z = (1 / kappa) * np.sin(kappa * s)

    return x, z


# hlavný program
plt.figure()

for i in range(5):
    # náhodné hodnoty
    L1 = np.random.uniform(0.8, 1.2)
    L2 = np.random.uniform(0.8, 1.2)
    L3 = np.random.uniform(0.8, 1.2)

    x, z = pcc_shape(L1, L2, L3)

    plt.plot(x, z, label=f"Set {i+1}")

plt.title("Continuum robot – PCC")
plt.xlabel("X")
plt.ylabel("Z")
plt.grid(True)
plt.legend()

plt.show()

'''
Použil som PCC model pre continuum robot. Vygeneroval som 5 náhodných kombinácií 
parametrov L1, L2 a L3. Pre každú kombináciu som vypočítal zakrivenie robota a jeho tvar. 
Následne som tieto tvary vykreslil do grafu. 
Každá kombinácia vytvorila iný ohnutý tvar robota.
'''