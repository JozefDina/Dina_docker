import numpy as np
import matplotlib.pyplot as plt
 
 
# ============================================================
# Robot parameters
# ============================================================
 
RADIUS = 0.01  # tendon radius from backbone [m]
 
 
# ============================================================
# Actuator space -> PCC parameters
# ============================================================
 
def actuator_to_pcc(L1: float, L2: float, L3: float, r: float = RADIUS):
    """
    Map tendon lengths to PCC parameters q = [kappa, phi, L].
    """
    phi = np.arctan2(np.sqrt(3.0) * (L2 - L3), 2.0 * L1 - L2 - L3)
 
    kappa = (2.0 / (3.0 * r)) * np.sqrt(
        (L1 - L2) ** 2 + (L2 - L3) ** 2 + (L3 - L1) ** 2
    )
 
    L = (L1 + L2 + L3) / 3.0
    return np.array([kappa, phi, L], dtype=float)
 
 
# ============================================================
# PCC forward kinematics
# ============================================================
 
def pcc_forward_kinematics(
    kappa: float,
    phi: float,
    L: float,
    num_points: int = 80
) :
    """
    Compute points along a single-segment PCC backbone.
 
    Returns:
        np.ndarray of shape (num_points, 3)
    """
    # s = np.linspace(0.0, L, num_points)
 
    if abs(kappa) < 1e-8:
        s = np.linspace(0.0, L, num_points)
        x = np.zeros_like(s)
        z = s
    else:
        s = np.linspace(0.0, L, num_points)
        x = (1.0 / kappa) * (1.0 - np.cos(kappa * s))
        z = (1.0 / kappa) * np.sin(kappa * s)
 
    X = x * np.cos(phi)
    Y = x * np.sin(phi)
    Z = z
 
    return np.column_stack((X, Y, Z))
 
 
def actuator_tip_position(Ls: np.ndarray) -> np.ndarray:
    """
    Tip position directly from actuator lengths using PCC forward kinematics.
    """
    kappa, phi, L = actuator_to_pcc(*Ls)
    curve = pcc_forward_kinematics(kappa, phi, L)
    return curve[-1]
 
 
# ============================================================
# Numerical actuator-space Jacobian
# ============================================================
 
def actuator_jacobian(Ls, eps = 1e-6):
    # J = dx/dL
    # x = [X, Y, Z]
    # L = [L1, L2, L3]
    J = np.zeros((3,3))

    for i in range(3):
        dL = np.zeros(3)
        dL[i] = eps

        fp = actuator_tip_position(Ls + dL)
        fm = actuator_tip_position(Ls - dL)

        J[:, i] = (fp - fm)/(2.0 * eps)
    return J

# ============================================================
# Iterative IK in actuator space
# ============================================================
 
def inverse_kinematics_actuator_space(target, L_init, max_iters = 100, tol = 1e-4, alpha = 0.3):
    Ls = np.array(L_init)
    history = []
    errors = []

    for _ in range(max_iters):
        pos = actuator_tip_position(Ls)
        err = target - pos

        history.append(pos.copy())
        errors.append(err.copy())

        if np.linalg.norm(err) < tol:
            break

        J = actuator_jacobian(Ls)
        dL = alpha * (np.linalg.pinv(J) @ err)
        Ls += dL
    return Ls, np.array(history), np.array(errors)
 
 
# ============================================================
# Visualization
# ============================================================
 
def plot_result(target: np.ndarray, Ls: np.ndarray, history: np.ndarray):
    """
    Plot final robot shape, target, and tip trajectory.
    """
    kappa, phi, L = actuator_to_pcc(*Ls)
    curve = pcc_forward_kinematics(kappa, phi, L)
    tip = curve[-1]
 
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(projection="3d")
 
    ax.plot(
        curve[:, 0],
        curve[:, 1],
        curve[:, 2],
        linewidth=4,
        label="Continuum robot"
    )
 
    ax.scatter(
        target[0],
        target[1],
        target[2],
        color="red",
        s=60,
        label="Target"
    )
 
    ax.scatter(
        tip[0],
        tip[1],
        tip[2],
        color="green",
        s=60,
        label="Final tip"
    )
 
    if len(history) > 0:
        ax.plot(
            history[:, 0],
            history[:, 1],
            history[:, 2],
            "--",
            linewidth=2,
            label="Tip path"
        )
 
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title("Continuum Robot IK in Actuator Space")
    ax.legend()
    plt.tight_layout()
    plt.show()
 
 
def plot_error(errors: np.ndarray):
    """
    Plot convergence of IK error.
    """
    plt.figure(figsize=(6, 4))
    plt.plot(errors, linewidth=2)
    plt.xlabel("Iteration")
    plt.ylabel("Position error norm [m]")
    plt.title("IK Convergence")
    plt.grid(True)
    plt.tight_layout()
    plt.show()
 
 
# ============================================================
# Main
# ============================================================
 
if __name__ == "__main__":
    # Initial actuator lengths
    L_init = np.array([0.20, 0.20, 0.20], dtype=float)
 
    # Reachable target
    target = np.array([0.03, 0.02, 0.18], dtype=float)
 
    # Solve IK
    L_sol, history, errors = inverse_kinematics_actuator_space(
        target=target,
        L_init=L_init,
        max_iters=100,
        tol=1e-4,
        alpha=0.4
    )
 
    final_tip = actuator_tip_position(L_sol)
    J_final = actuator_jacobian(L_sol)
 
    print("Final actuator lengths [L1, L2, L3]:")
    print(np.round(L_sol, 6))
 
    print("\nFinal tip position [X, Y, Z]:")
    print(np.round(final_tip, 6))
 
    print("\nTarget position [X, Y, Z]:")
    print(np.round(target, 6))
 
    print("\nFinal error norm:")
    print(np.linalg.norm(target - final_tip))
 
    print("\nFinal actuator-space Jacobian:")
    print(J_final)
 
    plot_result(target, L_sol, history)
    plot_error(errors)