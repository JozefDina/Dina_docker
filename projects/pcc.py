import numpy as np
import matplotlib.pyplot as plt
 
r = 0.01
L0 = 0.2
 
def actuator_to_pcc(L1,L2,L3):
    phi = np.arctan2(np.sqrt(3)*(L2-L3), 2*L1-L2-L3)
    kappa = (2/(3*r))* np.sqrt((L1-L2)**2 + (L2-L3)**2 + (L3-L1)**2)
    L = (L1+L2+L3)/3
    
    return kappa, phi, L
 
def pcc_forward_kinematics(kappa, phi, L, num_points = 50):
    if abs(kappa)< 1e-6:
        s = np.linspace(0, L, num_points)
        x = np.zeros_like(s)
        z = s
    else:
        s = np.linspace(0, L, num_points)
        x = (1/kappa) * (1 - np.cos(kappa*s))
        z = (1/kappa) * (np.sin(kappa*s))
        
    X = x * np.cos(phi)
    Y = x * np.sin(phi)
    Z = z
    return X,Y,Z
 
def plot_robot(L1, L2, L3):
    kappa, phi, L = actuator_to_pcc(L1,L2,L3)
    
    fig = plt.figure()
    ax = fig.add_subplot(projection = '3d')
    X,Y,Z = pcc_forward_kinematics(kappa, phi, L)
    
    ax.plot(X,Y,Z,linewidth = 4)
    ax.scatter(X[-1], Y[-1], Z[-1], color = 'red', label = 'Tip')
    
    ax.set_title(f"L1={L1: .2f}, L2={L2: .2f}, L3={L3: .2f}")
    ax.legend()
    
    plt.show()
    
if __name__ == "__main__":
    
    plot_robot(0.2,0.2,0.2)
    plot_robot(0.18,0.22,0.2)
    plot_robot(0.15,0.25,0.2)
    