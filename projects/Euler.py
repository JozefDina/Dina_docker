import numpy as np
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D


def plot_frame(ax, R, o, label, length=0.5):
    x_axis = o + R.dot(np.array([length, 0, 0]))
    y_axis = o + R.dot(np.array([0, length, 0]))
    z_axis = o + R.dot(np.array([0, 0, length]))

    ax.quiver(o[0], o[1], o[2], x_axis[0]-o[0], x_axis[1]-o[1], x_axis[2]-o[2], color='r', arrow_length_ratio=0.1)
    ax.quiver(o[0], o[1], o[2], y_axis[0]-o[0], y_axis[1]-o[1], y_axis[2]-o[2], color='g', arrow_length_ratio=0.1)
    ax.quiver(o[0], o[1], o[2], z_axis[0]-o[0], z_axis[1]-o[1], z_axis[2]-o[2], color='b', arrow_length_ratio=0.1)

    ax.text(o[0], o[1], o[2], label, fontsize=12, color='k')

def Rz(alpha):
    c = np.cos(alpha)
    s = np.sin(alpha)
    return np.array([[c, -s, 0],
                    [s, c, 0],
                    [0, 0, 1]])

def Ry(beta):
    c = np.cos(beta)
    s = np.sin(beta)
    return np.array([[c, 0, s],
                    [0, 1, 0],
                    [-s, 0, c]])

def Rx(gamma):
    c = np.cos(gamma)
    s = np.sin(gamma)
    return np.array([[1, 0, 0],
                    [0, c, -s],
                    [0, s, c]])

def rpy_to_rot(roll, pitch, yaw):
    return Rz(roll)@(Ry(pitch))@(Rx(yaw))

def skew(r):
    rx, ry, rz = r
    return np.array([[0, -rz, ry],
                    [rz, 0, -rx],
                    [-ry, rx, 0]])

def axis_angle_rot(theta, r):
    r = np.array(r) / np.linalg.norm(r)
    K = skew(r)
    I = np.eye(3)
    return I + np.sin(theta)*K + (1 - np.cos(theta)) * (K@K)

if __name__ == "__main__":
    theta = np.deg2rad(45)
    r_axis = [1, 1, 0]

    R_axis_angle = axis_angle_rot(theta, r_axis)
    print("Rotation matrix(Rodriguez): \n", R_axis_angle)

    roll = np.deg2rad(30)
    pitch = np.deg2rad(45)
    yaw = np.deg2rad(60)
    R_rpy = rpy_to_rot(roll, pitch, yaw)

    fig = plt.figure(figsize=(12,6))

    #subplot1
    ax1 = fig.add_subplot(121, projection='3d')
    plot_frame(ax1, np.eye(3), np.array([1,0,0]), "World Frame")
    plot_frame(ax1, R_rpy, np.array([0,0,0]), "R_rpy frame")

    ax1.set_title("RPY rotation")
    ax1.set_xlim(-2,2)
    ax1.set_ylim(-2,2)
    ax1.set_zlim(-2,2)
    ax1.legend()

    #subplot2
    ax2 = fig.add_subplot(122, projection='3d')
    plot_frame(ax2, np.eye(3), np.array([1,0,0]), "World Frame")
    plot_frame(ax2, R_axis_angle, np.array([0,0,0]), "Rodriguez frame")

    ax2.set_title("Angle-axis representation")
    ax2.set_xlim(-2,2)
    ax2.set_ylim(-2,2)
    ax2.set_zlim(-2,2)
    # ax2.legend()

    plt.show()