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

def homogenous_transform(R, o):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, :3] = o
    return T
def transform_point_homogenous(T, p):
    p_homog = np.append(p,1)
    p_transformed = T @(p_homog)
    return p_transformed[:3]

if __name__ == "__main__":
    R_example = Rz(np.deg2rad(45))
    o_example = np.array([2, 3, 1])
    A_example = homogenous_transform(R_example, o_example)
    p_local = np.array([1, 0, 0])

    print("Transformed point", transform_point_homogenous(A_example, p_local))
    print("Local point", p_local)

    fig = plt.figure()

    ax= fig.add_subplot(111, projection = '3d')
    plot_frame(ax, np.eye(3), np.array([0, 0, 0]), "World frame")
    plot_frame(ax, R_example, o_example, "Transformed frame")
    p_world = transform_point_homogenous(A_example, p_local)
    ax.scatter(*p_world, color = 'orange', label = 'Transformed point')

    ax.set_title("Homogenous transformation")

    ax.set_xlim(-1,4)
    ax.set_zlim(-1,4)
    ax.set_ylim(-1,4)
    ax.legend()

    plt.show()
