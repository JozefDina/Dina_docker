import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def transform_point(R,o,p_local):
    return o + R.dot(p_local)

def plot_frame(ax, R, o, label, length=0.5):
    x_axis = o + R.dot(np.array([length, 0, 0]))
    y_axis = o + R.dot(np.array([0, length, 0]))
    z_axis = o + R.dot(np.array([0, 0, length]))

    ax.quiver(o[0], o[1], o[2], x_axis[0]-o[0], x_axis[1]-o[1], x_axis[2]-o[2], color='r', arrow_length_ratio=0.1)
    ax.quiver(o[0], o[1], o[2], y_axis[0]-o[0], y_axis[1]-o[1], y_axis[2]-o[2], color='g', arrow_length_ratio=0.1)
    ax.quiver(o[0], o[1], o[2], z_axis[0]-o[0], z_axis[1]-o[1], z_axis[2]-o[2], color='b', arrow_length_ratio=0.1)

    ax.text(o[0], o[1], o[2], label, fontsize=12, color='k')

if __name__=="__main__":
    local_R = np.eye(3)
    local_origin = np.array([0, 0, 0])

    world_R = np.eye(3)
    world_origin = np.array([1,2,3])

    p_local = np.array([0.5, -0.5, 1])

    p_world = transform_point(world_R, world_origin, p_local)
    print("Local point:", p_local)
    print("World point:", p_world)

    fig = plt.figure(figsize=(12,6))

    #subplot1
    ax1 = fig.add_subplot(121, projection='3d')
    plot_frame(ax1, local_R, local_origin, "Local Frame", length=1)
    ax1.scatter(p_local[0], p_local[1],p_local[2], color='magenta',s=50,label='p in local frame')
    ax1.set_title("Local Frame an p (local coordinates)")
    ax1.set_xlim(-2,2)
    ax1.set_ylim(-2,2)
    ax1.set_zlim(-2,2)
    ax1.legend()

    #subplot2
    ax2 = fig.add_subplot(122, projection='3d')
    plot_frame(ax2, np.eye(3), np.array([0,0,0]), "Word base", length=1)
    plot_frame(ax2, world_R, world_origin, "Transfore frame", length=1)
    ax2.scatter(p_world[0], p_world[1],p_world[2], color='orange',s=50,label='p in world frame')
    ax2.set_title("World Frame and p (Transformed coordinates)")
    ax2.set_xlim(-1,4)
    ax2.set_ylim(-1,4)
    ax2.set_zlim(-1,4)
    ax2.legend()

    plt.show()