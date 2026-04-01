import numpy as np
import matplotlib.pyplot as plt
 
from mpl_toolkits.mplot3d import Axes3D
 
 
def plot_frame(ax, R, o, label, length=0.5):
    x_axis = o + R.dot(np.array([length, 0, 0]))
    y_axis = o + R.dot(np.array([0, length, 0]))
    z_axis = o + R.dot(np.array([0, 0, length]))
    
    ax.quiver(o[0], o[1], o[2], x_axis[0]-o[0], x_axis[1]-o[1], x_axis[2]-o[2],
              color='r', arrow_length_ratio=0.1)
    ax.quiver(o[0], o[1], o[2], y_axis[0]-o[0], y_axis[1]-o[1], y_axis[2]-o[2],
              color='g', arrow_length_ratio=0.1)
    ax.quiver(o[0], o[1], o[2], z_axis[0]-o[0], z_axis[1]-o[1], z_axis[2]-o[2],
              color='b', arrow_length_ratio=0.1)
    
    ax.text(o[0], o[1], o[2], label, fontsize=12, color='k')
    
def Rz(alpha):
    c = np.cos(alpha)
    s = np.sin(alpha)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]])
    
def Ry(Beta):
    c = np.cos(Beta)
    s = np.sin(Beta)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]])
    
def Rx(Gamma):
    c = np.cos(Gamma)
    s = np.sin(Gamma)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s,  c]])
    
def homogeneous_transform(R, o):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = o
    return T
 
def transform_point_homogeneous(T, p):
    p_homog = np.append(p, 1)  # Convert to homogeneous coordinates
    p_transformed = T @ (p_homog)
    return p_transformed[:3]  # Convert back to 3D coordinates
 
if __name__=="__main__":
    R_example = Rz(np.deg2rad(45))
    o_example = np.array([2, 3, 1])
    A_example = homogeneous_transform(R_example, o_example)
    p_local = np.array([1, 0, 0])
    print("Transformed point (homogeneous):", transform_point_homogeneous(A_example, p_local))
    print("Local point:", p_local)
    
        
    fig = plt.figure(figsize=(12,6))
    
    #subplot1
    ax1 = fig.add_subplot(121, projection='3d')
    plot_frame(ax1, np.eye(3), np.array([0,0,0]), "World Frame", length=1)
    plot_frame(ax1, R_example, np.array([0,0,0]), "Rz")
 
 
    ax1.set_title("homogenous transformation")
    ax1.set_title("RZ Rotation")
    ax1.set_xlim(-1,4)
    ax1.set_ylim(-1,4)
    ax1.set_zlim(-1,4)
    ax1.legend()
    
    #subplot2
    ax2 = fig.add_subplot(122, projection='3d')
    plot_frame(ax2, np.eye(3), np.array([1,0,0]), "World Frame", length=1)
    plot_frame(ax2, R_example, o_example, "Transformed frame", length=1)
    #plot_frame(ax2, R_x, np.array([0,0,0]), "Rx(Gamma)", length=1)
    p_world = transform_point_homogeneous(A_example, p_local)
    ax2.scatter(*p_world, color='magenta', s=50, label='p in world frame')
    
    
    
    ax2.set_title("Angle axis representation")
    ax2.set_xlim(-1,4)
    ax2.set_ylim(-1,4)
    ax2.set_zlim(-1,4)
    ax2.legend()
    
    plt.show()