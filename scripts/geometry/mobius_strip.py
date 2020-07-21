import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib.tri import Triangulation

theta = np.linspace(0, 2 * np.pi, 30)
w = np.linspace(-0.5, 0.5, 8)

w, theta = np.meshgrid(w, theta)

N= int(theta.shape[0]/2)
# M = theta[::2].shape[0]

# phi = 0 * theta
# phi[N:N+M] = 0.5 * theta[::2]
phi = 0.5*theta
# phi = np.linspace(0,0,15) + np.linspace(0, 2 * np.pi, 15)
# phiTwist = np.linspace(1.57, 3.14+1.57, N)

# for k in range(0,theta.shape[0]):
#   if k < N:
#     phi[k,:] = 1.57
#   else:
#     phi[k,:] = phiTwist[k-N]


# radius in x-y plane
r = 1 + w * np.cos(phi)

x = np.ravel(r * np.cos(theta))
y = np.ravel(r * np.sin(theta))
z =  np.ravel(w * np.sin(phi))

# triangulate in the underlying parametrization
tri = Triangulation(np.ravel(w), np.ravel(theta))

ax = plt.axes(projection='3d')
ax.plot_trisurf(x, y, z, triangles=tri.triangles,
                    edgecolor='none', color='grey', linewidths=0,
                    alpha=0.5);

# x_middle = np.ravel(r_middle * np.cos(theta_middle))
# y_middle = np.ravel(r_middle * np.sin(theta_middle))
# z_middle = np.ravel(w_middle * np.sin(phi_middle))
x_middle = np.ravel(np.cos(theta))
y_middle = np.ravel(np.sin(theta))
z_middle = np.ravel(0. * np.sin(phi))
plt.plot(x_middle,y_middle,z_middle,'-m',lw=5)

plt.plot([1,-1],[0,0],[0,0.5],'og', markersize=5)

ax.set_xlim(-1, 1); ax.set_ylim(-1, 1); ax.set_zlim(-1, 1);
plt.show()
