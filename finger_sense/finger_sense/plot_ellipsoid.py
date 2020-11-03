import matplotlib.pyplot as plt
import numpy as np

from mpl_toolkits.mplot3d import Axes3D
from numpy import linalg as LA
from scipy.spatial.transform import Rotation as R

A = np.loadtxt('data.npy', delimiter=',', usecols=(0, 1, 2)).transpose()
# your ellispsoid and center in matrix form
center = np.mean(A, axis=1)

U, W, _ = LA.svd(A)

fig = plt.figure(figsize=plt.figaspect(1))  # Square figure
ax = fig.add_subplot(111, projection='3d')

ax.scatter(A[0, :], A[1, :], A[2, :], color='r')


radii = 0.5 * np.sqrt(W)

# now carry on with EOL's answer
u = np.linspace(0.0, 2.0 * np.pi, 100)
v = np.linspace(0.0, np.pi, 100)
x = radii[0] * np.outer(np.cos(u), np.sin(v))
y = radii[1] * np.outer(np.sin(u), np.sin(v))
z = radii[2] * np.outer(np.ones_like(u), np.cos(v))
surf = np.kron(U[:, 0], x) + np.kron(U[:, 1], y) + np.kron(U[:, 0], z)
xs = surf[:, 0:100] + center[0]
ys = surf[:, 100:200] + center[1]
zs = surf[:, 200:300] + center[2]
# plot
ax.plot_wireframe(xs, ys, zs,  rstride=4, cstride=4, color='b', alpha=0.2)
plt.show()
