import numpy as np

ds = np.loadtxt('data.npy', delimiter=',')

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

print(ds[:, 2])
fig =plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(ds[:, 0], ds[:, 1], ds[:, 2])
plt.show()