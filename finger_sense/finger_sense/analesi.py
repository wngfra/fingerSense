import numpy as np
import matplotlib.pyplot as plt
from tensorly.tenalg import mode_dot
from tensorly.decomposition import tucker

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

ds = np.loadtxt('data.npy', delimiter=',')
data = ds.reshape(-1, 32, 32).transpose([1, 2, 0])

fig = plt.figure()
ax = fig.add_subplot(111)

for i in range(data.shape[0]):
    ax.imshow(data[:, :, i])
    plt.pause(0.25)

core, factors = tucker(data, rank=[3, 1, data.shape[2]])

xs = core[0, 0, :]
ys = core[1, 0, :]
zs = core[2, 0, :]

np.save('core.npy', core.squeeze(), allow_pickle=True)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(xs, ys, zs)
plt.show()