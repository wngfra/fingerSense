import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from utility import error_ellipsoid

core = np.load('core.npy', allow_pickle=True).squeeze()
class_ids = pd.read_csv('info.csv', delimiter=',')['class_id']

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for cid in set(class_ids):
    data = core[:, cid == class_ids]
    print(data)
    x, y, z, center, U, W = error_ellipsoid(data)

ax.scatter(core[0, :], core[1, :], core[2, :])

plt.show()