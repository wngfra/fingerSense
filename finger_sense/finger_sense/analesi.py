import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from tensorly.tenalg import mode_dot
from tensorly.decomposition import tucker

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from utility import error_ellipsoid

core = np.load('core.npy', allow_pickle=True)
info = pd.read_csv('info.csv', delimiter=',')
cids = info['class_id']

fig = plt.figure()

for i, cid in enumerate(set(cids)):
    data = core[:, :, cid == cids].squeeze()
    x, y, z, center, _, W = error_ellipsoid(data, 0.1)
    ax = fig.add_subplot(1, len(set(cids)), i+1, projection='3d')
    ax.scatter(data[0, :], data[1, :], data[2, :])
    ax.plot_surface(x, y, z, alpha=0.2)
    print(center, W)

plt.show()
