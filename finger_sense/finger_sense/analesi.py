import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from tensorly.tenalg import mode_dot
from tensorly.decomposition import tucker

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from utility import error_ellipsoid, KL_divergence_normal


def knowledge_test():
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


def kl_div_test():
    p = np.random.rand(10000, 100)
    q = np.random.rand(10000, 100)
    kl_div = KL_divergence_normal(p, q)

    return kl_div


def bar_plot_test():
    import matplotlib.pyplot as plt
    plt.rcdefaults()
    import numpy as np
    import matplotlib.pyplot as plt

    p = np.array(10)

    objects = {'Python':10, 'C++':9, 'Java':8, 'Perl':7, 'Scala':6, 'Lisp':2}
    y_pos = np.arange(len(objects))

    plt.bar(y_pos, objects.values(), align='center', alpha=0.5)
    plt.xticks(y_pos, objects.keys())
    plt.ylabel('Usage')
    plt.title('Programming language usage')

    plt.show()


if __name__ == '__main__':
    bar_plot_test()
