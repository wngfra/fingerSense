# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

import numpy as np
import tensorly as tl

from mayavi import mlab
from numpy.fft import fft
from skfda import FDataGrid
from skfda.representation import basis
from tensorly.decomposition import tucker
from tensorly.tenalg import mode_dot


def fourier_cov(x):
    # TODO: Normalize the sample first
    y = fft(x, axis=0) / x.shape[0]
    ys = y[:y.shape[0]//2, :]
    ys_real = np.concatenate([np.real(ys), np.imag(ys)], axis=1)

    cov_ys_real = np.cov(ys_real)

    return cov_ys_real


def main(args=None):
    pass


if __name__ == "__main__":
    x = np.ones((33, 16, 100))

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    core = np.load('core.npy', allow_pickle=True)
    factors = np.load('factors.npy', allow_pickle=True)
    projected = mode_dot(x, factors[0].transpose(), 0)
    projected = mode_dot(projected, factors[1].transpose(), 1)
    v = np.squeeze(projected)
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(v[0,:], v[1,:], v[2,:])
    plt.show()