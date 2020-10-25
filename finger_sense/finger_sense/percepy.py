# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

import numpy as np
from numpy.fft import fft

from mayavi import mlab
from tensorly.decomposition import tucker


def fourier_cov(x):
    y = fft(x, axis=0) / x.shape[0]
    ys = y[:y.shape[0]//2, :]
    ys_real = np.concatenate([np.real(ys), np.imag(ys)], axis=1)

    cov_ys_real = np.cov(ys_real)

    return cov_ys_real


def main(args=None):
    pass


if __name__ == "__main__":
    x, y = np.mgrid[0:64:1, 0:64:1]

    X = np.random.rand(64, 64, 100)
    s = mlab.surf(x, y, X[:, :, 0])

    @mlab.animate
    def anim():
        for i in range(100):
            s.mlab_source.scalars = X[:, :, i]
            yield

    anim()
    mlab.show()
