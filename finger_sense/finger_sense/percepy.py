# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

import numpy as np
import tensorly as tl

from numpy.fft import fft
from skfda import FDataGrid
from tensorly.decomposition import tucker
from tensorly.tenalg import mode_dot


def fourier_cov(x):
    # TODO: Normalize the sample first
    y = fft(x, axis=0) / x.shape[0]
    ys = y[:y.shape[0]//2, :]
    ys_real = np.concatenate([np.real(ys), np.imag(ys)], axis=1)

    cov_ys_real = np.cov(ys_real)

    return cov_ys_real