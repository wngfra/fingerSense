# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

import numpy as np
from numpy.fft import fft

def fourier_cov(x):
    y = fft(x, axis=0) / x.shape[0]
    ys = y[:y.shape[0]//2, :]
    ys_real = np.concatenate([np.real(ys), np.imag(ys)], axis=1)

    cov_ys_real = np.cov(ys_real)

    return cov_ys_real