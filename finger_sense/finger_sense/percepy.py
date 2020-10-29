# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

import numpy as np
import tensorly as tl

from numpy.fft import fft
from skfda import FDataGrid
from tensorly.decomposition import tucker
from tensorly.tenalg import mode_dot


def basis_expand(data_matrix, basis):
    fd = FDataGrid(data_matrix.transpose()).to_basis(basis)
    coeffs = fd.coefficients
    coeff_cov = np.cov(coeffs.transpose())
    
    return coeff_cov

def project2vec(A, factors):
    projA1 = mode_dot(A, factors[0].transpose(), 0)
    projA2 = mode_dot(projA1, factors[1].transpose(), 1)
    
    return projA2