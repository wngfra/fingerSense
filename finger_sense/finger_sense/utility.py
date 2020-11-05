# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

import numpy as np
import tensorly as tl

from numpy import linalg as LA
from skfda import FDataGrid
from tensorly.decomposition import tucker
from tensorly.tenalg import mode_dot


def basis_expand(data_matrix, basis):
    fd = FDataGrid(data_matrix.transpose()).to_basis(basis)
    coeffs = fd.coefficients
    coeff_cov = np.cov(coeffs[:, 1:].transpose())

    return coeff_cov

def fourier_transform(data_matrix):
    L = data_matrix.shape[0]
    F = np.fft.fft(data_matrix, axis=0)
    Y = np.abs(F/L)
    Ys = Y[1:L//2+1, :]
    cov = np.cov(Ys)

    return cov


def project2vec(A, factors):
    projA1 = mode_dot(A, factors[0].transpose(), 0)
    projA2 = mode_dot(projA1, factors[1].transpose(), 1)

    return projA2


def error_ellipsoid(A, scaling_factor=1.0):
    # coloumnwise data points
    center = np.mean(A, axis=1)

    U, W, _ = LA.svd(A)

    radii = scaling_factor * np.sqrt(W)

    # generate original ellipsoid
    u = np.linspace(0.0, 2.0 * np.pi, 100)
    v = np.linspace(0.0, np.pi, 100)
    x = radii[0] * np.outer(np.cos(u), np.sin(v))
    y = radii[1] * np.outer(np.sin(u), np.sin(v))
    z = radii[2] * np.outer(np.ones_like(u), np.cos(v))

    # rotate and translate to the data centroid
    points = np.kron(U[:, 0], x) + np.kron(U[:, 1], y) + np.kron(U[:, 0], z)
    xs = points[:, 0:100] + center[0]
    ys = points[:, 100:200] + center[1]
    zs = points[:, 200:300] + center[2]

    return xs, ys, zs, center, U, W
