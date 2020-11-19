# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

import numpy as np
from numpy import linalg as LA


def error_ellipsoid(A, scaling_factor=1.0):
    '''
        compute an error ellipsoid
    '''
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