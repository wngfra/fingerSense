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


def KL_divergence_normal(p, q):
    '''
        Compute analytical KL-divergence of two multivariate normal distribution

        ...

        Parameters
        ----------
        p : list
            True distribution mean and std
        q : numpy.array
            Anticipated distribution mean and std

        Returns
        -------
        divergence : numpy.array
            Computed D_{KL}(p||q) = 1/2 * [log(det(\Sigma_q|)/|det(\Sigma_p)) - k + (\mu_p - \mu_q)^T \Sigma_q^{-1} (\mu_p - \mu_q) + trace{\Sigma_q^{-1} \Sigma_p}+
    '''
    mu_p, mu_q = p[0], q[0]
    sigma_p, sigma_q = p[1], q[1]

    k = mu_p.shape

    if k != mu_q.shape:
        raise ValueError('dimension mismatch')

    return 0.5 * (
        np.log(LA.det(sigma_q)/LA.det(sigma_p)) - k +
        np.dot(np.dot((mu_p - mu_q).transpose(), LA.inv(sigma_q)), (mu_p - mu_q)) +
        np.trace(np.dot(LA.inv(sigma_q), sigma_p)))


def normalize(x, axis):
    return (x - np.mean(x, axis=axis, keepdims=True)) / np.std(x, axis=axis, keepdims=True)
