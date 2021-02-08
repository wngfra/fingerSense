# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

import numpy as np
from numpy import linalg as LA

import jax.numpy as jnp
import jax.numpy.linalg as jLA
from jax import jit


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


@jit
def KL_divergence_normal(y, p, q, y0, n):
    '''
        Compute analytical KL-divergence of two multivariate normal distribution

        ...

        Parameters
        ----------
        y : numpy.array
            Input data to compute jacobian
        p : list
            Sampling distribution mean and std
        q : list
            True distribution mean and std of one percept
        y0 : numpy.array
            Old data to be replaced
        n : int
            Length of data

        Returns
        -------
        Result : float
            Computed divergence D_{KL}(p_new||q) = 1/2 * [log(det(\Sigma_q|)/|det(\Sigma_p)) - k + (\mu_p - \mu_q)^T \Sigma_q^{-1} (\mu_p - \mu_q) + trace{\Sigma_q^{-1} \Sigma_p}
    '''
    mu_p, mu_q = jnp.array(p[0]), jnp.array(q[0])
    sigma_p, sigma_q = jnp.array(p[1]), jnp.array(q[1])

    if mu_p.shape != mu_q.shape or sigma_p.shape != sigma_q.shape:
        raise ValueError('dimension mismatch')

    y = jnp.array(y)
    y0 = jnp.array(y0)

    mu_p_new = mu_p + (y - y0)/n
    sigma_p_new = (n-1)/n*sigma_p + jnp.outer(y - mu_p_new, y - mu_p)/n
    k = y.shape[0]

    return 0.5 * (jnp.log(jLA.det(sigma_q)/jLA.det(sigma_p_new)) - k + jnp.dot(jnp.dot((mu_p_new - mu_q).T, jLA.inv(sigma_q)), (mu_p_new - mu_q)) + jnp.trace(jnp.dot(jLA.inv(sigma_q), sigma_p_new)) )


def normalize(x, axis):
    return (x - np.mean(x, axis=axis, keepdims=True)) / np.std(x, axis=axis, keepdims=True)
