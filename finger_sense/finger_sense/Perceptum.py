from collections import deque

import jax
import numpy as np
import pandas as pd

from skfda import FDataGrid
from skfda.representation.basis import Fourier
from tensorly.decomposition import tucker
from tensorly.tenalg import mode_dot

from finger_sense.utility import KL_divergence_normal, normalize


class Perceptum:

    def __init__(self, dirs, latent_dim, n_basis, model_name='Gaussian'):
        self.latent_dim = latent_dim
        self.model_name = model_name

        self.basis = Fourier([0, 2 * np.pi], n_basis=n_basis, period=1)
        self.jacobian = jax.jacfwd(KL_divergence_normal, 0)
        self.sensory_memory = deque(maxlen=2*n_basis-1)

        self.init_model(dirs)

    def init_model(self, dirs):
        '''
            Load prior knowldge and initialize the perception model
            ...

            Parameters
            ----------
            dirs : list of strings
                Directories of core, factors
        '''
        if None in dirs.values():
            self.core = None
            self.factors = None
            self.features = None
        else:
            self.core = pd.read_csv(dirs['core_dir'])
            self.factors = np.load(dirs['factor_dir'], allow_pickle=True)[0:2]

            self.features = {}
            class_names = pd.unique(self.core['material'])

            '''
                Fit a Gaussian distribution for each unique class
                Represent the class with mean and covariance
            '''
            for cn in set(class_names):
                data = self.core[:, class_names == cn]
                mean = np.mean(data, axis=1, keepdims=True)
                cov = np.cov(data)
                self.features[cn] = [mean, cov]

    def basis_expand(self, data_matrix):
        '''
            FDA basis expansion

            ...

            Parameters
            ----------
            data_matrix : numpy.array
                Input matrix with samples stacked in rows

            Returns
            -------
            coeff_cov : numpy.array
                Coefficients of functional basis representation
        '''
        normalized_data = normalize(data_matrix, axis=1)

        fd = FDataGrid(normalized_data.transpose()).to_basis(self.basis)
        coeffs = fd.coefficients.astype(np.float32).squeeze()
        coeff_cov = np.cov(coeffs[:, 1:].transpose())

        return coeff_cov

    def compress(self, T):
        '''
            Project tensor to lower rank matrices
            Factor matrices are obtained from tucker decomposition

            ...

            Parameters
            ----------
            T : numpy.array
                Input tensor
            factors : list of numpy.array
                Factors matrices

            Returns
            -------
            T : numpy.array
                Projected tensor
        '''
        if self.factors is not None:
            for i in range(len(self.factors) - 1):
                T = mode_dot(T, self.factors[i].T, i)

            return T
        else:
            return None

    def perceive(self, M):
        '''
            Perceive and process stimulus

            ...

            Parameters
            ----------
            M : numpy.array
                Input stimulus matrix in shape (self.stack_size, channel_size)

            Returns
            -------
            gradients : numpy.array
                Gradients with respect to the input signals
            weights : numpy.array
                Weights of graidents to update control parameter
            delta_latent : numpy.array
                Difference between current and last latent vectors
        '''
        coeff_cov = self.basis_expand(M)
        # latent = self.compress(coeff_cov)
        latent = np.random.rand(3)

        if len(self.sensory_memory) >= self.sensory_memory.maxlen:
            gradients = np.zeros((len(self.features), self.latent_dim))
            weights = np.zeros((len(self.features), 1))
            delta_latent = latent - self.sensory_memory[-1]

            # Get stats of the sensory memory
            stack = np.array(self.sensory_memory)
            p = np.mean(stack, axis=0, keepdims=True), np.cov(stack)
            y0 = self.sensory_memory.pop()
            self.sensory_memory.append(latent)

            divergences = np.zeros(len(self.features))
            # Compute gradients and weights for each percept_class
            for i, key in enumerate(self.features.keys()):
                q = self.features[key]
                divergences[i] = KL_divergence_normal(
                    latent, p, q, y0, self.stack_size)
                gradients[i, :] = self.jacobian(
                    latent, p, q, y0, self.stack_size).reshape(1, -1)
            exp = np.exp(-divergences)
            weights = np.reshape(exp / np.sum(exp), (-1, 1))

        return gradients, weights, delta_latent

    def update(self, x):
        gradients, weights, delta_latent = self.perceive(x)
        new_control = np.sum(weights * np.matmul(gradients, np.outer(delta_latent, 1 / (
            self.current_control_params - self.prev_control_params))), axis=0,)
