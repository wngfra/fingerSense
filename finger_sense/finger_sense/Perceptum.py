import jax
import jax.numpy as jnp
import numpy as np
import pandas as pd

from numpy import linalg as LA
from skfda import FDataGrid
from skfda.representation.basis import Fourier
from tensorly.tenalg import mode_dot

from finger_sense.utility import KL_divergence_normal, normalize

class Perceptum:

    def __init__(self, dirs, n_basis, stack_size, model_name='Gaussian'):
        self.model_name = model_name  # default gaussian model
        self.basis = Fourier([0, 2 * np.pi], n_basis=n_basis, period=1)
        self.stack_size = stack_size
        self.init_model(dirs)

    def init_model(self, dirs):
        '''
            Load prior knowldge and initialize the perception model
            ...

            Parameters
            ----------
            dirs : list of strings
                Directories of core, factors, info files
        '''
        if dirs is not None:
            self.core = np.load(dirs[0], allow_pickle=True).squeeze() # in shape (latent_dim, data_size)
            self.factors = np.load(dirs[1], allow_pickle=True)[0:2]
            self.percept_classes = {}

            info = pd.read_csv(dirs[2], delimiter=',')
            class_names = info['class_name']

            '''
                Fit a Gaussian distribution for each unique class
                Represent the class with mean and covariance
            '''
            for cn in set(class_names):
                data = self.core[:, class_names == cn]
                mean = np.mean(data, axis=1)
                std = np.std(data, axis=1)
                self.percept_classes[cn] = [mean, std]

            self.startIdx = self.core.shape[1] # Set starting index to skip training data
        else:
            self.core = None
            self.factors = None
            self.info = None
            self.percept_classes = None
            self.startIdx = 0

        self.count = self.startIdx

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

        # FIXME: dimension error to fix
        fd = FDataGrid(normalized_data.transpose()).to_basis(self.basis)
        coeffs = fd.coefficients
        coeff_cov = np.cov(coeffs[:, 1:].transpose())

        return coeff_cov

    def compress(self, A):
        '''
            Project tensor to lower rank matrices
            Factor matrices are obtained from tucker decomposition

            ...

            Parameters
            ----------
            A : numpy.array
                Input tensor
            factors : list of numpy.array
                Factors matrices

            Returns
            -------
            A : numpy.array
                Projected tensor
        '''
        if self.factors is not None:
            for i in range(len(self.factors)):
                A = mode_dot(A, self.factors[i].transpose(), i)

            return A
        else:
            return None

    def perceive(self, T, mode=None):
        '''
            Perceive and process stimulus

            ...

            Parameters
            ----------
            T : numpy.array
                Input stimulus matrix in shape (self.stack_size, channel_size)
        '''
        if self.factors is not None:  # With loaded prior knowledge base
            coeff_cov = self.basis_expand(T)
            latent = self.compress(coeff_cov).reshape(1, -1)

            if self.count - self.startIdx > self.stack_size:  # Start perception only when a new stack is filled
                # Slice of last self.stack_size elements
                stack = self.core[:, self.count - self.stack_size:]
                p = np.mean(stack, axis=1), np.std(stack, axis=1)
                y0 = self.core[:, self.count - self.stack_size]

                for i, key in enumerate(self.percept_classes.keys()):
                    q = self.percept_classes[key]
                    KL_div = KL_divergence_normal(p, q, y0, self.stack_size)
                    KL_jacobian = jax.jacfwd(KL_div)(latent)
                    # TODO: compute gradient w.r.t. control parameters

            # Append new latent vector to the core
            self.core = np.hstack((self.core, latent))
            self.count += 1

        else:  # Without prior, training mode
            # TODO Training mode append new data for HOOI
            pass
