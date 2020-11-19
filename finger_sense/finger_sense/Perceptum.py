import numpy as np
import pandas as pd

from numpy import linalg as LA
from skfda import FDataGrid
from skfda.representation.basis import Fourier
from tensorly.tenalg import mode_dot


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
            self.core = np.load(dirs[0], allow_pickle=True).squeeze()
            self.factors = np.load(dirs[1], allow_pickle=True)[0:2]
            info = pd.read_csv(dirs[2], delimiter=',')

            class_names = info['class_name']
            self.percept_classes = {}

            '''
                Fit a Gaussian distribution for each unique class
                Represent the class with mean and covariance
            '''
            for cn in set(class_names):
                data = self.core[:, class_names == cn]
                mean = np.mean(data, axis=1)
                std = np.std(data, axis=1)
                self.percept_classes[cn] = [mean, std]

            self.startIdx = self.core.shape[1]
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
                Input stimulus tensor
        '''
        coeff_cov = self.basis_expand(T)
        
        if self.factors is not None: # With loaded prior knowledge base
            latent = self.compress(coeff_cov).reshape(1, -1)
            self.core = np.hstack((self.core, latent.transpose()))
            self.count += 1

            if self.count - self.startIdx > self.startIdx:
                stack = self.core[:, self.count - self.stack_size:]
                mean, std = np.mean(stack, axis=1), np.std(stack, axis=1)
                kl_div = {}
                for pck in self.percept_classes.keys():
                    kl_div[pck] = KL_divergence_normal((mean, std), self.percept_classes[pck])
                # TODO compute gradient to motion parameters
        
        else: # Without prior, training mode
            # TODO Training mode append new data for HOOI
            pass
