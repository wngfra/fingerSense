import os
import re

import numpy as np
import pandas as pd
import torch

from numpy.fft import fft
from skfda import FDataGrid
from skfda.representation import basis
# from skfda.representation.interpolation import SplineInterpolator
from torch.utils.data import Dataset

class TacDataset(Dataset):

    def __init__(self, root_dir, transform=None):
        self.root_dir = root_dir
        self.transform = transform

        for _, _, filenames in os.walk(self.root_dir):
            self.filenames = list(filter(lambda x: '.csv' in x, filenames))

        self.class_names = [""] * len(self.filenames)
        self.params = np.zeros((len(self.filenames), 2))

        for i, filename in enumerate(self.filenames):
            basename = filename.split('.')[0]
            namegroup = basename.split('_')
            
            pressure = re.search(r'\d+', namegroup[1]).group(0)
            speed = re.search(r'[-+]?\d+', namegroup[2]).group(0)

            self.class_names[i] = namegroup[0]
            self.params[i, :] = [float(pressure), float(speed)]

        classmap = dict([(class_name, i+1) for i, class_name in enumerate(set(self.class_names))])
        self.class_ids = [classmap[class_name] for class_name in self.class_names]

    def __len__(self):
        return len(self.filenames)

    def __getitem__(self, idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()

        # Read data file
        filename = self.filenames[idx]
        fn = os.path.join(self.root_dir, filename)
        df = pd.read_csv(fn, dtype=np.float32)

        # Get sample and convert to frequency domain
        sample = df.iloc[:, 3:].values

        if self.transform:
            sample = self.transform(sample)

        return sample, self.class_ids[idx]

    def get_class_names(self):
        return self.class_names

    def get_params(self):
        return self.params

    def get_class_ids(self):
        return self.class_ids


class ToFDA(object):
    """Functional data analysis
       Apply basis and expansion and return the coefficient matrix.
    """

    def __init__(self, basis='Fourier', **kwargs):
        self.basis = basis
        try:
            self.n_basis = kwargs['n_basis']
        except KeyError:
            self.n_basis = 11
        try:
            self.order = kwargs['order']
        except KeyError:
            self.order = 3
        try:
            self.period = kwargs['period']
        except KeyError:
            self.period = 1
        try:
            self.transpose = kwargs['transpose']
        except KeyError:
            self.transpose = True

    def __call__(self, sample):
        if self.basis == 'Fourier':
            fd_basis = basis.Fourier(
                [0, 2 * np.pi], n_basis=self.n_basis, period=self.period)
        else:
            fd_basis = basis.BSpline(n_basis=self.n_basis, order=self.order)

        data_matrix = sample.transpose()
        fd = FDataGrid(data_matrix).to_basis(fd_basis)
        coeffs = fd.coefficients.astype(np.float32).squeeze()

        if self.transpose:
            coeffs = coeffs.transpose()

        return coeffs.squeeze()


class Normalize(object):
    """Normalize the input channel-wise"""

    def __init__(self, axis=-1):
        self.axis = axis

    def __call__(self, sample):
        return (sample - np.mean(sample, axis=self.axis, keepdims=True)) / np.std(sample, axis=self.axis, keepdims=True)


class ToFFT(object):

    def __init__(self, axis=0):
        self.axis = axis

    def __call__(self, sample):
        L = sample.shape[self.axis]
        fs = fft(sample/L, axis=self.axis)
        fs = fs[1:len(fs) // 2 + 1, :]

        composite_fs = np.vstack([np.real(fs), np.imag(fs)])

        return composite_fs


class ToSequence(object):

    def __init__(self, length, stride=1):
        self.length = length
        self.stride = stride

    def __call__(self, sample):
        nrows, ncols = sample.shape
        seq_length = int((nrows - self.length) / self.stride) + 1
        seq = np.empty((self.length, ncols, seq_length),
                       dtype=np.float32)

        for i in range(seq_length):
            seq[:, :, i] = sample[self.stride*i:self.stride*i+self.length, :]

        return seq
