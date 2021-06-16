# Copyright 2021 wngfra.
# SPDX-License-Identifier: Apache-2.0

import numpy as np
import torch
from skfda import FDataGrid
from skfda.representation import basis


""" Custom transforms """


class Normalize(object):
    def __init__(self, axis=0):
        self.axis = axis

    def __call__(self, sample):
        return (sample - np.mean(sample, axis=self.axis)) / np.std(sample, axis=self.axis)


class ToFourierBasis(object):
    def __init__(self, n_basis, period) -> None:
        self.basis = basis.Fourier(
            [0, 2 * np.pi], n_basis=n_basis, period=period)

    def __call__(self, sample):
        # Input sample matrix
        # Each row contains one observation and each column represents one variable
        fd = FDataGrid(sample.T).to_basis(self.basis)
        coeffs = fd.coefficients.squeeze()

        return coeffs.T


""" Custom collate functions"""


class PadSequence(object):
    def __call__(self, batch):
        # Each element in "batch" is a tuple (data, label).
        # Sort the batch in the descending order
        sorted_batch = sorted(batch, key=lambda x: x[0].shape[0], reverse=True)
        # Get each sequence and pad it
        sequences = [torch.Tensor(x[0]) for x in sorted_batch]
        sequences_padded = torch.nn.utils.rnn.pad_sequence(
            sequences, batch_first=True)
        # Store the length of each sequence
        lengths = torch.Tensor([len(x) for x in sequences])
        labels = torch.Tensor(list(map(lambda x: x[1], sorted_batch)))
        return sequences_padded, lengths, labels
