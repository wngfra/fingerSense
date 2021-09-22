# Copyright 2021 wngfra.
# SPDX-License-Identifier: Apache-2.0

import glob
import os
import re
import numpy as np
from bidict import bidict
from numpy.lib.stride_tricks import sliding_window_view
from torch.utils.data import Dataset

class Texture:
    """ Create a bidict from a texture name list."""

    def __init__(self, texture_names):
        self.texture_by_id = bidict()
        for i, tn in enumerate(set(texture_names)):
            self.texture_by_id[tn] = i

    def get_id(self, texture_name: str):
        return self.texture_by_id[texture_name]

    def get_name(self, texture_id: int):
        return self.texture_by_id.inverse[texture_id]


class TacDataset(Dataset):
    def __init__(self, root_dir, transform=None):
        self.root_dir = root_dir
        self.transform = transform

        self.filelist = [y for x in os.walk(
            root_dir) for y in glob.glob(os.path.join(x[0], '*.npy'))]
        self.params = [(0.0, 0.0)] * len(self.filelist)
        self.texture_names = []
        for i, filename in enumerate(self.filelist):
            basename = os.path.basename(filename)
            namegroups = basename.split('_')

            self.texture_names.append(namegroups[0])
            self.params[i] = [float(re.search(r"\d+.\d+", namegroups[1]).group(0)),
                              float(re.search(r"-?\d+.\d+", namegroups[2]).group(0))]
        self.textures = Texture(self.texture_names)

    def __len__(self):
        return len(self.filelist)

    def __getitem__(self, index):
        filename = os.path.join(self.root_dir, self.filelist[index])
        rawdata  = np.load(filename)
        tacdata  = rawdata[32:-32, :16]
        # wrench   = rawdata[:, -6:]
        texture_name = self.texture_names[index]
        if self.transform:
            tacdata = self.transform(tacdata)
        labels = [self.textures.get_id(texture_name)] + self.params[index]
        return tacdata, labels

    def get_texture_name(self, texture_id):
        return self.textures.get_name(texture_id)


# +
""" Custom transforms """

class Normalize(object):
    def __init__(self, axis=0):
        self.axis = axis

    def __call__(self, sample):
        return (sample - np.mean(sample, axis=self.axis, keepdims=True)) / np.std(sample, axis=self.axis, keepdims=True)

# +
from skfda import FDataGrid
from skfda.representation import basis

class ToFDA(object):
    def __init__(self, flatten=True):
        self.basis = basis.Fourier((0, np.pi), nbasis=33, period=1)
        self.flatten = flatten
    
    def __call__(self, x):
        ''' Basis expansion'''
        fd = FDataGrid(x).to_basis(self.basis)
        coeffs = fd.coefficients.squeeze()
        coeffs = coeffs[:, 1:].T
        if self.flatten:
            coeffs = coeffs.flatten()
        
        return coeffs

