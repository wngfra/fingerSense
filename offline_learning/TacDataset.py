# Copyright 2021 wngfra.
# SPDX-License-Identifier: Apache-2.0

import glob
import numpy as np
import os
import re
from bidict import bidict
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
            self.params[i] = (re.search(r"\d+.\d+", namegroups[1]).group(0),
                              re.search(r"\d+.\d+", namegroups[2]).group(0))
        self.textures = Texture(self.texture_names)

    def __len__(self):
        return len(self.filelist)

    def __getitem__(self, index):
        filename = os.path.join(self.root_dir, self.filelist[index])
        tacdata = np.load(filename)
        tacdata = tacdata[:, :16]
        texture_name = self.texture_names[index]
        if self.transform:
            tacdata = self.transform(tacdata)
        return tacdata, self.textures.get_id(texture_name)

    def get_info(self, index):
        return self.texture_names[index], self.params[index]
