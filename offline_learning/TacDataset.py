# Copyright 2021 wngfra.
# SPDX-License-Identifier: Apache-2.0

import glob
import numpy as np
import os
from torch.utils.data import Dataset

string2id = {
    'BlackWool': 0,
    'GreenVelvet': 1,
    'NavyDenim': 2
}


class TacDataset(Dataset):
    def __init__(self, root_dir, transform=None):
        self.root_dir = root_dir
        self.transform = transform

        self.filelist = glob.glob(os.path.join(root_dir, '*.csv'))
        self.texture_labels = [""] * len(self.filelist)
        self.params = [(0.0, 0.0)] * len(self.filelist)
        for i, filename in enumerate(self.filelist):
            basename = os.path.basename(filename)
            namegroups = basename.split('@')

            self.texture_labels[i] = namegroups[0]
            self.params = (namegroups[1][:-1], namegroups[2][:-7])

    def __len__(self):
        return len(self.filelist)

    def __getitem__(self, index):
        tacdata_path = os.path.join(self.root_dir, self.filelist[index])
        tacdata = np.genfromtxt(tacdata_path, delimiter=',')
        label = self.texture_labels[index]
        if self.transform:
            tacdata = self.transform(tacdata)
        return tacdata, string2id[label]
