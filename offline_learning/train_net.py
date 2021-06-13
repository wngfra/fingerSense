# Copyright 2021 wngfra.
# SPDX-License-Identifier: Apache-2.0

import matplotlib.pyplot as plt
from torchvision.transforms.transforms import Pad
from torch.utils.data import DataLoader
from torchvision import transforms

from collate_functions import PadSequence
from TacDataset import TacDataset

device = torch.device('cuda:0') if torch.cuda.is_available() else torch.device('cpu')

ds = TacDataset('../data', transform=None)
train_loader = DataLoader(
    ds, batch_size=32, collate_fn=PadSequence(), num_workers=6, pin_memory=True, shuffle=True)

if __name__ == '__main__':
    for i, data in enumerate(train_loader):
        print(data)
