# Copyright 2021 wngfra.
# SPDX-License-Identifier: Apache-2.0

import enum
import numpy as np
import matplotlib.pyplot as plt
import torch
from torch.utils.data import DataLoader, dataloader
from torchvision import transforms

from utility import Normalize, PadSequence, ToFourierBasis
from TacDataset import TacDataset

from scipy.spatial.distance import euclidean
from fastdtw import fastdtw

device = torch.device(
    'cuda:0') if torch.cuda.is_available() else torch.device('cpu')

transform = transforms.Compose([Normalize(axis=0)])
ds = TacDataset('../data', transform=transform)
train_loader = DataLoader(
    ds, batch_size=16, collate_fn=PadSequence(), pin_memory=True, shuffle=True)

if __name__ == '__main__':
    for i, (batch, lengths, labels) in enumerate(train_loader):
        print(i)

