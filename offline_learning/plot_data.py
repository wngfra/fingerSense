#! /usr/bin/env python3
# Copyright 2021 wngfra.
# SPDX-License-Identifier: Apache-2.0

import matplotlib.pyplot as plt
import numpy as np
from utility import Normalize
from numpy.fft import fft
from torchvision import transforms
from TacDataset import TacDataset

transform = transforms.Compose([Normalize(axis=0)])
ds = TacDataset('../data', transform=transform)


def main():
    fig = plt.figure()
    for i, (data, _) in enumerate(ds):

        L = data.shape[0]
        Y = fft(data, axis=0)
        Ys = np.abs(Y / L)
        Ys = Ys[1:L//2+1, :]

        plt.subplot(211)
        plt.cla()
        plt.plot(data)
        plt.subplot(212)
        plt.cla()
        plt.plot(Ys)

        title, (force, speed) = ds.get_info(i)
        plt.suptitle("{} @ {}N and {}mmps".format(title, force, speed))

        figManager = plt.get_current_fig_manager()
        figManager.window.showMaximized()
        plt.show()


if __name__ == '__main__':
    main()
