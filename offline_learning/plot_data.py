#! /usr/bin/env python3

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

from numpy.fft import fft

N_SPLITS = 0
PLOT_ID = 0

def main():
    global N_SPLITS, PLOT_ID

    argc = len(sys.argv)
    if argc < 2:
        print("No data file specified!")
        exit(1)
    
    if argc > 2:
        N_SPLITS = int(sys.argv[2])
    
    if argc > 3:
        PLOT_ID = int(sys.argv[3])
        if PLOT_ID > N_SPLITS - 1:
            PLOT_ID = N_SPLITS - 1

    filepath = sys.argv[1]

    try:
        df = np.load(filepath)
        df = df[:, :16]
    except FileNotFoundError:
        print("File does not exist!")
        exit(1)

    if N_SPLITS > 0:
        splits = np.array_split(df, N_SPLITS, axis=0)
        data = splits[PLOT_ID]
    else:
        data = df

    # Compute the frequency spectrum
    L = len(data)
    Y = fft(data, axis=0)
    Ys = np.abs(Y/L)
    Ys = Ys[1:L//2+1, :]

    # Plot both time and frequency domains
    _ = plt.figure()
    plt.subplot(211)
    plt.plot(data)
    plt.subplot(212)
    plt.plot(Ys)
    plt.suptitle(os.path.basename(filepath), fontsize=20)
    plt.show()


if __name__ == '__main__':
    main()
