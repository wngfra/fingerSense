#! /bin/env python3

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

from numpy.fft import fft

Fs = 32


def main():
    if len(sys.argv) < 2:
        print("No data file specified!")
        exit(1)

    filepath = sys.argv[1]

    try:
        df = pd.read_csv(filepath, header=0)
    except FileNotFoundError:
        print("File does not exist!")
        exit(1)

    df = df.iloc[:, 3:]

    # Compute the frequency spectrum
    L = len(df)
    Y = fft(df, axis=0)
    Ys = np.abs(Y/L)
    Ys = Ys[1:L//2+1, :]

    # Plot both time and frequency domains
    # _ = plt.figure(figsize=(20, 20))
    # plt.subplot(211)
    # plt.plot(df)
    # plt.subplot(212)
    # plt.plot(Ys)
    # plt.suptitle(os.path.basename(filepath), fontsize=20)
    # plt.show()
    plt.plot(df)
    plt.show()


if __name__ == '__main__':
    main()
