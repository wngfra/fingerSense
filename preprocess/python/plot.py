import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

from numpy.fft import fft

Fs = 32
basename = 'preprocess/data/PLAVertical_20.0_-'


if __name__ == '__main__':
    try:
        filepath = f'{basename}0.04.csv'
        df = pd.read_csv(filepath, header=None)

    except:
        raise ValueError("No specified filepath!")

    N = len(df) // Fs
    splits = np.array_split(df, N, axis=0)

    L = len(df)
    Y = fft(df, axis=0)
    Ys = np.abs(Y/L)
    Ys = Ys[1:L//2+1, :]

    fig = plt.figure(figsize=(20, 20))
    plt.subplot(211)
    plt.plot(df)
    plt.subplot(212)
    plt.plot(Ys)
    plt.plot()

    # transforms to covariance of fourier coefficients
    for j, data in enumerate(splits):
        L = len(data)
        Y = fft(data, axis=0)
        Ys = np.abs(Y/L)
        Ys = Ys[1:L//2+1, :]
        cov = np.cov(Ys)

        _, axs = plt.subplots(3, 1, figsize=(21, 21))
        axs[0].imshow(cov)
        axs[1].plot(Ys)
        axs[2].plot(data)
        plt.show()