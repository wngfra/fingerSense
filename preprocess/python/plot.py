import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

from scipy.signal import stft

Fs = 32

try:
    filepath = 'preprocess/data/PLA5_19.0_-0.05.csv'
    df = pd.read_csv(filepath, header=None)
    plt.plot(df)
    plt.show()

    f, t, Zxx = stft(df.T, Fs, nperseg=32)
    Zx = np.mean(Zxx, axis=0)
    plt.pcolormesh(t, f, np.abs(Zx), vmin=0, vmax=2, shading='gouraud')
    plt.title('STFT Magnitude')
    plt.ylabel('Frequency [Hz]')
    plt.xlabel('Time [sec]')
    plt.show()
except:
    raise ValueError("No specified filepath!")