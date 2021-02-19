import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

from scipy.signal import stft

Fs = 32

try:
    filepath = 'preprocess/data/NavyDenim_15.0_-0.05.csv'
    df = pd.read_csv(filepath, header=None)

    f, t, Zxx = stft(df.T, Fs, nperseg=32)
    Zx = np.mean(Zxx, axis=0)
    plt.pcolormesh(t, f, np.abs(Zx), vmin=0, vmax=0.5, shading='gouraud')
    plt.title('STFT Magnitude')
    plt.ylabel('Frequency [Hz]')
    plt.xlabel('Time [sec]')
    plt.show()
except:
    raise ValueError("No specified filepath!")