import os
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D
from numpy.fft import fft
# from skfda import FDataGrid
# from skfda.representation import basis
from tensorly.decomposition import tucker


DEBUG = True
SAVE_RESULT = True

Fs = 32
N_BASIS = 17
OFFSET = 1
ROOT_PATH = './preprocess/'
DATA_PATH = ROOT_PATH + 'data/fabrics/'

# NOTE channel 9 malfunctions
usecols = [i for i in range(16) if i != 8]


def get_cmap(n, name='plasma'):
    return plt.cm.get_cmap(name, n)


def main():
    ''' Pre-processing data and save to files
        NOTE extract projection matrices by tucker decomposition
    '''
    # find all csv files
    dirs = os.listdir(DATA_PATH)
    files = list(filter(lambda x: 'csv' in x, dirs))

    # construct Fourier basis
    # fd_basis = basis.Fourier([0, np.pi], n_basis=N_BASIS, period=1)

    # prepare covariance tensor
    cov_tensor = np.zeros((Fs * len(usecols), Fs * len(usecols), len(files)))

    # prepare data list
    tags = []

    for i, f in enumerate(files):
        # extract data info from filename
        basename = os.path.splitext(f)[0]
        namegroup = basename.split('_')
        material = namegroup[0]
        pressure = re.search(r'\d+.\d+', namegroup[1]).group(0)
        speed = re.search(r'\d+.\d+', namegroup[2]).group(0)

        tags.append((material, pressure, speed))

        # load data
        data = pd.read_csv(f'{DATA_PATH}{f}', usecols=usecols, header=None)

        # splits data into equal-sized segments
        L = len(data)
        N = L // (Fs * 2)
        data = data.iloc[L - N * (Fs * 2):, :]
        splits = np.array_split(data, N, axis=0)
        sample = np.zeros((Fs * len(usecols), len(splits)))

        # transforms to covariance of fourier coefficients
        for j, sd in enumerate(splits):
            # fd = FDataGrid(sd.T).to_basis(fd_basis)
            # coeffs = fd.coefficients.squeeze()
            Y = fft(sd/len(sd))
            Ys = np.abs(Y)
            Ys = Ys[1:len(Ys)//2+1, :]
            sample[:, j] = Ys.flatten()
            
        cov = np.corrcoef(sample)
        cov_tensor[:, :, i] = cov

    """ tucker decomposition
    """
    core, factors = tucker(cov_tensor, rank=(3, 1, cov_tensor.shape[2]))
    core3d = core.squeeze().T

    # save tags into DataFrame
    df1 = pd.DataFrame(
        tags, columns=['material', 'pressure', 'speed'], dtype=float)
    df2 = pd.DataFrame(core3d, columns=['x1', 'x2', 'x3'], dtype=float)
    df = pd.concat([df1, df2], axis=1)

    # generate random color map
    ums = pd.unique(df['material'])
    cmap = get_cmap(len(ums))

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    df = df.loc[np.abs(df['x1']) < 1e-10]

    for i, m in enumerate(ums):
        if DEBUG:
            # plot coefficients and covariance matrix
            X = df.loc[df['material'] == m]

            # plot core vectors
            xs, ys, zs = X['x1'], X['x2'], X['x3']
            ax.scatter(xs, ys, zs, s=30, c=np.tile(cmap(i), (len(xs), 1)))

    plt.show()

    if SAVE_RESULT:
        df.to_csv(f'{ROOT_PATH}core.csv')
        np.save(f'{ROOT_PATH}factors.npy', factors, allow_pickle=True)


if __name__ == '__main__':
    main()
