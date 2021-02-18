import os
import re
import numpy as np
from numpy.lib.arraysetops import unique
import pandas as pd
import matplotlib.pyplot as plt

from skfda import FDataGrid
from skfda.representation import basis
from tensorly.decomposition import tucker
from mpl_toolkits.mplot3d import Axes3D

DEBUG = True

N_BASIS = 65
OFFSET = 1
ROOT_PATH = './preprocess/'
DATA_PATH = ROOT_PATH + 'data/'


def get_cmap(n, name='seismic'):
    return plt.cm.get_cmap(name, n)


def main():
    ''' Pre-processing data and save to files
        NOTE extract projection matrices by tucker decomposition
    '''
    # find all csv files
    dirs = os.listdir(DATA_PATH)
    files = list(filter(lambda x: '.csv' in x, dirs))

    # construct Fourier basis
    fd_basis = basis.Fourier([0, 2 * np.pi], n_basis=N_BASIS, period=1)

    # prepare covariance tensor
    cov_tensor = np.zeros((N_BASIS - OFFSET, N_BASIS - OFFSET, len(files)))

    # prepare data list
    coeff_list = []
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
        # NOTE channel 9 malfunctions
        usecols = [i for i in range(16) if i != 8]
        data = pd.read_csv(f'{DATA_PATH}{f}', usecols=usecols, header=None)

        # transform to functional representation
        fd = FDataGrid(data.T).to_basis(fd_basis)
        coeffs = fd.coefficients.squeeze()
        coeffs = coeffs.T[OFFSET:, :]

        coeff_list.append(coeffs)

        # append the coefficient and covariance matrices
        coeff_list.append(coeffs)
        cov_tensor[:, :, i] = np.corrcoef(coeffs)

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

    # debug plotting
    if DEBUG:
        # prepare figures     
        fig2 = plt.figure()
        ax = fig2.add_subplot(111, projection='3d')

        for i, m in enumerate(ums):
            # plot coefficients and covariance matrix
            ind = df.loc[(df['material'] == m) & (df['pressure'] == 8.0)].index
            fig1, axes = plt.subplots(2, 10, figsize=(32, 16))
            fig1.suptitle(m, fontsize=20)
            for i in range(10):
                axes[0][i].plot(coeff_list[ind[i]])
                axes[1][i].imshow(cov_tensor[:, :, ind[i]])

            # plot core vectors
            # X = df.loc[df['material'] == m][['x1', 'x2', 'x3']]
            # xs, ys, zs = X.iloc[:, 0], X.iloc[:, 1], X.iloc[:, 2]
            # ax.scatter(xs, ys, zs, s=25, c=np.tile(cmap(i), (len(xs), 1)))
            
            plt.show()

    # df.to_csv(f'{ROOT_PATH}core.csv')
    # np.save(f'{ROOT_PATH}factors.npy', factors, allow_pickle=True)


if __name__ == '__main__':
    main()
