import os
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from skfda import FDataGrid
from skfda.representation import basis
from tensorly.decomposition import tucker
from mpl_toolkits.mplot3d import Axes3D

N_BASIS = 33
OFFSET = 0

''' Pre-processing data and save to files
'''

def main():
    # find all csv files
    dirs = os.listdir('data/')
    files = list(filter(lambda x: '.csv' in x, dirs))

    # construct Fourier basis
    fd_basis = basis.Fourier([0, 2 * np.pi], n_basis=N_BASIS, period=1)

    # prepare covariance tensor
    cov_tensor = np.zeros((N_BASIS - OFFSET, N_BASIS - OFFSET, len(files)))

    # prepare info list
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
        data = pd.read_csv(
            f"./data/{f}", usecols=[i for i in range(16) if i != 8], header=None)
        data /= float(pressure)
        data = data.iloc[153:, :].values

        # transform to functional representation
        data_transposed = data.transpose()
        fd = FDataGrid(data_transposed).to_basis(fd_basis)
        coeffs = fd.coefficients.astype(np.float32).squeeze()
        cov = np.corrcoef(coeffs.T[OFFSET:, :])
        cov_tensor[:, :, i] = (cov + 1.0) * 0.5

    core, factors = tucker(cov_tensor, rank=(3, 1, cov_tensor.shape[2]))
    core3d = core.squeeze()

    # save tags into DataFrame
    df = pd.DataFrame(tags, columns =['material', 'pressure', 'speed'], dtype=float)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(core3d[0, :], core3d[1, :], core3d[2, :], s=40, c='r')
    plt.show()


if __name__ == '__main__':
    main()
