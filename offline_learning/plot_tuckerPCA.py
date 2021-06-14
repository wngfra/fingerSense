import enum
import os
import re
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D
from numpy.fft import fft

from skfda import FDataGrid
from skfda.representation import basis
from tensorly.decomposition import tucker

Fs = 32
N_BASIS = 33        
N_SPLITS = 3  # experiment repetition
DATA_PATH = None
SAVE_PATH = None

# construct Fourier basis
fd_basis = basis.Fourier([0, np.pi], n_basis=N_BASIS, period=1)
transform = np.log


def get_cmap(n, name="plasma"):
    ''' Get a color map for 3D plot.'''
    return plt.cm.get_cmap(name, n)


def compute_cov_fft(data):
    pass


def compute_cov_fda(data):
    ''' Compute covariance matrix with functional basis decomposition.'''
    data -= np.mean(data, axis=0)
    splits = np.array_split(data, N_SPLITS, axis=0)
    cov = np.zeros((N_BASIS, N_BASIS, N_SPLITS))
    for i, ds in enumerate(splits):
        fd = FDataGrid(ds.T).to_basis(fd_basis)
        coeffs = fd.coefficients.squeeze()
        cov[:, :, i] = np.cov(coeffs.T)

    return cov


def main():
    ''' Tensor PCA by tucker decomposition.'''
    try:
        dirs = os.listdir(DATA_PATH)
        files = list(filter(lambda x: "csv" in x, dirs))
    except ValueError:
        print("Data directory wrong!")

    # prepare covariance tensor
    cov_fda = np.zeros((N_BASIS, N_BASIS, len(files * N_SPLITS)))
    # cov_fft = np.zeros((Fs * 16, Fs * 16, len(files)))

    tags = []  # data label

    for i, f in enumerate(files):
        # extract label from filename
        basename = os.path.splitext(f)[0]
        namegroup = basename.split("@")
        material = namegroup[0]
        force = re.search(r"\d+.\d+", namegroup[1]).group(0)
        speed = re.search(r"\d+.\d+", namegroup[2]).group(0)

        for _ in range(N_SPLITS):
            tags.append((material, force, speed))

        data = pd.read_csv(f"{DATA_PATH}{f}", header=None, usecols=range(16))
        cov_fda[:, :, i*N_SPLITS:i*N_SPLITS + N_SPLITS] = compute_cov_fda(data)

    # tucker decomposition
    cov_tensor = cov_fda
    core, factors = tucker(cov_tensor, rank=(3, 1, cov_tensor.shape[2]))
    core3d = core.squeeze().T  # get projected vectors

    # save tags into DataFrame
    df1 = pd.DataFrame(
        tags, columns=["material", "force", "speed"], dtype=float)
    df2 = pd.DataFrame(core3d, columns=["x1", "x2", "x3"], dtype=float)
    if transform != None:
        df2 = transform(df2)
    df = pd.concat([df1, df2], axis=1)

    # generate random color map
    classes = pd.unique(df["material"])
    cmap = get_cmap(len(classes))

    fig = plt.figure(figsize=(30, 30))
    ax = fig.add_subplot(111, projection="3d")
    for i, m in enumerate(classes):
        X = df.loc[df["material"] == m]
        # plot core vectors
        xs, ys, zs = X["x1"], X["x2"], X["x3"]
        ax.scatter(xs, ys, zs, s=30, c=np.tile(cmap(i), (len(xs), 1)))
    ax.legend(classes)
    plt.show()

    if SAVE_PATH is not None:
        df.to_csv(f"{SAVE_PATH}/core.csv")
        np.save(f"{SAVE_PATH}/factors.npy", factors, allow_pickle=True)


if __name__ == "__main__":
    if len(sys.argv) >= 2:
        DATA_PATH = sys.argv[1]
    if len(sys.argv) >= 3:
        SAVE_PATH = sys.argv[2]
    main()
