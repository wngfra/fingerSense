#! /usr/bin/env python3

import glob
import os
import re
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D
# from numpy.fft import fft
from skfda import FDataGrid
from skfda.representation import basis
from tensorly.decomposition import non_negative_tucker

Fs = 32
N_BASIS = 33
N_SPLITS = 3  # experiment repetition
DATA_PATH = None
SAVE_PATH = None

# construct Fourier basis
fd_basis = basis.Fourier([0, 2 * np.pi], n_basis=N_BASIS, period=1)
transform = np.log10


def get_cmap(n, name="plasma"):
    ''' Get a color map for 3D plot.'''
    return plt.cm.get_cmap(name, n)


def compute_cov_fft(data):
    pass


def compute_cov_fda(data):
    ''' Compute covariance matrix with functional basis decomposition.'''
    splits = np.array_split(data, N_SPLITS, axis=0)
    cov = np.zeros((N_BASIS, N_BASIS, N_SPLITS))
    for i, ds in enumerate(splits):
        fd = FDataGrid(ds.T).to_basis(fd_basis)
        coeffs = fd.coefficients.squeeze()
        cov[:, :, i] = np.cov(coeffs.T)

    return cov


def main():
    """ Find all files with extension 'npy' recursively."""
    try:
        files = [y for x in os.walk(DATA_PATH)
                 for y in glob.glob(os.path.join(x[0], '*.npy'))]
    except ValueError:
        print("Data directory wrong!")

    # prepare covariance tensor
    cov_fda = np.zeros((N_BASIS, N_BASIS, len(files) * N_SPLITS))
    # cov_fft = np.zeros((Fs * 16, Fs * 16, len(files)))

    tags = []  # data label

    for i, f in enumerate(files):
        # extract label from filename
        basename = os.path.basename(f)
        namegroups = basename.split("_")
        material = namegroups[0]
        force = re.search(r"\d+.\d+", namegroups[1]).group(0)
        speed = re.search(r"\d+.\d+", namegroups[2]).group(0)
        tags += [(material, force, speed)] * N_SPLITS
        # process data
        data = np.load(os.path.join(DATA_PATH, f))
        data = data[:, :16]
        data -= np.mean(data, axis=0)
        cov_fda[:, :, i*N_SPLITS:(i+1)*N_SPLITS] = compute_cov_fda(data)

    # tucker decomposition
    cov_tensor = cov_fda
    core, factors = non_negative_tucker(cov_tensor, rank=(3, 1, cov_tensor.shape[2]))
    core3d = core.squeeze().T  # get projected vectors
    if transform is not None:
        core3d = transform(core3d)
    print(core3d)

    # save tags into DataFrame
    df1 = pd.DataFrame(
        tags, columns=["material", "force", "speed"], dtype=float)
    df2 = pd.DataFrame(core3d, columns=["x1", "x2", "x3"], dtype=float)
    df = pd.concat([df1, df2], axis=1)

    df = df.iloc[3:, :]
    df = df[df["speed"] == 0.015]
    df = df[df['force'] > 3.0]

    # generate random color map
    unique_materials = df["material"].unique()
    unique_forces = df["force"].unique()
    unique_forces = df["speed"].unique()
    cmap = get_cmap(len(unique_materials))

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    for i, material in enumerate(unique_materials):
        X = df.loc[df["material"] == material]
        # plot core vectors
        xs, ys, zs = X["x1"], X["x2"], X["x3"]
        ax.scatter(xs, ys, zs, s=30, c=np.tile(cmap(i), (len(xs), 1)))
    ax.legend(unique_materials)
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
