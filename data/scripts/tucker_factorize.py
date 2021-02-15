import numpy as np
import pandas as pd
from tensorly.decomposition import tucker

from torchvision import transforms
from TacDataset import Normalize, TacDataset, ToFDA, ToFFT

import matplotlib.pyplot as plt


n_basis = 33
n_channel = 16
period = 1


def extract_coeffs(ds):
    fcoeffs = np.zeros((n_basis-1, n_basis-1, len(ds)))

    for i in range(len(ds)):
        sample, _ = ds[i]
        cov = np.cov(sample[1:, :])
        fcoeffs[:, :, i] = cov

    return fcoeffs


def tucker_factorize(root_dir, transform):
    ds = TacDataset(root_dir, transform=tf)
    coeff_tensor = extract_coeffs(ds)
    core, factors = tucker(coeff_tensor, ranks=(3, 1, coeff_tensor.shape[2]))

    params = ds.get_params()
    d = {'class_name': ds.get_class_names(), 'class_id': ds.get_class_ids(),
         'pressure': params[:, 0], 'speed': params[:, 1]}
    df = pd.DataFrame(data=d)

    return core, factors, df


if __name__ == "__main__":
    tf = transforms.Compose(
        [Normalize(axis=1), ToFDA(basis='Fourier', n_basis=n_basis, period=period)])

    core, factors, info = tucker_factorize('../data/fabric', tf)
    np.save('core.npy', core)
    np.save('factors.npy', np.array(factors, dtype=object))
    info.to_csv('info.csv')
