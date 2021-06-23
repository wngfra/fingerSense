import glob
import os
import re
import sys
import numpy as np

root_dir = sys.argv[1]

filelist = [y for x in os.walk(root_dir) for y in glob.glob(os.path.join(x[0], '*.npy'))]
for i, filename in enumerate(filelist):
    data_org  = np.load(filename)
    splits = np.array_split(data_org, 3, axis=0)
    np.save(filename, splits[0])
    np.save(filename.replace(".npy", "_0.npy"), splits[1])
    np.save(filename.replace(".npy", "_1.npy"), splits[2])
    