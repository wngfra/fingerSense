import numpy as np
import pandas as pd


class Perceptum:

    def __init__(self, dirs, model_name = 'Gaussian'):
        self.model_name = model_name # default gaussian model
        self.init_model(dirs)
        
    def init_model(self, dirs):
        '''
            Load prior knowldge and initialize the perception model
            ...

            Parameters
            ----------
            dirs : list of strings
                Directories of core, factors, info files
        '''
        self.core = np.load(dirs[0], allow_pickle=True).squeeze()
        self.factors = np.load(dirs[1], allow_pickle=True)[0:2]
        self.info = pd.read_csv(dirs[2], delimiter=',')

        class_names = self.info['class_name']
        unique_classes = set(class_names)
        self.classes = {}
        
        for uc in unique_classes:
            data = self.core[:, class_names == uc]
            mean = np.mean(data, axis=1)
            cov = np.cov(data)
            self.classes[uc] = [mean, cov]
