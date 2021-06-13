# Copyright 2021 wngfra.
# SPDX-License-Identifier: Apache-2.0

import torch
import torch.nn as nn
import torch.nn.functional as F


class Encoder(nn.Module):
    def __init__(self):
        super().__init__()

    def forward(self, x):
        pass


class Decoder(nn.Module):
    def __init__(self):
        super().__init__()

    def forward(self, x):
        pass


class RAE(nn.Module):
    def __init__(self, input_size, hidden_size):
        super().__init__()

        self.encoder = Encoder()
        self.decoder = Decoder()

    def foward(self, x):
        torch.manual_seed(0)
        encoded = self.encoder(x)
        decoded = self.decoder(encoded)

        return decoded

    def encode(self, x):
        self.eval()
        with torch.no_grad():
            encoded = self.encoder(x)
        return encoded

    def decode(self, x):
        self.eval()
        with torch.no_grad():
            decoded = self.decoder(x)
        return decoded
