# Copyright 2021 wngfra.
# SPDX-License-Identifier: Apache-2.0

import torch


class PadSequence(object):
    def __call__(self, batch):
        # Each element in "batch" is a tuple (data, label).
        # Sort the batch in the descending order
        sorted_batch = sorted(batch, key=lambda x: x[0].shape[0], reverse=True)
        # Get each sequence and pad it
        sequences = [torch.Tensor(x[0]) for x in sorted_batch]
        sequences_padded = torch.nn.utils.rnn.pad_sequence(
            sequences, batch_first=True)
        # Store the length of each sequence
        lengths = torch.Tensor([len(x) for x in sequences])
        labels = torch.Tensor(list(map(lambda x: x[1], sorted_batch)))
        return sequences_padded, lengths, labels
