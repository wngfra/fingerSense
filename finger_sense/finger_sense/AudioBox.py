# Copyright 2021 wngfra.
# SPDX-License-Identifier: Apache-2.0

import asyncio
import sys
import time
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
import sounddevice as sd


class AudioBox:

    def __init__(self, device, channels=[1], sample_rate=48000.0, buffer_size=None, filter=None) -> None:
        self.stream = sd.InputStream(
            device=device, channels=max(channels), samplerate=sample_rate, callback=self.stream_callback)
        self.buffer = deque(maxlen=buffer_size)
        # Channel numbers start with 1
        self.mapping = [c - 1 for c in channels]

    def stream_callback(self, indata, frames, time, status):
        """This is called (from a separate thread) for each audio block."""
        if status:
            print(status, file=sys.stderr)
        # Fancy indexing with mapping creates a (necessary!) copy:
        self.buffer.append(indata[:, self.mapping])

    def record(self, duration=None, aync=False):
        """Start recording; if duration=None, wait for stop command."""
        self.stream.start()
        if duration is not None:
            time.sleep(int(duration * 1000))
            self.stream.stop()
    
    def stop(self):
        if self.stream.active:
            self.stream.stop()

    def save(self, filename):
        np.savetxt(filename, self.buffer, )

    def plot(self):
        plt.plot(self.buffer)
        plt.show()

if __name__=='__main__':
    ab = AudioBox(device=1)
    ab.record(duration=3.0)