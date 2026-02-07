import numpy as np


class MovingAverageFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.buffer = []

    def apply(self, value):
        self.buffer.append(value)
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)
        return np.mean(self.buffer, axis=0)