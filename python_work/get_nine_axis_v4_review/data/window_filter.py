from collections import deque
import numpy as np

class WindowFilter:
    def __init__(self, window_size=10):
        self.window_size = window_size
        self.windows = {
            'acceleration': {'x': deque(maxlen=window_size), 'y': deque(maxlen=window_size), 'z': deque(maxlen=window_size)},
            'gyroscope': {'x': deque(maxlen=window_size), 'y': deque(maxlen=window_size), 'z': deque(maxlen=window_size)},
            'magnetometer': {'x': deque(maxlen=window_size), 'y': deque(maxlen=window_size), 'z': deque(maxlen=window_size)}
        }

    def add_data(self, data):
        for sensor in self.windows:
            for axis in self.windows[sensor]:
                self.windows[sensor][axis].append(data[sensor][axis])

    def get_filtered_data(self):
        filtered_data = {}
        for sensor in self.windows:
            filtered_data[sensor] = {}
            for axis in self.windows[sensor]:
                filtered_data[sensor][axis] = np.mean(self.windows[sensor][axis])
        return filtered_data
