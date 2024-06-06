import numpy as np

class Calibration:
    def __init__(self):
        self.calibration_data = {'acceleration': [], 'gyroscope': [], 'magnetometer': []}
        self.bias = {'acceleration': {'x': 0, 'y': 0, 'z': 0}, 'gyroscope': {'x': 0, 'y': 0, 'z': 0}, 'magnetometer': {'x': 0, 'y': 0, 'z': 0}}

    def add_data(self, data):
        self.calibration_data['acceleration'].append([data['acceleration']['x'], data['acceleration']['y'], data['acceleration']['z']])
        self.calibration_data['gyroscope'].append([data['gyroscope']['x'], data['gyroscope']['y'], data['gyroscope']['z']])
        self.calibration_data['magnetometer'].append([data['magnetometer']['x'], data['magnetometer']['y'], data['magnetometer']['z']])

    def calculate_bias(self):
        for sensor in self.calibration_data:
            data = np.array(self.calibration_data[sensor])
            self.bias[sensor]['x'] = np.mean(data[:, 0])
            self.bias[sensor]['y'] = np.mean(data[:, 1])
            self.bias[sensor]['z'] = np.mean(data[:, 2])

    def get_bias(self):
        return self.bias

    def apply_calibration(self, data):
        data['acceleration']['x'] -= self.bias['acceleration']['x']
        data['acceleration']['y'] -= self.bias['acceleration']['y']
        data['acceleration']['z'] -= self.bias['acceleration']['z']
        data['gyroscope']['x'] -= self.bias['gyroscope']['x']
        data['gyroscope']['y'] -= self.bias['gyroscope']['y']
        data['gyroscope']['z'] -= self.bias['gyroscope']['z']
        data['magnetometer']['x'] -= self.bias['magnetometer']['x']
        data['magnetometer']['y'] -= self.bias['magnetometer']['y']
        data['magnetometer']['z'] -= self.bias['magnetometer']['z']
        return data
