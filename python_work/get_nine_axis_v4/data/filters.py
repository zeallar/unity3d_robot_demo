# filters.py
import numpy as np

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, estimated_measurement_variance):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimated_measurement_variance = estimated_measurement_variance
        self.posteri_estimate = 0.0
        self.posteri_error_estimate = 1.0

    def filter(self, measurement):
        priori_estimate = self.posteri_estimate
        priori_error_estimate = self.posteri_error_estimate + self.process_variance

        blending_factor = priori_error_estimate / (priori_error_estimate + self.measurement_variance)
        self.posteri_estimate = priori_estimate + blending_factor * (measurement - priori_estimate)
        self.posteri_error_estimate = (1 - blending_factor) * priori_error_estimate

        return self.posteri_estimate

def moving_average(data, window_size):
    return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

def apply_kalman_filter(data, process_variance=1e-5, measurement_variance=0.1 ** 2, estimated_measurement_variance=1.0):
    kalman_filter = KalmanFilter(process_variance, measurement_variance, estimated_measurement_variance)
    return np.array([kalman_filter.filter(data_point) for data_point in data])

def combined_filter(data, process_variance=1e-5, measurement_variance=0.1 ** 2, estimated_measurement_variance=1.0, window_size=5):
    kalman_filtered_data = apply_kalman_filter(data, process_variance, measurement_variance, estimated_measurement_variance)
    combined_filtered_data = moving_average(kalman_filtered_data, window_size)
    return combined_filtered_data

def threshold_filter(data, threshold=0.5):
    filtered_data = []
    current_value = data[0]
    for value in data:
        if abs(value - current_value) > threshold:
            current_value = value
        filtered_data.append(current_value)
    return np.array(filtered_data)
