import numpy as np
from collections import deque
from typing import Tuple

class WindowFilter:
    def __init__(self, window_size: int = 10):
        self.window_size = window_size
        self.accel_x = deque(maxlen=window_size)
        self.accel_y = deque(maxlen=window_size)
        self.accel_z = deque(maxlen=window_size)
        self.gyro_x = deque(maxlen=window_size)
        self.gyro_y = deque(maxlen=window_size)
        self.gyro_z = deque(maxlen=window_size)
        self.mag_x = deque(maxlen=window_size)
        self.mag_y = deque(maxlen=window_size)
        self.mag_z = deque(maxlen=window_size)

    def update_accel(self, new_accel: Tuple[float, float, float]) -> Tuple[float, float, float]:
        self.accel_x.append(new_accel[0])
        self.accel_y.append(new_accel[1])
        self.accel_z.append(new_accel[2])
        
        filtered_x = np.mean(self.accel_x)
        filtered_y = np.mean(self.accel_y)
        filtered_z = np.mean(self.accel_z)
        
        return (filtered_x, filtered_y, filtered_z)
    
    def update_gyro(self, new_gyro: Tuple[float, float, float]) -> Tuple[float, float, float]:
        self.gyro_x.append(new_gyro[0])
        self.gyro_y.append(new_gyro[1])
        self.gyro_z.append(new_gyro[2])
        
        filtered_x = np.mean(self.gyro_x)
        filtered_y = np.mean(self.gyro_y)
        filtered_z = np.mean(self.gyro_z)
        
        return (filtered_x, filtered_y, filtered_z)
    
    def update_mag(self, new_mag: Tuple[float, float, float]) -> Tuple[float, float, float]:
        self.mag_x.append(new_mag[0])
        self.mag_y.append(new_mag[1])
        self.mag_z.append(new_mag[2])
        
        filtered_x = np.mean(self.mag_x)
        filtered_y = np.mean(self.mag_y)
        filtered_z = np.mean(self.mag_z)
        
        return (filtered_x, filtered_y, filtered_z)