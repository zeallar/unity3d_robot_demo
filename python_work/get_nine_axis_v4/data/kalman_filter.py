# kalman_filter.py

import numpy as np
from filterpy.kalman import KalmanFilter

class MultiAxisKalmanFilter:
    def __init__(self):
        self.kf = KalmanFilter(dim_x=3, dim_z=3)
        self.kf.x = np.array([0., 0., 0.])  # 初始状态
        self.kf.F = np.eye(3)  # 状态转移矩阵
        self.kf.H = np.eye(3)  # 观测矩阵
        self.kf.P *= 1000.  # 初始协方差矩阵
        self.kf.R = np.eye(3) * 0.1  # 观测噪声协方差矩阵
        self.kf.Q = np.eye(3) * 0.01  # 过程噪声协方差矩阵

    def apply_filter(self, x, y, z):
        self.kf.predict()
        self.kf.update([x, y, z])
        return self.kf.x
