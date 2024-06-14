
#### `src/calibration.py`

import numpy as np

def calibrate_data(data, Ta, Ka, Ba, Tg, Kg, Bg, Tm2a, Bm):
    """
    校准IMU数据
    data: 原始IMU数据 (10,)
    Ta, Ka, Ba: 加速度计校准参数
    Tg, Kg, Bg: 陀螺仪校准参数
    Tm2a, Bm: 地磁传感器校准参数
    return: 校准后的IMU数据 (10,)
    """
    data[1:4] = np.dot(Ta, np.dot(Ka, data[1:4] + Ba))
    data[4:7] = np.dot(Tg, np.dot(Kg, data[4:7] + Bg))
    data[7:10] = np.dot(Tm2a, data[7:10] + Bm)
    return data

def calculate_rotation_matrix(acc):
    """
    根据加速度计的数据计算旋转矩阵，将磁力计数据转换到加速度计坐标系
    acc: 加速度计数据 (3,)
    return: 旋转矩阵 (3, 3)
    """
    g = acc / np.linalg.norm(acc)
    h = np.array([1, 0, 0]) if abs(np.dot(g, np.array([1, 0, 0]))) < 0.9 else np.array([0, 1, 0])
    vX = np.cross(g, h)
    vX = vX / np.linalg.norm(vX)
    vY = np.cross(g, vX)
    vY = vY / np.linalg.norm(vY)
    vZ = g
    rotation_matrix = np.vstack((vX, vY, vZ)).T
    return rotation_matrix

def transform_mag_to_acc_frame(mag, rotation_matrix):
    """
    将磁力计数据转换到加速度计坐标系
    mag: 磁力计数据 (3,)
    rotation_matrix: 旋转矩阵 (3, 3)
    return: 转换后的磁力计数据 (3,)
    """
    return np.dot(rotation_matrix, mag)
