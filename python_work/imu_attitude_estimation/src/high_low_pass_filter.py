import numpy as np
from .quaternion import axis_angle_to_quaternion, quaternion_product, normalize_vector, acc_mag_to_quaternion
from .calibration import calculate_rotation_matrix, transform_mag_to_acc_frame

def high_low_pass_filter(Qfuse1, ImuData, t):
    """
    高低通滤波器
    Qfuse1: 上一次的四元数姿态估计 (4,)
    ImuData: 当前IMU数据 (10,)
    t: 时间间隔
    return: 更新后的四元数姿态估计 (4,)
    """
    a = 0.01
    norm_g = np.linalg.norm(ImuData[4:7])
    norm_a = np.linalg.norm(ImuData[1:4])
    
    if norm_g < 0.0873:  # 5 * pi / 180
        q = np.array([1, 0, 0, 0])
    else:
        q = axis_angle_to_quaternion(ImuData[4:7] / norm_g, norm_g * t)
    
    if abs(norm_a - 9.8) < 2 and norm_g < 2:
        # 计算旋转矩阵
        rotation_matrix = calculate_rotation_matrix(ImuData[1:4])
        # 将磁力计数据转换到加速度计坐标系
        mag_transformed = transform_mag_to_acc_frame(ImuData[7:10], rotation_matrix)
        # 用转换后的磁力计数据计算四元数
        ImuData[7:10] = mag_transformed
        
        Qtemp1 = acc_mag_to_quaternion(ImuData)
        Qtemp2 = quaternion_product(Qfuse1, q)
        
        if np.dot(Qtemp1, Qtemp2) > 0:
            Qfuse2 = (1 - a) * Qtemp2 + a * Qtemp1
        else:
            Qfuse2 = (1 - a) * Qtemp2 - a * Qtemp1
        
        Qfuse2 = normalize_vector(Qfuse2)
    else:
        Qfuse2 = quaternion_product(Qfuse1, q)
    
    if Qfuse2[0] < 0:
        Qfuse2 = -Qfuse2
        
    return Qfuse2
