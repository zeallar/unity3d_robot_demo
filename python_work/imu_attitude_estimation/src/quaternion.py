import numpy as np

def axis_angle_to_quaternion(axis, angle):
    """
    将轴角转换为四元数
    axis: 旋转轴 (3,)
    angle: 旋转角度
    return: 四元数 (4,)
    """
    q0 = np.cos(angle / 2)
    q1 = axis[0] * np.sin(angle / 2)
    q2 = axis[1] * np.sin(angle / 2)
    q3 = axis[2] * np.sin(angle / 2)
    return np.array([q0, q1, q2, q3])

def quaternion_product(a, b):
    """
    计算两个四元数的乘积
    a: 四元数 (4,)
    b: 四元数 (4,)
    return: 四元数乘积 (4,)
    """
    ab = np.zeros(4)
    ab[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3]
    ab[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2]
    ab[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1]
    ab[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]
    return ab

def normalize_vector(v):
    """
    归一化向量
    v: 向量 (n,)
    return: 归一化后的向量 (n,)
    """
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

def acc_mag_to_quaternion(data):
    """
    将加速度计和地磁数据转换为四元数
    data: 包含加速度计和地磁数据的数组 (10,)
    return: 四元数 (4,)
    """
    vX = normalize_vector(np.cross(data[7:10], data[1:4]))
    vY = normalize_vector(np.cross(data[1:4], vX))

    qX = quaternion_from_two_vectors(vX, np.array([1, 0, 0]))
    y = quaternion_rotate_vector(vY, qX)
    qY = quaternion_from_two_vectors(y, np.array([0, 1, 0]))

    qx = np.array([-qX[0], qX[1], qX[2], qX[3]])
    qy = np.array([-qY[0], qY[1], qY[2], qY[3]])

    q = quaternion_product(qx, qy)
    q = np.array([q[0], -q[1], -q[2], -q[3]])
    if q[0] < 0:
        q = -q
    return q

def quaternion_from_two_vectors(u, v):
    """
    通过两个向量计算四元数
    u: 向量 (3,)
    v: 向量 (3,)
    return: 四元数 (4,)
    """
    half = normalize_vector(u + v)
    w = np.dot(u, half)
    xyz = np.cross(u, half)
    return np.concatenate(([w], xyz))

def quaternion_rotate_vector(vec, q):
    """
    使用四元数旋转向量
    vec: 向量 (3,)
    q: 四元数 (4,)
    return: 旋转后的向量 (3,)
    """
    x, y, z = vec
    qx, qy, qz, qw = q
    return np.array([
        (1 - 2 * (qy ** 2 + qz ** 2)) * x + 2 * (qx * qy - qw * qz) * y + 2 * (qx * qz + qw * qy) * z,
        2 * (qx * qy + qw * qz) * x + (1 - 2 * (qx ** 2 + qz ** 2)) * y + 2 * (qy * qz - qw * qx) * z,
        2 * (qx * qz - qw * qy) * x + 2 * (qy * qz + qw * qx) * y + (1 - 2 * (qx ** 2 + qy ** 2)) * z
    ])
