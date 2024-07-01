import numpy as np
import math
from typing import Tuple
from utils.logger import Logger
logger = Logger(__name__)
class MadgwickAHRS:
    def __init__(self, beta: float = 1.5):
        self.beta = beta
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

    def normalize(self, v: np.ndarray) -> np.ndarray:
        norm = np.linalg.norm(v)
        if norm == 0:
            raise ValueError("Zero norm vector")
        return v / norm

    def update(self, ax: float, ay: float, az: float, 
               gx: float, gy: float, gz: float,
               mx: float, my: float, mz: float, deltat: float):
        
        q1, q2, q3, q4 = self.q

        # 打印原始九轴数据
        logger.info(f"Original data - Acc: [{ax}, {ay}, {az}], Gyro: [{gx}, {gy}, {gz}], Mag: [{mx}, {my}, {mz}]")
        
        logger.info(f"Initial quaternion: {self.q}")
        
        # Normalize accelerometer measurement
        a = self.normalize(np.array([ax, ay, az]))
        ax, ay, az = a[0], a[1], a[2]
        logger.info(f"Normalized accelerometer: [{ax}, {ay}, {az}]")

        # Normalize magnetometer measurement
        m = self.normalize(np.array([mx, my, mz]))
        mx, my, mz = m[0], m[1], m[2]
        logger.info(f"Normalized magnetometer: [{mx}, {my}, {mz}]")

        # Auxiliary variables to avoid repeated arithmetic
        _2q1mx = 2.0 * q1 * mx
        _2q1my = 2.0 * q1 * my
        _2q1mz = 2.0 * q1 * mz
        _2q2mx = 2.0 * q2 * mx
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q4 = 2.0 * q4
        _2q1q3 = 2.0 * q1 * q3
        _2q3q4 = 2.0 * q3 * q4
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4

        # Reference direction of Earth's magnetic field
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4
        _2bx = np.sqrt(hx * hx + hy * hy)
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4
        _4bx = 2.0 * _2bx
        _4bz = 2.0 * _2bz

        logger.info(f"hx: {hx}, hy: {hy}, _2bx: {_2bx}, _2bz: {_2bz}")

        # Gradient descent algorithm corrective step
        s1 = -_2q3 * (2.0 * q2q4 - _2q1q3 - ax) + _2q2 * (2.0 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s2 = _2q4 * (2.0 * q2q4 - _2q1q3 - ax) + _2q1 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q2 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s3 = -_2q1 * (2.0 * q2q4 - _2q1q3 - ax) + _2q4 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q3 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s4 = _2q2 * (2.0 * q2q4 - _2q1q3 - ax) + _2q3 * (2.0 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        
        logger.info(f"s1: {s1}, s2: {s2}, s3: {s3}, s4: {s4}")
        
        norm = np.linalg.norm([s1, s2, s3, s4])
        if norm > 0:
            s1, s2, s3, s4 = s1 / norm, s2 / norm, s3 / norm, s4 / norm

        # Compute rate of change of quaternion
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

        logger.info(f"qDot1: {qDot1}, qDot2: {qDot2}, qDot3: {qDot3}, qDot4: {qDot4}")

        # Integrate to yield quaternion
        q1 += qDot1 * deltat
        q2 += qDot2 * deltat
        q3 += qDot3 * deltat
        q4 += qDot4 * deltat
        
        norm = np.linalg.norm([q1, q2, q3, q4])
        if norm > 0:
            self.q[0], self.q[1], self.q[2], self.q[3] = q1 / norm, q2 / norm, q3 / norm, q4 / norm

        logger.info(f"Updated quaternion: {self.q}\n")

    def get_quaternion(self) -> np.ndarray:
        return self.q
    
    def quaternion_to_rotation_matrix(self) -> np.ndarray:
        w, x, y, z = self.q
        R = np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ])
        return R

    def rotation_matrix_to_euler_angles(self, R: np.ndarray) -> np.ndarray:
        assert self.is_rotation_matrix(R)
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def is_rotation_matrix(self, R) -> bool:
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    def quaternion_to_euler(self) -> Tuple[float, float, float]:
        R = self.quaternion_to_rotation_matrix()
        euler = self.rotation_matrix_to_euler_angles(R)
        return tuple(euler)