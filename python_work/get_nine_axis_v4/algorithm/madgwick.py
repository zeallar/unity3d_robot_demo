# -*- coding: utf-8 -*-

import numpy as np
import math
from algorithm.common.orientation import q_prod, q_conj, acc2q, ecompass
from algorithm.utils.core import _assert_numerical_iterable
from typing import Tuple

class Madgwick:
    
    def __init__(self, gyr: np.ndarray = None, acc: np.ndarray = None, mag: np.ndarray = None, **kwargs):
        self.frequency: float = kwargs.get('frequency', 100.0)
        self.Dt: float = kwargs.get('Dt', (1.0/self.frequency) if self.frequency else 0.01)
        self.q0: np.ndarray = kwargs.get('q0', np.array([1.0, 0.0, 0.0, 0.0]))
        self.beta: float = kwargs.get('beta', 1.5)
        self._assert_validity_of_inputs()

    def _assert_validity_of_inputs(self):
        for item in ["frequency", "Dt", "beta"]:
            if isinstance(self.__getattribute__(item), bool):
                raise TypeError(f"Parameter '{item}' must be numeric.")
            if not isinstance(self.__getattribute__(item), (int, float)):
                raise TypeError(f"Parameter '{item}' is not a non-zero number.")
            if self.__getattribute__(item) <= 0.0:
                raise ValueError(f"Parameter '{item}' must be a non-zero number.")
        if self.q0 is not None:
            if not isinstance(self.q0, (list, tuple, np.ndarray)):
                raise TypeError(f"Parameter 'q0' must be an array. Got {type(self.q0)}.")
            self.q0 = np.copy(self.q0)
            if self.q0.shape != (4,):
                raise ValueError(f"Parameter 'q0' must be an array of shape (4,). It is {self.q0.shape}.")
            if not np.allclose(np.linalg.norm(self.q0), 1.0):
                raise ValueError(f"Parameter 'q0' must be a versor (norm equal to 1.0). Its norm is equal to {np.linalg.norm(self.q0)}.")

    def updateMARG(self, q: np.ndarray, gyr: np.ndarray, acc: np.ndarray, mag: np.ndarray, dt: float = None) -> np.ndarray:
        _assert_numerical_iterable(q, 'Quaternion')
        _assert_numerical_iterable(gyr, 'Tri-axial gyroscope sample')
        _assert_numerical_iterable(acc, 'Tri-axial accelerometer sample')
        _assert_numerical_iterable(mag, 'Tri-axial magnetometer sample')
        
        dt = self.Dt if dt is None else dt
        
        if gyr is None or not np.linalg.norm(gyr) > 0:
            return q
        if mag is None or not np.linalg.norm(mag) > 0:
            return q  # If no magnetometer data, use only accelerometer and gyroscope
        
        q1, q2, q3, q4 = q

        # Normalize accelerometer measurement
        acc = acc / np.linalg.norm(acc)
        ax, ay, az = acc[0], acc[1], acc[2]

        # Normalize magnetometer measurement
        mag = mag / np.linalg.norm(mag)
        mx, my, mz = mag[0], mag[1], mag[2]

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

        # Gradient descent algorithm corrective step
        s1 = -_2q3 * (2.0 * q2q4 - _2q1q3 - ax) + _2q2 * (2.0 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s2 = _2q4 * (2.0 * q2q4 - _2q1q3 - ax) + _2q1 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q2 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s3 = -_2q1 * (2.0 * q2q4 - _2q1q3 - ax) + _2q4 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q3 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s4 = _2q2 * (2.0 * q2q4 - _2q1q3 - ax) + _2q3 * (2.0 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)

        norm = np.linalg.norm([s1, s2, s3, s4])
        s1, s2, s3, s4 = s1 / norm, s2 / norm, s3 / norm, s4 / norm

        # Compute rate of change of quaternion using more stable integration
        qDot1 = 0.5 * (-q2 * gyr[0] - q3 * gyr[1] - q4 * gyr[2]) - self.beta * s1
        qDot2 = 0.5 * (q1 * gyr[0] + q3 * gyr[2] - q4 * gyr[1]) - self.beta * s2
        qDot3 = 0.5 * (q1 * gyr[1] - q2 * gyr[2] + q4 * gyr[0]) - self.beta * s3
        qDot4 = 0.5 * (q1 * gyr[2] + q2 * gyr[1] - q3 * gyr[0]) - self.beta * s4

        # Integrate to yield quaternion
        q1 += qDot1 * dt
        q2 += qDot2 * dt
        q3 += qDot3 * dt
        q4 += qDot4 * dt

        norm = np.linalg.norm([q1, q2, q3, q4])
        q1, q2, q3, q4 = q1 / norm, q2 / norm, q3 / norm, q4 / norm

        return np.array([q1, q2, q3, q4])
    
    def quaternion_to_rotation_matrix(self) -> np.ndarray:
        w, x, y, z = self.q0
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
