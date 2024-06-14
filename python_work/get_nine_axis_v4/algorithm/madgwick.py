import numpy as np
import math

class Madgwick:
    def __init__(self, beta=0.1, dt=0.01):
        self.beta = beta
        self.dt = dt
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.prev_q = np.array([1.0, 0.0, 0.0, 0.0])
        self.error_integral = np.zeros(3)
        self.error_kp = 0.65
        self.error_ki = 0.0
        self.prev_euler_angles = np.zeros(3)

    def _assert_numerical_iterable(self, data, name):
        if not isinstance(data, (np.ndarray, list, tuple)):
            raise ValueError(f"{name} must be an array, list or tuple.")
        if not np.issubdtype(np.array(data).dtype, np.number):
            raise ValueError(f"{name} must contain numerical values.")

    def q_prod(self, q, r):
        w0, x0, y0, z0 = q
        w1, x1, y1, z1 = r
        return np.array([
            -x0 * x1 - y0 * y1 - z0 * z1 + w0 * w1,
             x0 * w1 + y0 * z1 - z0 * y1 + w0 * x1,
            -x0 * z1 + y0 * w1 + z0 * x1 + w0 * y1,
             x0 * y1 - y0 * x1 + z0 * w1 + w0 * z1])

    def q_conj(self, q):
        q = np.copy(q)
        q[1:] = -q[1:]
        return q
    #陀螺仪数据（gyr）：弧度每秒（rad/s）
    #加速度计数据（acc）：g（重力加速度的倍数）
    #磁力计数据（mag）：归一化的无量纲数据
    def updateMARG(self, gyr: np.ndarray, acc: np.ndarray, mag: np.ndarray, dt: float = None, sensitivity: float = 1.0) -> np.ndarray:
        self._assert_numerical_iterable(gyr, '三轴陀螺仪采样')
        self._assert_numerical_iterable(acc, '三轴加速度计采样')
        self._assert_numerical_iterable(mag, '三轴磁力计采样')

        # 确保使用弧度每秒作为陀螺仪数据的单位
        if np.max(np.abs(gyr)) > 10:  # 假定数据是度每秒
            gyr = np.radians(gyr)  # 转换为弧度每秒

        dt = self.dt if dt is None else dt

        if np.linalg.norm(gyr) == 0:
            return self.q

        gyr = np.asarray(gyr).flatten()
        qDot = 0.5 * self.q_prod(self.q, np.concatenate(([0], gyr))) * sensitivity

        a_norm = np.linalg.norm(acc)
        if a_norm > 0:
            acc = acc / a_norm  # 归一化加速度计数据
            mag = mag / np.linalg.norm(mag)  # 归一化磁力计数据
            h = self.q_prod(self.q, self.q_prod(np.concatenate(([0], mag)), self.q_conj(self.q)))
            bx = np.sqrt(h[1]**2 + h[2]**2)
            bz = h[3]

            f = np.array([
                2.0 * (self.q[1] * self.q[3] - self.q[0] * self.q[2]) - acc[0],
                2.0 * (self.q[0] * self.q[1] + self.q[2] * self.q[3]) - acc[1],
                2.0 * (0.5 - self.q[1]**2 - self.q[2]**2) - acc[2],
                2.0 * bx * (0.5 - self.q[2]**2 - self.q[3]**2) + 2.0 * bz * (self.q[1] * self.q[3] - self.q[0] * self.q[2]) - mag[0],
                2.0 * bx * (self.q[1] * self.q[2] - self.q[0] * self.q[3]) + 2.0 * bz * (self.q[0] * self.q[1] + self.q[2] * self.q[3]) - mag[1],
                2.0 * bx * (self.q[0] * self.q[2] + self.q[1] * self.q[3]) + 2.0 * bz * (0.5 - self.q[1]**2 - self.q[2]**2) - mag[2]
            ])
            J = np.array([
                [-2.0 * self.q[2], 2.0 * self.q[3], -2.0 * self.q[0], 2.0 * self.q[1]],
                [2.0 * self.q[1], 2.0 * self.q[0], 2.0 * self.q[3], 2.0 * self.q[2]],
                [0.0, -4.0 * self.q[1], -4.0 * self.q[2], 0.0],
                [-2.0 * bz * self.q[2], 2.0 * bz * self.q[3], -4.0 * bx * self.q[2] - 2.0 * bz * self.q[0], -4.0 * bx * self.q[3] + 2.0 * bz * self.q[1]],
                [-2.0 * bx * self.q[3] + 2.0 * bz * self.q[1], 2.0 * bx * self.q[2] + 2.0 * bz * self.q[0], 2.0 * bx * self.q[1] + 2.0 * bz * self.q[3], -2.0 * bx * self.q[0] + 2.0 * bz * self.q[2]],
                [2.0 * bx * self.q[2], 2.0 * bx * self.q[3] - 4.0 * bz * self.q[1], 2.0 * bx * self.q[0] - 4.0 * bz * self.q[2], 2.0 * bx * self.q[1]]
            ])
            gradient = J.T @ f
            gradient /= np.linalg.norm(gradient)
            qDot -= self.beta * gradient * sensitivity

            self.error_integral += gradient[:3] * dt * sensitivity
            self.error_integral = np.clip(self.error_integral, -0.035, 0.035)

        new_q = self.q + qDot * dt
        new_q /= np.linalg.norm(new_q)
        
        self.q = new_q

        return self.q

    def get_euler_angles(self):
        q1, q2, q3, q4 = self.q
        pitch = math.asin(-2.0 * (q2 * q4 - q1 * q3))
        roll = math.atan2(2.0 * (q1 * q2 + q3 * q4), 1.0 - 2.0 * (q2 * q2 + q3 * q3))
        yaw = math.atan2(2.0 * (q1 * q4 + q2 * q3), 1.0 - 2.0 * (q3 * q3 + q4 * q4))

        current_euler_angles = np.array([pitch, roll, yaw])
        angle_diff = current_euler_angles - self.prev_euler_angles

        angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi

        self.prev_euler_angles += angle_diff

        return self.prev_euler_angles.tolist()

    def get_smoothed_euler_angles(self):
        delta_q = self.q_prod(self.q, self.q_conj(self.prev_q))
        self.prev_q = np.copy(self.q)

        q1, q2, q3, q4 = delta_q
        pitch = math.asin(2.0 * (q1 * q3 - q4 * q2))
        roll = math.atan2(2.0 * (q1 * q2 + q3 * q4), 1.0 - 2.0 * (q2 * q2 + q3 * q3))
        yaw = math.atan2(2.0 * (q1 * q4 + q2 * q3), 1.0 - 2.0 * (q3 * q3 + q4 * q4))

        delta_euler_angles = np.array([pitch, roll, yaw])
        
        delta_euler_angles = (delta_euler_angles + np.pi) % (2 * np.pi) - np.pi

        self.prev_euler_angles += delta_euler_angles

        return self.prev_euler_angles.tolist()
