import numpy as np
import math
class My_Madgwick:
    def __init__(self, beta=0.1, sample_freq=256.0):
        self.beta = beta  # Algorithm gain
        self.sample_freq = sample_freq  # Sample frequency in Hz
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # Initial quaternion

    def update_imu(self, gx, gy, gz, ax, ay, az):
        q1, q2, q3, q4 = self.q

        # Convert gyroscope degrees/sec to radians/sec
        gx *= np.pi / 180
        gy *= np.pi / 180
        gz *= np.pi / 180

        # Normalize accelerometer measurement
        norm = np.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0.0:
            return  # Handle NaN
        ax /= norm
        ay /= norm
        az /= norm

        # Auxiliary variables to avoid repeated calculations
        _2q1mx = 2.0 * q1 * ax
        _2q1my = 2.0 * q1 * ay
        _2q1mz = 2.0 * q1 * az
        _2q2mx = 2.0 * q2 * ax
        hx = _2q1mx + q2 * az - q4 * ay
        hy = _2q1my + q4 * ax - q3 * az
        _2bx = np.sqrt(hx * hx + hy * hy)
        _2bz = -q1 * ay + q2 * ax + q4 * az - q3 * ax

        # Gradient decent algorithm corrective step
        s1 = -_2q3 * (2.0 * q2 * q4 - _2q1 * q3 - ax) + _2q2 * (2.0 * q1 * q2 + _2q3 * q4 - ay)
        s2 = _2q4 * (2.0 * q2 * q4 - _2q1 * q3 - ax) + _2q1 * (2.0 * q1 * q2 + _2q3 * q4 - ay) - 4.0 * q2 * (1.0 - 2.0 * q2 * q2 - 2.0 * q3 * q3 - az)
        s3 = -4.0 * q1 * q3 * (1.0 - 2.0 * q2 * q2 - 2.0 * q3 * q3 - az) + _2bz * q3 * (_2bx * (0.5 - q3 * q3 - q4 * q4) + _2bz * (q2 * q4 - q1 * q3) - mx) + (_2bx * q4 + _2bz * q2) * (_2bx * (q2 * q3 - q1 * q4) + _2bz * (q1 * q2 + q3 * q4) - my) + (_2bx * q3 - 4.0 * q4 * _2bz) * (_2bx * (q1 * q3 + q2 * q4) + _2bz * (q3 * q4 - q1 * q2) - mz)
        s4 = 4.0 * q1 * q4 * (1.0 - 2.0 * q2 * q2 - 2.0 * q3 * q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3 * q3 - q4 * q4) + _2bz * (q2 * q4 - q1 * q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2 * q3 - q1 * q4) + _2bz * (q1 * q2 + q3 * q4) - my) + (_2bx * q2 - 4.0 * q3 * _2bz) * (_2bx * (q1 * q3 + q2 * q4) + _2bz * (q3 * q4 - q1 * q2) - mz)
        norm = np.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)
        if norm == 0.0:
            return  # Handle NaN
        s1 /= norm
        s2 /= norm
        s3 /= norm
        s4 /= norm

        # Compute rate of change of quaternion
        q_dot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        q_dot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        q_dot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        q_dot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

        # Integrate to yield quaternion
        q1 += q_dot1 * (1.0 / self.sample_freq)
        q2 += q_dot2 * (1.0 / self.sample_freq)
        q3 += q_dot3 * (1.0 / self.sample_freq)
        q4 += q_dot4 * (1.0 / self.sample_freq)
        norm = np.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
        if norm == 0.0:
            return  # Handle NaN
        self.q = np.array([q1 / norm, q2 / norm, q3 / norm, q4 / norm])

    def update_marg(self, gx, gy, gz, ax, ay, az, mx, my, mz):
        q1, q2, q3, q4 = self.q

        # Convert gyroscope degrees/sec to radians/sec
        gx *= np.pi / 180
        gy *= np.pi / 180
        gz *= np.pi / 180

        # Normalize accelerometer measurement
        norm = np.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0.0:
            return  # Handle NaN
        ax /= norm
        ay /= norm
        az /= norm

        # Normalize magnetometer measurement
        norm = np.sqrt(mx * mx + my * my + mz * mz)
        if norm == 0.0:
            return  # Handle NaN
        mx /= norm
        my /= norm
        mz /= norm

        # Auxiliary variables to avoid repeated calculations
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
        q2q2 = q2 * q2
        q3q3 = q3 * q3
        q4q4 = q4 * q4
        hx = mx * q1 * q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2 * q2 + _2q2 * q3 * my + _2q2 * q4 * mz - mx * q3 * q3 - mx * q4 * q4
        hy = _2q1mx * q4 + my * q1 * q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2 * q2 + my * q3 * q3 + _2q3 * q4 * mz - my * q4 * q4
        _2bx = np.sqrt(hx * hx + hy * hy)
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1 * q1 + _2q2mx * q4 - mz * q2 * q2 + _2q3 * q4 * my - mz * q3 * q3 + mz * q4 * q4
       
        _4bx = 2.0 * _2bx
        _4bz = 2.0 * _2bz
        
        # Gradient decent algorithm corrective step
        s1 = -_2q3 * (2.0 * q2 * q4 - _2q1 * q3 - ax) + _2q2 * (2.0 * q1 * q2 + _2q3 * q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3 * q3 - q4 * q4) + _2bz * (q2 * q4 - q1 * q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2 * q3 - q1 * q4) + _2bz * (q1 * q2 + q3 * q4) - my) + _2bx * q3 * (_2bx * (q1 * q3 + q2 * q4) + _2bz * (q3 * q4 - q1 * q2) - mz)
        s2 = _2q4 * (2.0 * q2 * q4 - _2q1 * q3 - ax) + _2q1 * (2.0 * q1 * q2 + _2q3 * q4 - ay) - 4.0 * q2 * (1.0 - 2.0 * q2 * q2 - 2.0 * q3 * q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3 * q3 - q4 * q4) + _2bz * (q2 * q4 - q1 * q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2 * q3 - q1 * q4) + _2bz * (q1 * q2 + q3 * q4) - my) + (_2bx * q2 - _4bz * q4) * (_2bx * (q1 * q3 + q2 * q4) + _2bz * (q3 * q4 - q1 * q2) - mz)
        s3 = -4.0 * q1 * q3 * (1.0 - 2.0 * q2 * q2 - 2.0 * q3 * q3 - az) + _2q2 * (2.0 * q1 * q2 + _2q3 * q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3 * q3 - q4 * q4) + _2bz * (q2 * q4 - q1 * q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2 * q3 - q1 * q4) + _2bz * (q1 * q2 + q3 * q4) - my) + _2bx * q3 * (_2bx * (q1 * q3 + q2 * q4) + _2bz * (q3 * q4 - q1 * q2) - mz)
        s4 = _2q1 * (2.0 * q2 * q4 - _2q1 * q3 - ax) + _2q2 * (2.0 * q1 * q2 + _2q3 * q4 - ay) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q2 * q3 - q1 * q4) + _2bz * (q1 * q2 + q3 * q4) - my) + (_2bx * q4 - _4bz * q4) * (_2bx * (q1 * q3 + q2 * q4) + _2bz * (q3 * q4 - q1 * q2) - mz)
        norm = np.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)
        if norm == 0.0:
            return  # Handle NaN
        s1 /= norm
        s2 /= norm
        s3 /= norm
        s4 /= norm

        # Compute rate of change of quaternion
        q_dot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        q_dot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        q_dot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        q_dot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

        # Integrate to yield quaternion
        q1 += q_dot1 * (1.0 / self.sample_freq)
        q2 += q_dot2 * (1.0 / self.sample_freq)
        q3 += q_dot3 * (1.0 / self.sample_freq)
        q4 += q_dot4 * (1.0 / self.sample_freq)
        norm = np.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
        if norm == 0.0:
            return  # Handle NaN
        self.q = np.array([q1 / norm, q2 / norm, q3 / norm, q4 / norm])


    def get_euler_angles(self):
        q1, q2, q3, q4 = self.q
        pitch = math.asin(-2.0 * (q2 * q4 - q1 * q3))
        roll = math.atan2(2.0 * (q1 * q2 + q3 * q4), 1.0 - 2.0 * (q2 * q2 + q3 * q3))
        yaw = math.atan2(2.0 * (q1 * q4 + q2 * q3), 1.0 - 2.0 * (q3 * q3 + q4 * q4))
        return pitch, roll, yaw