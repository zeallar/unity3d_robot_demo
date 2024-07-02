#VOFA角度模式
#算法参考：https://github.com/769484623/MahonyAHRSWithMPU9250/blob/master/USR/MahonyAHRS.c
# Madgwick's implementation of Mayhony's AHRS algorithm.
# See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
# Algorithm paper:
# http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=4608934&url=http%3A%2F%2Fieeexplore.ieee.org%2Fstamp%2Fstamp.jsp%3Ftp%3D%26arnumber%3D4608934
import numpy as np

class MahonyAHRS:
    def __init__(self, sample_frequency=512.0):
        self.twoKp = 1500.0 * 0.5
        self.twoKi = 0.01 * 1.0
        self.q0, self.q1, self.q2, self.q3 = 1.0, 0.0, 0.0, 0.0
        self.integralFBx, self.integralFBy, self.integralFBz = 0.0, 0.0, 0.0
        self.invSampleFreq = 1.0 / sample_frequency
        self.angles_computed = False
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0

    def inv_sqrt(self, x):
        halfx = 0.5 * x
        y = x
        i = np.frombuffer(np.float32(y).tobytes(), dtype=np.int32)
        i = 0x5f3759df - (i >> 1)
        y = np.frombuffer(i.tobytes(), dtype=np.float32)
        y = y * (1.5 - (halfx * y * y))
        y = y * (1.5 - (halfx * y * y))
        return y

    def update(self, gx, gy, gz, ax, ay, az, mx, my, mz):
        gx *= np.pi / 180.0
        gy *= np.pi / 180.0
        gz *= np.pi / 180.0

        if not ((ax == 0.0) and (ay == 0.0) and (az == 0.0)):
            recip_norm = self.inv_sqrt(ax * ax + ay * ay + az * az)
            ax *= recip_norm
            ay *= recip_norm
            az *= recip_norm

            recip_norm = self.inv_sqrt(mx * mx + my * my + mz * mz)
            mx *= recip_norm
            my *= recip_norm
            mz *= recip_norm

            q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
            q0q0, q0q1, q0q2, q0q3 = q0*q0, q0*q1, q0*q2, q0*q3
            q1q1, q1q2, q1q3 = q1*q1, q1*q2, q1*q3
            q2q2, q2q3 = q2*q2, q2*q3
            q3q3 = q3*q3

            hx = 2.0 * (mx * (0.5 - q2q2 - q3q3) + my * (q1q2 - q0*q3) + mz * (q1*q3 + q0*q2))
            hy = 2.0 * (mx * (q1*q2 + q0*q3) + my * (0.5 - q1q1 - q3q3) + mz * (q2*q3 - q0*q1))
            bx = np.sqrt(hx * hx + hy * hy)
            bz = 2.0 * (mx * (q1*q3 - q0*q2) + my * (q2*q3 + q0*q1) + mz * (0.5 - q1*q1 - q2q2))

            halfvx = q1*q3 - q0*q2
            halfvy = q0*q1 + q2*q3
            halfvz = q0*q0 - 0.5 + q3*q3
            halfwx = bx * (0.5 - q2q2 - q3q3) + bz * (q1*q3 - q0*q2)
            halfwy = bx * (q1*q2 - q0*q3) + bz * (q0*q1 + q2*q3)
            halfwz = bx * (q0*q2 + q1*q3) + bz * (0.5 - q1*q1 - q2*q2)

            halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy)
            halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz)
            halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx)

            if self.twoKi > 0.0:
                self.integralFBx += self.twoKi * halfex * self.invSampleFreq
                self.integralFBy += self.twoKi * halfey * self.invSampleFreq
                self.integralFBz += self.twoKi * halfez * self.invSampleFreq
                gx += self.integralFBx
                gy += self.integralFBy
                gz += self.integralFBz
            else:
                self.integralFBx = 0.0
                self.integralFBy = 0.0
                self.integralFBz = 0.0

            gx += self.twoKp * halfex
            gy += self.twoKp * halfey
            gz += self.twoKp * halfez

        gx *= 0.5 * self.invSampleFreq
        gy *= 0.5 * self.invSampleFreq
        gz *= 0.5 * self.invSampleFreq
        qa, qb, qc = self.q0, self.q1, self.q2
        self.q0 += (-qb * gx - qc * gy - self.q3 * gz)
        self.q1 += (qa * gx + qc * gz - self.q3 * gy)
        self.q2 += (qa * gy - qb * gz + self.q3 * gx)
        self.q3 += (qa * gz + qb * gy - qc * gx)

        recip_norm = self.inv_sqrt(self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)
        self.q0 *= recip_norm
        self.q1 *= recip_norm
        self.q2 *= recip_norm
        self.q3 *= recip_norm
        self.angles_computed = False

    def compute_angles(self):
        self.roll = float(np.arctan2(self.q0 * self.q1 + self.q2 * self.q3, 0.5 - self.q1 * self.q1 - self.q2 * self.q2))
        self.pitch = float(np.arcsin(-2.0 * (self.q1 * self.q3 - self.q0 * self.q2)))
        self.yaw = float(np.arctan2(self.q1 * self.q2 + self.q0 * self.q3, 0.5 - self.q2 * self.q2 - self.q3 * self.q3))
        self.angles_computed = True

    def get_roll(self):
        if not self.angles_computed:
            self.compute_angles()
        return self.roll * 57.29578

    def get_pitch(self):
        if not self.angles_computed:
            self.compute_angles()
        return self.pitch * 57.29578

    def get_yaw(self):
        if not self.angles_computed:
            self.compute_angles()
        return self.yaw * 57.29578 + 180.0


