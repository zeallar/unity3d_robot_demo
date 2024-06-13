import math
from utils.logger import Logger

logger = Logger(__name__)

class PoseModule:
    def __init__(self):
        self.run = True
        self.use_mag = True
        self.a_x = self.a_y = self.a_z = 0.0
        self.g_x = self.g_y = self.g_z = 0.0
        self.m_x = self.m_y = self.m_z = 0.0
        self.error_ki = 0.0
        self.error_kp = 0.65
        self.correct_kp = 0.45
        self.error = [0.0, 0.0, 0.0]
        self.error_integral = [0.0, 0.0, 0.0]
        self.quaternion = [1.0, 0.0, 0.0, 0.0]
        self.rotate_matrix = [[0.0] * 3 for _ in range(3)]
        self.mag_world = [0.0, 0.0, 0.0]
        self.acc_world = [0.0, 0.0, 0.0]
        self.mag_correct = [0.0, 0.0, 0.0]
        self.acc_correct = [0.0, 0.0, 0.0]
        self.gyro_correct = [0.0, 0.0, 0.0]
        self.pit = self.rol = self.yaw = 0.0

    def calculatePose_Module(self, cycle, sensitivity):
        if not self.run:
            return self.quaternion

        length = math.sqrt(self.a_x ** 2 + self.a_y ** 2 + self.a_z ** 2)
        if length != 0:
            acc_tmp = [self.a_x / length, self.a_y / length, self.a_z / length]
        else:
            return self.quaternion

        if all(abs(val) < 1050 for val in [self.a_x, self.a_y, self.a_z]):
            if 800.0 < length < 1200.0:
                error = [
                    acc_tmp[1] * self.rotate_matrix[2][2] - acc_tmp[2] * self.rotate_matrix[1][2],
                    acc_tmp[2] * self.rotate_matrix[0][2] - acc_tmp[0] * self.rotate_matrix[2][2],
                    acc_tmp[0] * self.rotate_matrix[1][2] - acc_tmp[1] * self.rotate_matrix[0][2]
                ]

                self.error = [self.error[i] + 3.14 * cycle * (error[i] - self.error[i]) for i in range(3)]

            self.error_integral = [self.error_integral[i] + self.error[i] * self.error_ki * cycle for i in range(3)]
            self.error_integral = [min(max(val, -0.035), 0.035) for val in self.error_integral]

            self.gyro_correct = [
                (self.g_x - self.rotate_matrix[0][2] * self.correct_kp) * 0.01745329 + (self.error_kp * self.error[0] + self.error_integral[0]),
                (self.g_y - self.rotate_matrix[1][2] * self.correct_kp) * 0.01745329 + (self.error_kp * self.error[1] + self.error_integral[1]),
                (self.g_z - self.rotate_matrix[2][2] * self.correct_kp) * 0.01745329 + (self.error_kp * self.error[2] + self.error_integral[2])
            ]

            q_dot = [
                0.5 * (-self.quaternion[1] * self.gyro_correct[0] - self.quaternion[2] * self.gyro_correct[1] - self.quaternion[3] * self.gyro_correct[2]),
                0.5 * (self.quaternion[0] * self.gyro_correct[0] + self.quaternion[2] * self.gyro_correct[2] - self.quaternion[3] * self.gyro_correct[1]),
                0.5 * (self.quaternion[0] * self.gyro_correct[1] - self.quaternion[1] * self.gyro_correct[2] + self.quaternion[3] * self.gyro_correct[0]),
                0.5 * (self.quaternion[0] * self.gyro_correct[2] + self.quaternion[1] * self.gyro_correct[1] - self.quaternion[2] * self.gyro_correct[0])
            ]
            
            self.quaternion = [self.quaternion[i] + q_dot[i] * cycle * sensitivity for i in range(4)]
            norm = math.sqrt(sum(val ** 2 for val in self.quaternion))
            self.quaternion = [val / norm for val in self.quaternion]

            self.rotate_matrix[0][0] = self.quaternion[0] ** 2 + self.quaternion[1] ** 2 - self.quaternion[2] ** 2 - self.quaternion[3] ** 2
            self.rotate_matrix[0][1] = 2 * (self.quaternion[1] * self.quaternion[2] + self.quaternion[0] * self.quaternion[3])
            self.rotate_matrix[0][2] = 2 * (self.quaternion[1] * self.quaternion[3] - self.quaternion[0] * self.quaternion[2])
            self.rotate_matrix[1][0] = 2 * (self.quaternion[1] * self.quaternion[2] - self.quaternion[0] * self.quaternion[3])
            self.rotate_matrix[1][1] = self.quaternion[0] ** 2 - self.quaternion[1] ** 2 + self.quaternion[2] ** 2 - self.quaternion[3] ** 2
            self.rotate_matrix[1][2] = 2 * (self.quaternion[2] * self.quaternion[3] + self.quaternion[0] * self.quaternion[1])
            self.rotate_matrix[2][0] = 2 * (self.quaternion[1] * self.quaternion[3] + self.quaternion[0] * self.quaternion[2])
            self.rotate_matrix[2][1] = 2 * (self.quaternion[2] * self.quaternion[3] - self.quaternion[0] * self.quaternion[1])
            self.rotate_matrix[2][2] = self.quaternion[0] ** 2 - self.quaternion[1] ** 2 - self.quaternion[2] ** 2 + self.quaternion[3] ** 2

            self.acc_world = [
                sum(self.rotate_matrix[i][j] * acc_tmp[j] for j in range(3)) for i in range(3)
            ]

            self.pit = -math.asin(self.rotate_matrix[0][2])
            self.rol = math.atan2(self.rotate_matrix[2][2], self.rotate_matrix[1][2])
            self.yaw = math.atan2(self.rotate_matrix[0][0], self.rotate_matrix[0][1])

            self.acc_correct = [
                self.acc_world[0] * math.cos(self.yaw) + self.acc_world[1] * math.sin(self.yaw),
                -self.acc_world[0] * math.sin(self.yaw) + self.acc_world[1] * math.cos(self.yaw),
                self.acc_world[2]
            ]

            if self.use_mag:
                mag_tmp = [self.m_x, self.m_y, self.m_z]
                length = math.sqrt(sum(val ** 2 for val in mag_tmp))
                if length != 0:
                    ref_v = self.rotate_matrix[2]
                    self.simple_3d_trans(ref_v, mag_tmp, self.mag_correct)

        return self.quaternion

    def simple_3d_trans(self, ref, in_vector, out_vector):
        h_tmp_x = math.sqrt(ref[2] ** 2 + ref[1] ** 2)
        h_tmp_y = math.sqrt(ref[2] ** 2 + ref[0] ** 2)
        pn = -1 if ref[2] < 0 else 1
        out_vector[0] = (h_tmp_x * in_vector[0] - pn * ref[0] * in_vector[2])
        out_vector[1] = (pn * h_tmp_y * in_vector[1] - ref[1] * in_vector[2])
        out_vector[2] = ref[0] * in_vector[0] + ref[1] * in_vector[1] + ref[2] * in_vector[2]

    #加入了计算四元数导数 q_dot 和使用梯度下降法更新四元数的
    def calculatePose_Module_v2(self, cycle, sensitivity):
            if not self.run:
                return self.quaternion

            length = math.sqrt(self.a_x ** 2 + self.a_y ** 2 + self.a_z ** 2)
            if length != 0:
                acc_tmp = [self.a_x / length, self.a_y / length, self.a_z / length]
            else:
                return self.quaternion

            q = self.quaternion
            g = [self.g_x, self.g_y, self.g_z]
            acc = acc_tmp

            if all(abs(val) < 1050 for val in [self.a_x, self.a_y, self.a_z]):
                if 800.0 < length < 1200.0:
                    error = [
                        acc[1] * self.rotate_matrix[2][2] - acc[2] * self.rotate_matrix[1][2],
                        acc[2] * self.rotate_matrix[0][2] - acc[0] * self.rotate_matrix[2][2],
                        acc[0] * self.rotate_matrix[1][2] - acc[1] * self.rotate_matrix[0][2]
                    ]

                    self.error = [self.error[i] + 3.14 * cycle * (error[i] - self.error[i]) for i in range(3)]

                self.error_integral = [self.error_integral[i] + self.error[i] * self.error_ki * cycle for i in range(3)]
                self.error_integral = [min(max(val, -0.035), 0.035) for val in self.error_integral]

                gyro_correct = [
                    (self.g_x - self.rotate_matrix[0][2] * self.correct_kp) * 0.01745329 + (self.error_kp * self.error[0] + self.error_integral[0]),
                    (self.g_y - self.rotate_matrix[1][2] * self.correct_kp) * 0.01745329 + (self.error_kp * self.error[1] + self.error_integral[1]),
                    (self.g_z - self.rotate_matrix[2][2] * self.correct_kp) * 0.01745329 + (self.error_kp * self.error[2] + self.error_integral[2])
                ]

                q_dot = [
                    0.5 * (-q[1] * gyro_correct[0] - q[2] * gyro_correct[1] - q[3] * gyro_correct[2]),
                    0.5 * (q[0] * gyro_correct[0] + q[2] * gyro_correct[2] - q[3] * gyro_correct[1]),
                    0.5 * (q[0] * gyro_correct[1] - q[1] * gyro_correct[2] + q[3] * gyro_correct[0]),
                    0.5 * (q[0] * gyro_correct[2] + q[1] * gyro_correct[1] - q[2] * gyro_correct[0])
                ]

                q_new = [q[i] + q_dot[i] * cycle * sensitivity for i in range(4)]
                norm = math.sqrt(sum(val ** 2 for val in q_new))
                q_new = [val / norm for val in q_new]
                self.quaternion = q_new

                self.rotate_matrix[0][0] = q_new[0] ** 2 + q_new[1] ** 2 - q_new[2] ** 2 - q_new[3] ** 2
                self.rotate_matrix[0][1] = 2 * (q_new[1] * q_new[2] + q_new[0] * q_new[3])
                self.rotate_matrix[0][2] = 2 * (q_new[1] * q_new[3] - q_new[0] * q_new[2])
                self.rotate_matrix[1][0] = 2 * (q_new[1] * q_new[2] - q_new[0] * q_new[3])
                self.rotate_matrix[1][1] = q_new[0] ** 2 - q_new[1] ** 2 + q_new[2] ** 2 - q_new[3] ** 2
                self.rotate_matrix[1][2] = 2 * (q_new[2] * q_new[3] + q_new[0] * q_new[1])
                self.rotate_matrix[2][0] = 2 * (q_new[1] * q_new[3] + q_new[0] * q_new[2])
                self.rotate_matrix[2][1] = 2 * (q_new[2] * q_new[3] - q_new[0] * q_new[1])
                self.rotate_matrix[2][2] = q_new[0] ** 2 - q_new[1] ** 2 - q_new[2] ** 2 + q_new[3] ** 2

                self.acc_world = [
                    sum(self.rotate_matrix[i][j] * acc[j] for j in range(3)) for i in range(3)
                ]

                self.pit = -math.asin(self.rotate_matrix[0][2])
                self.rol = math.atan2(self.rotate_matrix[2][2], self.rotate_matrix[1][2])
                self.yaw = math.atan2(self.rotate_matrix[0][0], self.rotate_matrix[0][1])

                self.acc_correct = [
                    self.acc_world[0] * math.cos(self.yaw) + self.acc_world[1] * math.sin(self.yaw),
                    -self.acc_world[0] * math.sin(self.yaw) + self.acc_world[1] * math.cos(self.yaw),
                    self.acc_world[2]
                ]

                if self.use_mag:
                    mag_tmp = [self.m_x, self.m_y, self.m_z]
                    length = math.sqrt(sum(val ** 2 for val in mag_tmp))
                    if length != 0:
                        ref_v = self.rotate_matrix[2]
                        self.simple_3d_trans(ref_v, mag_tmp, self.mag_correct)

            return self.quaternion
    