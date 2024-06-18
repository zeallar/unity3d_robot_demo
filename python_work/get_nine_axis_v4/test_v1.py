#这个版本有问题，yaw角变化异常
import time
import math
from madgwickahrs import MadgwickAHRS
import data_retrieval  # 假设这个模块用于从串口获取数据

# Constants
GRAVITY = 9.802
GYRO_X_OFFSET = 0.0000820
GYRO_Y_OFFSET = -0.0002375
GYRO_Z_OFFSET = -0.0000904
ACCEL_X_OFFSET = 0.1405817
ACCEL_Y_OFFSET = -0.1235667
ACCEL_Z_OFFSET = -10.2402658
MAGN_ELLIPSOID_CENTER = [-1.22362, -3.49591, -28.3068]
MAGN_ELLIPSOID_TRANSFORM = [
    [0.936683, -0.0120599, -0.00747369],
    [-0.0120599, 0.997691, -5.88781e-05],
    [-0.00747369, -5.88781e-05, 0.846255]
]

# AHRS initialization
ahrs = MadgwickAHRS()

# Structs for converting quaternion to Euler angles
class Quaternion:
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

class EulerAngles:
    def __init__(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

def to_euler_angles(q):
    angles = EulerAngles(0, 0, 0)

    # roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    angles.roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        angles.pitch = math.copysign(math.pi / 2, sinp)
    else:
        angles.pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    angles.yaw = math.atan2(siny_cosp, cosy_cosp)

    return angles

def matrix_vector_multiply(a, b):
    return [a[0][0] * b[0] + a[0][1] * b[1] + a[0][2] * b[2],
            a[1][0] * b[0] + a[1][1] * b[1] + a[1][2] * b[2],
            a[2][0] * b[0] + a[2][1] * b[1] + a[2][2] * b[2]]

def compensate_sensor_errors(accel, magnetom, gyro):
    # Compensate accelerometer error
    accel[0] -= ACCEL_X_OFFSET
    accel[1] -= ACCEL_Y_OFFSET
    accel[2] -= (ACCEL_Z_OFFSET + GRAVITY)

    # Compensate magnetometer error
    magnetom_tmp = [magnetom[i] - MAGN_ELLIPSOID_CENTER[i] for i in range(3)]
    magnetom = matrix_vector_multiply(MAGN_ELLIPSOID_TRANSFORM, magnetom_tmp)

    # Compensate gyroscope error
    gyro[0] -= GYRO_X_OFFSET
    gyro[1] -= GYRO_Y_OFFSET
    gyro[2] -= GYRO_Z_OFFSET

    return accel, magnetom, gyro

def read_sensors():
    sensor_data = data_retrieval.get_data_from_serial()
    accel = [sensor_data['acceleration']['x'], sensor_data['acceleration']['y'], sensor_data['acceleration']['z']]
    magnetom = [sensor_data['magnetometer']['x'], sensor_data['magnetometer']['y'], sensor_data['magnetometer']['z']]
    gyro = [sensor_data['gyroscope']['x'], sensor_data['gyroscope']['y'], sensor_data['gyroscope']['z']]
    return accel, magnetom, gyro

def send_to_pc(data1, data2, data3):
    # Simulate sending data to PC (placeholder for actual implementation)
    print(f"Roll: {data1}, Pitch: {data2}, Yaw: {data3}")

def main():
    time_former = time.time()

    while True:
        accel, magnetom, gyro = read_sensors()
        accel, magnetom, gyro = compensate_sensor_errors(accel, magnetom, gyro)

        time_now = time.time()
        deltat = time_now - time_former
        time_former = time_now

        ahrs.update(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], magnetom[0], magnetom[1], magnetom[2], deltat)
        q = ahrs.quaternion

        qua = Quaternion(q[0], q[1], q[2], q[3])
        eul = to_euler_angles(qua)
        eul.yaw += 0.8
        if eul.yaw > math.pi:
            eul.yaw -= 2 * math.pi

        send_to_pc(eul.roll, eul.pitch, eul.yaw)
        time.sleep(0.0005)

if __name__ == "__main__":
    main()
