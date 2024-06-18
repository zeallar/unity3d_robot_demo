#备份测试版本
import time
import threading
import math
from algorithm.madgwickFilter import MadgwickAHRS
from data.data_retrieval import DataRetrieval  # 假设这个模块用于从串口获取数据
from transmit.tcp_server import TCPServer
from algorithm.pose_calculation import PoseModule
from algorithm.madgwick_v0 import Madgwick
from algorithm.origin_madgwick import Madgwick
import numpy as np
# Constants
GRAVITY = 9.802
GYRO_X_OFFSET = 0.00337001938739419
GYRO_Y_OFFSET = 0.0005921793470098631
GYRO_Z_OFFSET = 0.00255169917738042
ACCEL_X_OFFSET = 0.006626719665527345
ACCEL_Y_OFFSET = 0.041801074218750005
ACCEL_Z_OFFSET = 9.8
MAGN_ELLIPSOID_CENTER = [-0.010116, 0.0202537, 0.988309]
MAGN_ELLIPSOID_TRANSFORM = [
    [0.97269, 0.0124029, -0.00955096],
    [0.0124029, 0.994026, 6.33932e-05],
    [-0.00955096, 6.33932e-05, 0.943093]
]
def get_euler_angles():
    return pose.rol, pose.pit, pose.yaw
# AHRS initialization
def get_nine_axis():
    return pose.a_x, pose.a_y, pose.a_z,pose.g_x, pose.g_y, pose.g_z,pose.m_x, pose.m_y, pose.m_z

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
def EulerAndQuaternionTransform( q0, q1, q2, q3):
    """
        四元素与欧拉角互换
    """
    angle_is_not_rad = False
    w = q0 
    x = q1
    y = q2
    z = q3

    r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    p = math.asin(2 * (w * y - z * x))
    y = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    if angle_is_not_rad : # pi -> 180
        r = math.degrees(r)
        p = math.degrees(p)
        y = math.degrees(y)
    return [r,p,y]
def matrix_vector_multiply(a, b):
    return [a[0][0] * b[0] + a[0][1] * b[1] + a[0][2] * b[2],
            a[1][0] * b[0] + a[1][1] * b[1] + a[1][2] * b[2],
            a[2][0] * b[0] + a[2][1] * b[1] + a[2][2] * b[2]]

def compensate_sensor_errors(accel, magnetom, gyro):
    # Compensate accelerometer error
    accel[0] -= ACCEL_X_OFFSET
    accel[1] -= ACCEL_Y_OFFSET
    accel[2] -= (ACCEL_Z_OFFSET- GRAVITY )

    # Compensate magnetometer error
    #magnetom_tmp = [magnetom[i] - MAGN_ELLIPSOID_CENTER[i] for i in range(3)]
    #magnetom = matrix_vector_multiply(MAGN_ELLIPSOID_TRANSFORM, magnetom_tmp)

    # Compensate gyroscope error
    gyro[0] -= GYRO_X_OFFSET
    gyro[1] -= GYRO_Y_OFFSET
    gyro[2] -= GYRO_Z_OFFSET

    return accel, magnetom, gyro

def read_sensors():
    sensor_data = data_retrieval.get_data_from_phyphox()
    accel = [sensor_data['acceleration']['x'], sensor_data['acceleration']['y'], sensor_data['acceleration']['z']]
    magnetom = [sensor_data['magnetometer']['x'], sensor_data['magnetometer']['y'], sensor_data['magnetometer']['z']]
    gyro = [sensor_data['gyroscope']['x'], sensor_data['gyroscope']['y'], sensor_data['gyroscope']['z']]
    #print("Magnetometer data:", magnetom)
    return accel, magnetom, gyro

def send_to_pc(data1, data2, data3):
    # Simulate sending data to PC (placeholder for actual implementation)
    pose.rol,pose.pit,pose.yaw=data1,data2,data3
    #print(f"Roll: {data1}, Pitch: {data2}, Yaw: {data3}")
def preprocess_gyro_data(g_x, g_y, g_z, threshold):
    if abs(g_x) < threshold:
        g_x = 0
    if abs(g_y) < threshold:
        g_y = 0
    if abs(g_z) < threshold:
        g_z = 0
    return g_x, g_y, g_z
def main():
    time_former = time.time()
    global pose
    pose = PoseModule()
    ahrs = MadgwickAHRS()
    madgwick = Madgwick(beta=0.1, dt=0.01)
    global data_retrieval 
    data_retrieval = DataRetrieval()
    initial_quaternion = np.array([1.0, 0.0, 0.0, 0.0])
    while True:
        accel, magnetom, gyro = read_sensors()
        accel, magnetom, gyro = compensate_sensor_errors(accel, magnetom, gyro)

        time_now = time.time()
        deltat = time_now - time_former
        time_former = time_now
        pose.a_x, pose.a_y, pose.a_z=accel[0], accel[1], accel[2]
        pose.g_x, pose.g_y, pose.g_z=gyro[0], gyro[1], gyro[2]
        pose.m_x, pose.m_y, pose.m_z=magnetom[0], magnetom[1], magnetom[2]

        # pose.a_x, pose.a_y, pose.a_z=preprocess_gyro_data(pose.a_x, pose.a_y, pose.a_z,0.15)
        # pose.g_x, pose.g_y, pose.g_z=preprocess_gyro_data(pose.g_x, pose.g_y, pose.g_z,0.5)

        #ahrs.update(accel[0], accel[1], accel[2],gyro[0], gyro[1], gyro[2], magnetom[0], magnetom[1], magnetom[2], deltat)
        #q = ahrs.get_quaternion()

        acc_data = np.array([[pose.a_x, pose.a_y, pose.a_z]])
        gyro_data = np.array([[pose.g_x, pose.g_y, pose.g_z]])
        mag_data = np.array([[pose.m_x, pose.m_y, pose.m_z]])
        # q = madgwick.updateMARG(gyr=gyro_data, acc=acc_data, mag=mag_data)

        # #qua = Quaternion(q[0], q[1], q[2], q[3])
        # eul=EulerAndQuaternionTransform(q[0], q[1], q[2], q[3])
        # send_to_pc(eul[0],eul[1], eul[2])
        # eul = to_euler_angles(qua)
        # eul.yaw += 0.8
        # if eul.yaw > math.pi:
        #     eul.yaw -= 2 * math.pi

        # send_to_pc(eul.roll, eul.pitch, eul.yaw)


        # 初始化 Madgwick 滤波器
        madgwick = Madgwick()
        # 初始化四元数，假设初始姿态为单位四元数
        
        # 估计方向
        q= madgwick.updateMARG(initial_quaternion, gyr=gyro_data[0], acc=acc_data[0], mag=mag_data[0])
        initial_quaternion = q
        eul=EulerAndQuaternionTransform(q[0], q[1], q[2], q[3])
        send_to_pc(eul[0],eul[1], eul[2])

        time.sleep(0.0005)

if __name__ == "__main__":
    # tcp_thread = threading.Thread(target=TCPServer().start, args=(get_nine_axis,))
    tcp_thread = threading.Thread(target=TCPServer().start, args=(get_euler_angles,))
    tcp_thread.start()
    main()
