#这个版本可以，但是会瞬时变化
import time
import threading
import math
from algorithm.madgwickFilter import MadgwickAHRS
from data.data_retrieval import DataRetrieval  # 假设这个模块用于从串口获取数据
from transmit.tcp_server import TCPServer
from algorithm.pose_calculation import PoseModule
from algorithm.madgwick import Madgwick
#from algorithm.madgwick_ahrs import Madgwick
import numpy as np
from data import filters 
from data.window_filter import WindowFilter
from data.kalman_filter import MultiAxisKalmanFilter
from algorithm.MahonyAHRS import MahonyAHRS
from utils.logger import Logger
logger = Logger(__name__)
# Constants
GRAVITY = 9.802
#这是手机的rad/s
# GYRO_X_OFFSET = 0.00337001938739419
# GYRO_Y_OFFSET = 0.0005921793470098631
# GYRO_Z_OFFSET = 0.00255169917738042
# ACCEL_X_OFFSET = 0.006626719665527345
# ACCEL_Y_OFFSET = 0.041801074218750005
# ACCEL_Z_OFFSET = 9.8
#这个是产品的rad/s
# GYRO_X_OFFSET = 0.020479999999999998
# GYRO_Y_OFFSET = -0.06695000000000002
# GYRO_Z_OFFSET = -0.11270000000000004
# ACCEL_X_OFFSET = -0.28642000000000006
# ACCEL_Y_OFFSET = 0.0023799999999999997
# ACCEL_Z_OFFSET = 9.975689999999998
#这个是产品的度/s
GYRO_X_OFFSET = 4.337833333333333
GYRO_Y_OFFSET = -3.3301666666666665
GYRO_Z_OFFSET = -2.0822000000000003
ACCEL_X_OFFSET = -0.28642000000000006
ACCEL_Y_OFFSET = 0.0023799999999999997
ACCEL_Z_OFFSET = 9.975689999999998
#这个是产品的度/s * (3.14159265358979323846 / 180.0)
# GYRO_X_OFFSET = 0.07207357859531774
# GYRO_Y_OFFSET = -0.05832775919732442
# GYRO_Z_OFFSET = -0.034314381270903006
# ACCEL_X_OFFSET = -0.28642000000000006
# ACCEL_Y_OFFSET = 0.0023799999999999997
# ACCEL_Z_OFFSET = 9.975689999999998
#这是手机的
# MAGN_ELLIPSOID_CENTER = [-0.010116, 0.0202537, 0.988309]
# MAGN_ELLIPSOID_TRANSFORM = [
#     [0.97269, 0.0124029, -0.00955096],
#     [0.0124029, 0.994026, 6.33932e-05],
#     [-0.00955096, 6.33932e-05, 0.943093]
# ]
#这是产品的v1
# MAGN_ELLIPSOID_CENTER = [-6.03386, -10.1709, -7.15779]
# MAGN_ELLIPSOID_TRANSFORM = [
#     [0.968578, 0.000440177, 0.0177124],
#     [0.000440177, 0.994927, -0.0111642],
#     [0.0177124, -0.0111642, 0.9665]
# ]
#这是产品的v2
MAGN_ELLIPSOID_CENTER = [-5.96847, -8.53266, -6.55064]
MAGN_ELLIPSOID_TRANSFORM = [
    [0.974685, -0.00672609, -0.000430064],
    [-0.00672609, 0.996443, 0.00602083],
    [-0.000430064, 0.00602083, 0.978725]
]
def get_euler_angles():
    return pose.rol, pose.pit, pose.yaw
def get_quaternion():
    return pose.quaternion
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
def quaternion_to_euler(w, x, y, z):
    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

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
    # accel[2] -= (ACCEL_Z_OFFSET)

    # Compensate magnetometer error
    #magnetom_tmp = [magnetom[i] - MAGN_ELLIPSOID_CENTER[i] for i in range(3)]
    #magnetom = matrix_vector_multiply(MAGN_ELLIPSOID_TRANSFORM, magnetom_tmp)

    # Compensate gyroscope error
    gyro[0] -= GYRO_X_OFFSET
    gyro[1] -= GYRO_Y_OFFSET
    gyro[2] -= GYRO_Z_OFFSET

    return accel, magnetom, gyro

def read_sensors():
    sensor_data = data_retrieval.read_json_data()
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
    mahony_ahrs = MahonyAHRS()
    madgwick = Madgwick()
    global data_retrieval 
    data_retrieval = DataRetrieval()
    #filter = WindowFilter(window_size=10)  # 创建窗口滤波器对象.
    # 初始化卡尔曼滤波器
    kf = MultiAxisKalmanFilter()

    while True:
        accel, magnetom, gyro = read_sensors()
        accel, magnetom, gyro = compensate_sensor_errors(accel, magnetom, gyro)

        time_now = time.time()
        deltat = time_now - time_former
        time_former = time_now
        acc_scale=100
        mag_scale=1000
        # pose.a_x, pose.a_y, pose.a_z=accel[0]*acc_scale, accel[1]*acc_scale, accel[2]*acc_scale
        pose.a_x, pose.a_y, pose.a_z=accel[0], accel[1], accel[2]
        pose.g_x, pose.g_y, pose.g_z=gyro[0], gyro[1], gyro[2]
        
        # pose.m_x, pose.m_y, pose.m_z=magnetom[0]*mag_scale, magnetom[1]*mag_scale, magnetom[2]*mag_scale
        pose.m_x, pose.m_y, pose.m_z=magnetom[0], magnetom[1], magnetom[2]


        # pose.a_x, pose.a_y, pose.a_z=preprocess_gyro_data(pose.a_x, pose.a_y, pose.a_z,0.5)
        # pose.g_x, pose.g_y, pose.g_z=preprocess_gyro_data(pose.g_x, pose.g_y, pose.g_z,0.5)
        # if abs(pose.a_z) < 0.5:
        #     pose.a_z = 9.8
        # pose.m_x, pose.m_y, pose.m_z=40,1,-21
        # pose.m_x, pose.m_y, pose.m_z=-18.8,30.1,-15.5

        # ahrs.update(pose.a_x, pose.a_y, pose.a_z,pose.g_x, pose.g_y, pose.g_z,pose.m_x, pose.m_y, pose.m_z, deltat)
        # q=ahrs.get_quaternion()
        # pose.quaternion = list(q)
        # roll, pitch, yaw= ahrs.quaternion_to_euler()

        mahony_ahrs.update(pose.g_x, pose.g_y, pose.g_z,pose.a_x, pose.a_y, pose.a_z,pose.m_x, pose.m_y, pose.m_z)
         # 获取姿态角度
        roll = mahony_ahrs.get_roll()
        pitch = mahony_ahrs.get_pitch()
        yaw = mahony_ahrs.get_yaw()
        # # 处理数据并存储结果
        # filtered_data = kf.apply_filter(roll, pitch, yaw)
        # send_to_pc(filtered_data[0], filtered_data[1], filtered_data[2])
        send_to_pc(roll, pitch, yaw)

        # acc_data = np.array([pose.a_x, pose.a_y, pose.a_z])
        # gyro_data = np.array([pose.g_x, pose.g_y, pose.g_z])
        # mag_data = np.array([pose.m_x, pose.m_y, pose.m_z])
        # updated_quaternion = madgwick.updateMARG(madgwick.q0, gyro_data, acc_data, mag_data,dt=deltat)
        # madgwick.q0=updated_quaternion
        # roll, pitch, yaw = madgwick.quaternion_to_euler()
        # send_to_pc(roll, pitch, yaw)
        # logger.info(f"Original data - Acc: [{pose.a_x}, {pose.a_y}, {pose.a_z}], Gyro: [{pose.g_x}, {pose.g_y}, {pose.g_z}], Mag: [{pose.m_x}, {pose.m_y}, {pose.m_z}]")
        # pose.calculatePose_Module(deltat,1)

        time.sleep(0.01)

if __name__ == "__main__":
    # tcp_thread = threading.Thread(target=TCPServer().start, args=(get_nine_axis,))
    tcp_thread = threading.Thread(target=TCPServer().start, args=(get_euler_angles,))
    # tcp_thread = threading.Thread(target=TCPServer().start, args=(get_quaternion,))
    
    tcp_thread.start()
    main()
