import threading
import logging
import time
from data.data_retrieval import DataRetrieval
from data.data_structure import Data
from data.calibration import Calibration
from data.window_filter import WindowFilter
from algorithm.pose_calculation import PoseModule
from algorithm.madgwickFilter import MadgwickAHRS
from algorithm.madgwick_v0 import Madgwick
from algorithm.my_madgwick import My_Madgwick
from transmit.tcp_server import TCPServer
from utils.logger import Logger
import numpy as np

logger = Logger(__name__)

def get_euler_angles():
    return pose.rol, pose.pit, pose.yaw
def get_nine_axis():
    return pose.a_x, pose.a_y, pose.a_z,pose.g_x, pose.g_y, pose.g_z,pose.m_x, pose.m_y, pose.m_z

def main():
    print("Starting data logging...")
    
    data_retrieval = DataRetrieval()
    data = Data()
    global pose
    pose = PoseModule()
    calibration = Calibration()
    my_madgwick = My_Madgwick(beta=0.1, sample_freq=256.0)
    madgwick = Madgwick(beta=0.1, dt=0.01)
    madgwickFilter=MadgwickAHRS()
    window_filter = WindowFilter(window_size=10)

    # Choose data source
    source = input("Select data source (1: Phyphox, 2: UDP, 3: Serial): ")
    
    if source not in ('1', '2', '3'):
        print("Invalid selection.")
        logger.error("Invalid data source selection.")
        return

    # Choose whether to calibrate
    calibrate = input("Do you want to calibrate the acc gyr sensor data? (y/n): ")
    mag_calibrate= input("Do you want to calibrate the mag sensor data? (y/n): ")
    if calibrate.lower() == 'y' and mag_calibrate.lower() == 'n':
        if source == '1':
            for _ in range(300):  # 3 seconds of data at 100Hz
                sensor_data = data_retrieval.get_data_from_phyphox()
                if sensor_data:
                    calibration.add_data(sensor_data)
                    time.sleep(0.01)
            calibration.calculate_bias()
            logger.info("Calibration completed for Phyphox data source")
        elif source == '2':
            for _ in range(300):  # 3 seconds of data at 100Hz
                sensor_data = data_retrieval.get_data_from_udp()
                if sensor_data:
                    calibration.add_data(sensor_data)
                    time.sleep(0.01)
            calibration.calculate_bias()
            logger.info("Calibration completed for UDP data source")
        elif source == '3':
            data_thread = threading.Thread(target=data_collection_thread, args=(calibration, data_retrieval,source))
            data_thread.start()
            # 主线程实时可视化数据
            #calibration.visualize_mag_data()
            # 等待数据采集线程结束
            data_thread.join()
            calibration.calculate_bias()
            
            # mag_data=calibration.get_mag_data()
            # # 椭球拟合
            # params = calibration.ellipsoid_fit(mag_data)
            # # 校正数据
            # corrected_data = calibration.correct_magnetometer_data(mag_data,params)
            # # 可视化原始和校正后的数据
            # calibration.plot_magnetometer_data(mag_data, title='Original Magnetometer Data')
            # calibration.plot_magnetometer_data(corrected_data, title='Corrected Magnetometer Data')
            # #校准参数保存和加载函数
            # calibration.save_calibration_params(params)
            logger.info("Calibration completed for Serial data source")
        calibration.save_calibration_data("bias.txt")
    if mag_calibrate.lower() == 'y' and calibrate.lower() == 'n':
        data_thread = threading.Thread(target=data_collection_thread, args=(calibration, data_retrieval,source))
        data_thread.start()
        # 主线程实时可视化数据
        calibration.visualize_mag_data()
        # 等待数据采集线程结束
        data_thread.join()
        #保存采集到的数据
        #calibration.save_magnetometer_data_to_csv('magnetometer_data.csv')    
    elif calibrate.lower() == 'n':
        logger.info("Skipping calibration")
    else:
        print("Invalid selection.")
        logger.error("Invalid calibration selection.")
        return

    while True:
        if source == '1':
            sensor_data = data_retrieval.get_data_from_phyphox()
        elif source == '2':
            sensor_data = data_retrieval.get_data_from_udp()
        elif source == '3':
            sensor_data = data_retrieval.read_json_data()

        if sensor_data:
            if calibrate.lower() == 'y':
                sensor_data = calibration.apply_calibration(sensor_data)
            else:
                calibration.load_calibration_data("bias.txt")
                sensor_data = calibration.apply_calibration(sensor_data)
            window_filter.add_data(sensor_data)
            filtered_data = window_filter.get_filtered_data()

            pose.a_x, pose.a_y, pose.a_z = filtered_data['acceleration']['x'], filtered_data['acceleration']['y'], filtered_data['acceleration']['z']
            pose.g_x, pose.g_y, pose.g_z = filtered_data['gyroscope']['x'], filtered_data['gyroscope']['y'], filtered_data['gyroscope']['z']
            pose.m_x, pose.m_y, pose.m_z = filtered_data['magnetometer']['x'], filtered_data['magnetometer']['y'], filtered_data['magnetometer']['z']
          
            # pose.a_x, pose.a_y, pose.a_z=preprocess_gyro_data(pose.a_x, pose.a_y, pose.a_z,0.15)
            # pose.g_x, pose.g_y, pose.g_z=preprocess_gyro_data(pose.g_x, pose.g_y, pose.g_z,0.5)
            
            #pose.calculatePose_Module(0.01, 2.6)
            #Madgwick
            #my_madgwick.update_marg(pose.a_x, pose.a_y, pose.a_z,pose.g_x, pose.g_y, pose.g_z, pose.m_x, pose.m_y, pose.m_z)
            #pose.pit,pose.rol,pose.yaw=my_madgwick.get_euler_angles()
            #python madgwick,这个可以
            acc_data = np.array([pose.a_x, pose.a_y, pose.a_z])
            gyro_data = np.array([pose.g_x, pose.g_y, pose.g_z])
            mag_data = np.array([pose.m_x, pose.m_y, pose.m_z])
            #q_estimated = madgwick.updateMARG(gyr=gyro_data, acc=acc_data, mag=mag_data,sensitivity=0.05)
            # q_estimated = madgwick.updateMARG(gyr=gyro_data, acc=acc_data, mag=mag_data)
            # euler_angles =madgwick.get_euler_angles()
            # pose.pit,pose.rol,pose.yaw = euler_angles


            madgwickFilter.update(pose.a_x, pose.a_y, pose.a_z,pose.g_x, pose.g_y, pose.g_z, pose.m_x, pose.m_y, pose.m_z, 0.01)
            q = madgwickFilter.get_quaternion()
            pose.rol,pose.pit,  pose.yaw  = madgwickFilter.quaternion_to_euler()

            # q_estimated = madgwick.updateMARG(gyr=gyro_data, acc=acc_data, mag=acc_data)
            # euler_angles =madgwick.get_euler_angles()
            #pose.pit,pose.rol,  pose.yaw = euler_angles
            #打印计算的四元数
            #print("Estimated Quaternion (MARG):", q_estimated)
            #logger.info(f"euler: {pose.a_x:.2f},{pose.a_y:.2f},{pose.a_z:.2f},{pose.g_x:.2f},{pose.g_y:.2f},{pose.g_z:.2f},{pose.m_x:.2f},{pose.m_y:.2f},{pose.m_z:.2f}\n")
            print(f"Updated pose: Roll={pose.rol}, Pitch={pose.pit}, Yaw={pose.yaw}")

        time.sleep(0.01)
def data_collection_thread(calibration, data_retrieval,source):
    for _ in range(1000):  # 3 seconds of data at 100Hz
        if source == '1':
            sensor_data = data_retrieval.get_data_from_phyphox()
        elif source == '2':
            sensor_data = data_retrieval.get_data_from_udp()
        elif source == '3':
            sensor_data = data_retrieval.read_json_data()
        if sensor_data:
            # 保留小数点后两位
            for sensor, axes in sensor_data.items():
                for axis in axes:
                    sensor_data[sensor][axis] = round(axes[axis], 2)
            logger.info(f"sensor_data: {sensor_data}")
            calibration.add_data(sensor_data)
            time.sleep(0.01)  # 以10ms的间隔获取数据
def preprocess_gyro_data(g_x, g_y, g_z, threshold):
    if abs(g_x) < threshold:
        g_x = 0
    if abs(g_y) < threshold:
        g_y = 0
    if abs(g_z) < threshold:
        g_z = 0
    return g_x, g_y, g_z
if __name__ == "__main__":
    tcp_thread = threading.Thread(target=TCPServer().start, args=(get_nine_axis,))
    # tcp_thread = threading.Thread(target=TCPServer().start, args=(get_euler_angles,))
    tcp_thread.start()
    main()