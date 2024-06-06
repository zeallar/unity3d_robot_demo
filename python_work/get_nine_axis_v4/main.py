import threading
import logging
import time
from data.data_retrieval import DataRetrieval
from data.data_structure import Data
from data.calibration import Calibration
from data.window_filter import WindowFilter
from algorithm.pose_calculation import PoseModule
from transmit.tcp_server import TCPServer
from utils.logger import Logger
logger = Logger(__name__)


def get_euler_angles():
    return pose.rol, pose.pit, pose.yaw

def main():
    print("Starting data logging...")
    
    data_retrieval = DataRetrieval()
    data = Data()
    global pose
    pose = PoseModule()
    calibration = Calibration()
    window_filter = WindowFilter(window_size=10)

    # Choose data source
    source = input("Select data source (1: Phyphox, 2: UDP): ")
    
    if source not in ('1', '2'):
        print("Invalid selection.")
        logger.error("Invalid data source selection.")
        return

    # Choose whether to calibrate
    calibrate = input("Do you want to calibrate the sensor data? (y/n): ")

    if calibrate.lower() == 'y':
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

        if sensor_data:
            if calibrate.lower() == 'y':
                sensor_data = calibration.apply_calibration(sensor_data)
            window_filter.add_data(sensor_data)
            filtered_data = window_filter.get_filtered_data()

            pose.a_x, pose.a_y, pose.a_z = filtered_data['acceleration']['x'], filtered_data['acceleration']['y'], filtered_data['acceleration']['z']
            pose.g_x, pose.g_y, pose.g_z = filtered_data['gyroscope']['x'], filtered_data['gyroscope']['y'], filtered_data['gyroscope']['z']
            pose.m_x, pose.m_y, pose.m_z = filtered_data['magnetometer']['x'], filtered_data['magnetometer']['y'], filtered_data['magnetometer']['z']

            pose.madgwick_update(0.01,2.6)
            logger.debug(f"Updated pose: Roll={pose.rol}, Pitch={pose.pit}, Yaw={pose.yaw}")

        time.sleep(0.01)

if __name__ == "__main__":
    tcp_thread = threading.Thread(target=TCPServer().start, args=(get_euler_angles,))
    tcp_thread.start()

    main()