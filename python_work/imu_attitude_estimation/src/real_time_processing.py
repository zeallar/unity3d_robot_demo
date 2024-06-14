import serial
import time
import matplotlib.pyplot as plt
from collections import deque
from .calibration import calibrate_data
from .high_low_pass_filter import high_low_pass_filter

class IMUProcessor:
    def __init__(self, port='COM30', baudrate=1000000):
        self.port = port
        self.baudrate = baudrate

    def get_data_from_serial(self):
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"在 {self.port} 端口以 {self.baudrate} 波特率建立串行连接")

            buffer = b''
            while True:
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    buffer += data
                    print(f"接收到串行数据: {data.hex()}")

                    while len(buffer) >= 22:
                        line = buffer[:22]
                        buffer = buffer[22:]
                        
                        result = self.parse_serial_data(line)
                        if result:
                            print(f"九轴原始数据: {result}")
                            physical_data = self.convert_to_physical_data(result)
                            print(f"处理数据行: {physical_data}")
                            return physical_data
        except serial.SerialException as e:
            print(f"串行连接错误: {e}")
            return None

    def parse_serial_data(self, data):
        if len(data) != 22:
            print("数据长度不正确")
            return None

        try:
            accel_x = int.from_bytes(data[0:2], byteorder='big', signed=True)
            accel_y = int.from_bytes(data[2:4], byteorder='big', signed=True)
            accel_z = int.from_bytes(data[4:6], byteorder='big', signed=True)
            gyro_x = int.from_bytes(data[6:8], byteorder='big', signed=True)
            gyro_y = int.from_bytes(data[8:10], byteorder='big', signed=True)
            gyro_z = int.from_bytes(data[10:12], byteorder='big', signed=True)
            mag_x = int.from_bytes(data[12:14], byteorder='big', signed=True)
            mag_y = int.from_bytes(data[14:16], byteorder='big', signed=True)
            mag_z = int.from_bytes(data[16:18], byteorder='big', signed=True)

            result = {
                'acceleration': [self.convert_signed(accel_x), self.convert_signed(accel_y), self.convert_signed(accel_z)],
                'gyroscope': [self.convert_signed(gyro_x), self.convert_signed(gyro_y), self.convert_signed(gyro_z)],
                'magnetometer': [self.convert_signed(mag_x), self.convert_signed(mag_y), self.convert_signed(mag_z)]
            }
            return result
        except Exception as e:
            print(f"解析串行数据出错: {e}")
            return None

    def convert_signed(self, value):
        if value > 32767:
            value -= 65536
        return value

    def convert_to_physical_data(self, data):
        acc = data['acceleration']
        gyro = data['gyroscope']
        mag = data['magnetometer']
        
        accel_sensitivity = 0.061 * 0.001 * 9.81  # 单位：m/s²/LSB
        acc_x = acc[0] * accel_sensitivity
        acc_y = acc[1] * accel_sensitivity
        acc_z = acc[2] * accel_sensitivity

        gyro_sensitivity = 17.50 * 0.001  # 单位：rad/s/LSB
        gyro_x = gyro[0] * gyro_sensitivity  # 转换为 rad/s
        gyro_y = gyro[1] * gyro_sensitivity  # 转换为 rad/s
        gyro_z = gyro[2] * gyro_sensitivity  # 转换为 rad/s

        mag_scale = 1.0  # 单位：μT，根据实际情况调整
        mag_x = mag[0] * mag_scale
        mag_y = mag[1] * mag_scale
        mag_z = mag[2] * mag_scale

        physical_data = {
            'acceleration': {'x': acc_x, 'y': acc_y, 'z': acc_z},
            'gyroscope': {'x': gyro_x, 'y': gyro_y, 'z': gyro_z},
            'magnetometer': {'x': mag_x, 'y': mag_y, 'z': mag_z}
        }
        return physical_data

def real_time_processing():
    # 设置校准参数（根据实际情况设置）
    Ta = np.eye(3)
    Ka = np.eye(3)
    Ba = np.zeros(3)
    Tg = np.eye(3)
    Kg = np.eye(3)
    Bg = np.zeros(3)
    Tm2a = np.eye(3)
    Bm = np.zeros(3)

    # 初始化
    Qfuse1 = np.array([1, 0, 0, 0])
    QfuseHL = deque(maxlen=500)
    timestamps = deque(maxlen=500)

    # 实时接收数据
    imu_processor = IMUProcessor()
    
    try:
        plt.ion()
        fig, axs = plt.subplots(4, 1, figsize=(10, 8))
        
        while True:
            data = imu_processor.get_data_from_serial()
            if data:
                timestamp = time.time()
                
                # 校准数据
                acc = [data['acceleration']['x'], data['acceleration']['y'], data['acceleration']['z']]
                gyro = [data['gyroscope']['x'], data['gyroscope']['y'], data['gyroscope']['z']]
                mag = [data['magnetometer']['x'], data['magnetometer']['y'], data['magnetometer']['z']]
                ImuData = [timestamp] + acc + gyro + mag
                
                if len(QfuseHL) > 0:
                    t = timestamp - timestamps[-1]
                else:
                    t = 0.01  # 假设初始时间间隔为0.01秒
                
                Qfuse1 = high_low_pass_filter(Qfuse1, ImuData, t)
                QfuseHL.append(Qfuse1)
                timestamps.append(timestamp)
                
                # 实时绘图
                if len(QfuseHL) > 10:
                    axs[0].cla()
                    axs[1].cla()
                    axs[2].cla()
                    axs[3].cla()
                    
                    axs[0].plot(list(QfuseHL)[:, 0], 'r')
                    axs[0].set_ylim([-1, 1])
                    axs[0].set_ylabel('q0')
                    
                    axs[1].plot(list(QfuseHL)[:, 1], 'g')
                    axs[1].set_ylim([-1, 1])
                    axs[1].set_ylabel('q1')
                    
                    axs[2].plot(list(QfuseHL)[:, 2], 'b')
                    axs[2].set_ylim([-1, 1])
                    axs[2].set_ylabel('q2')
                    
                    axs[3].plot(list(QfuseHL)[:, 3], 'm')
                    axs[3].set_ylim([-1, 1])
                    axs[3].set_ylabel('q3')
                    axs[3].set_xlabel('Samples')
                    
                    plt.pause(0.01)
    except KeyboardInterrupt:
        print("实时接收IMU数据结束")
    finally:
        plt.ioff()
        plt.show()
