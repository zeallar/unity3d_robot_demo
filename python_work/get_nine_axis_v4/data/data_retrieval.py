import requests
import socket
import serial
from utils.logger import Logger

PHYPHOX_URL = "http://192.168.100.118/get?accX&accY&accZ&gyroX&gyroY&gyroZ&magX&magY&magZ"

logger = Logger(__name__)

class DataRetrieval:
    def get_data_from_phyphox(self):
        try:
            response = requests.get(PHYPHOX_URL)
            response.raise_for_status()
            data = response.json()
            result = {}

            if 'accX' in data['buffer']:
                result['acceleration'] = {
                    # "x": data['buffer']['accX']['buffer'][0] * 100,  # m/s² 转 cm/s²
                    # "y": data['buffer']['accY']['buffer'][0] * 100,  # m/s² 转 cm/s²
                    # "z": data['buffer']['accZ']['buffer'][0] * 100   # m/s² 转 cm/s²
                    "x": data['buffer']['accX']['buffer'][0] ,  # m/s² 
                    "y": data['buffer']['accY']['buffer'][0] ,  # m/s² 
                    "z": data['buffer']['accZ']['buffer'][0]    # m/s² 
                }

            if 'gyroX' in data['buffer']:
                result['gyroscope'] = {
                    "x": data['buffer']['gyroX']['buffer'][0] * 57.2958,  # rad/s 转 度/s
                    "y": data['buffer']['gyroY']['buffer'][0] * 57.2958,  # rad/s 转 度/s
                    "z": data['buffer']['gyroZ']['buffer'][0] * 57.2958   # rad/s 转 度/s

                    # "x": data['buffer']['gyroX']['buffer'][0] ,  # rad/s 
                    # "y": data['buffer']['gyroY']['buffer'][0] ,  # rad/s 
                    # "z": data['buffer']['gyroZ']['buffer'][0]    # rad/s 
                }

            if 'magX' in data['buffer']:
                result['magnetometer'] = {
                    "x": data['buffer']['magX']['buffer'][0],#ut
                    "y": data['buffer']['magY']['buffer'][0],#ut
                    "z": data['buffer']['magZ']['buffer'][0]#ut
                    # "x": data['buffer']['magX']['buffer'][0]*1000,#nt
                    # "y": data['buffer']['magY']['buffer'][0]*1000,#nt
                    # "z": data['buffer']['magZ']['buffer'][0]*1000#nt
                }

            logger.info("从 Phyphox 获取数据成功")
            return result
        except requests.RequestException as e:
            logger.error(f"获取数据失败: {e}")
            return None

    def get_data_from_udp(self, local_addr='192.168.100.132', local_port=7777, remote_addr='192.168.100.124', remote_port=8888):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((local_addr, local_port))
        sock.sendto(b'123', (remote_addr, remote_port))
        logger.info(f"与 {remote_addr}:{remote_port} 建立 UDP 连接")

        while True:
            data, _ = sock.recvfrom(1024)  # 缓冲区大小为 1024 字节
            logger.info("接收到 UDP 数据")
            logger.info(data)
            return self.parse_udp_serial_data(data)

    def get_data_from_serial(self, port='COM30', baudrate=1000000):
        try:
            ser = serial.Serial(port, baudrate, timeout=1)
            print(f"在 {port} 端口以 {baudrate} 波特率建立串行连接")

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
                            # 调用封装好的方法进行数据转换3
                            print(f"九轴原始数据: {result}")
                            physical_data = self.convert_to_physical_data(result)
                            print(f"处理数据行: {physical_data}")
                            return physical_data
        except serial.SerialException as e:
            logger.error(f"串行连接错误: {e}")
            return None
    def parse_serial_data(self, data):
        if len(data) != 22:
            logger.error("数据长度不正确")
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

            accel_x = self.convert_signed(accel_x)
            accel_y = self.convert_signed(accel_y)
            accel_z = self.convert_signed(accel_z)
            gyro_x = self.convert_signed(gyro_x)
            gyro_y = self.convert_signed(gyro_y)
            gyro_z = self.convert_signed(gyro_z)
            mag_x = self.convert_signed(mag_x)
            mag_y = self.convert_signed(mag_y)
            mag_z = self.convert_signed(mag_z)

            result = {
                'acceleration': [accel_x, accel_y, accel_z],
                'gyroscope': [gyro_x, gyro_y, gyro_z],
                'magnetometer': [mag_x, mag_y, mag_z]
            }
            return result
        except Exception as e:
            logger.error(f"解析串行数据出错: {e}")
            return None

    def convert_signed(self, value):
        if value > 32767:
            value -= 65536
        return value

    def convert_to_physical_data(self, data):
        acc = data['acceleration']
        gyro = data['gyroscope']
        mag = data['magnetometer']
        
        #加速度计和陀螺仪的灵敏度
        #accel_sensitivity = 0.061 * 0.001*9.81; #单位：m/s²/LSB
        accel_sensitivity = 0.061 * 0.001*9.81; #单位：m/s²/LSB
        acc_x = acc[0]* accel_sensitivity
        acc_y = acc[1] * accel_sensitivity
        acc_z = acc[2] * accel_sensitivity


        #gyro_sensitivity = 17.50 * 0.001 * (3.14159265358979323846 / 180.0)* 57.2958; #单位：rad/s/LSB
        gyro_sensitivity = 17.50 * 0.001 ; #单位：rad/s/LSB
        gyro_x = gyro[0] * gyro_sensitivity  # 转换为 rad/s
        gyro_y = gyro[1] * gyro_sensitivity  # 转换为 rad/s
        gyro_z = gyro[2] * gyro_sensitivity # 转换为 rad/s

        # 磁力计单位: μT
        mag_scale = 1.0  # LSB to μT, 根据实际情况调整
        mag_x = mag[0] * mag_scale
        mag_y = mag[1] * mag_scale
        mag_z = mag[2] * mag_scale

        physical_data = {
            'acceleration': {'x': acc_x, 'y': acc_y, 'z': acc_z},
            'gyroscope': {'x': gyro_x, 'y': gyro_y, 'z': gyro_z},
            'magnetometer': {'x': mag_x, 'y': mag_y, 'z': mag_z}
        }
        return physical_data

    
    def parse_udp_data(self, data):
        hex_data = data.hex()
        if len(hex_data) < 40:
            logger.warning("数据长度不足")
            return None
        
        try:
            accel_x = int(hex_data[0:4], 16)
            accel_y = int(hex_data[4:8], 16)
            accel_z = int(hex_data[8:12], 16)
            gyro_x = int(hex_data[12:16], 16)
            gyro_y = int(hex_data[16:20], 16)
            gyro_z = int(hex_data[20:24], 16)
            mag_x = int(hex_data[24:28], 16)
            mag_y = int(hex_data[28:32], 16)
            mag_z = int(hex_data[32:36], 16)
            temp = int(hex_data[36:40], 16)

            payload = bytes.fromhex(hex_data[:-4])
            crc_received = hex_data[-4:]
            crc_calculated = self.calculate_crc16_modbus(payload)
            if crc_received.lower() != crc_calculated.lower():
                logger.error(f"CRC 校验失败: 接收到的 CRC={crc_received}, 计算的 CRC={crc_calculated}")
                return None

            result = {
                'acceleration': {"x": accel_x, "y": accel_y, "z": accel_z},
                'gyroscope': {"x": gyro_x, "y": gyro_y, "z": gyro_z},
                'magnetometer': {"x": mag_x, "y": mag_y, "z": mag_z},
                'temperature': temp
            }
            logger.info("从 UDP 数据解析成功")
            return result
        except Exception as e:
            logger.error(f"解析 UDP 数据出错: {e}")
            return None

    @staticmethod
    def calculate_crc16_modbus(data):
        crc16 = 0xFFFF
        for pos in data:
            crc16 ^= pos
            for _ in range(8):
                if (crc16 & 0x0001) != 0:
                    crc16 >>= 1
                    crc16 ^= 0xA001
                else:
                    crc16 >>= 1
        crc16 = ((crc16 & 0xFF) << 8) | (crc16 >> 8)  # 交换字节
        return f"{crc16:04x}"
