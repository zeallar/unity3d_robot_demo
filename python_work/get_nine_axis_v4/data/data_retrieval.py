import requests
import socket
from utils.logger import Logger

PHYPHOX_URL = "http://192.168.100.121/get?accX&accY&accZ&gyroX&gyroY&gyroZ&magX&magY&magZ"

logger = Logger(__name__)

class DataRetrieval:
    def get_data_from_phyphox(self):
        response = requests.get(PHYPHOX_URL)
        if response.status_code == 200:
            data = response.json()
            result = {}

            if 'accX' in data['buffer']:
                result['acceleration'] = {
                    "x": data['buffer']['accX']['buffer'][0] * 100,  # m/s² to cm/s²
                    "y": data['buffer']['accY']['buffer'][0] * 100,  # m/s² to cm/s²
                    "z": data['buffer']['accZ']['buffer'][0] * 100   # m/s² to cm/s²
                }

            if 'gyroX' in data['buffer']:
                result['gyroscope'] = {
                    "x": data['buffer']['gyroX']['buffer'][0] * 57.2958,  # rad/s to degrees/s
                    "y": data['buffer']['gyroY']['buffer'][0] * 57.2958,  # rad/s to degrees/s
                    "z": data['buffer']['gyroZ']['buffer'][0] * 57.2958   # rad/s to degrees/s
                }

            if 'magX' in data['buffer']:
                result['magnetometer'] = {
                    "x": data['buffer']['magX']['buffer'][0],
                    "y": data['buffer']['magY']['buffer'][0],
                    "z": data['buffer']['magZ']['buffer'][0]
                }

            #logger.info("Data retrieved from Phyphox")
            return result
        else:
            logger.error(f"Failed to get data: {response.status_code}")
            return None

    def get_data_from_udp(self, local_addr='192.168.100.132', local_port=7777, remote_addr='192.168.100.124', remote_port=8888):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((local_addr, local_port))
        sock.sendto(b'123', (remote_addr, remote_port))
        logger.info(f"UDP connection established with {remote_addr}:{remote_port}")
        
        while True:
            data, _ = sock.recvfrom(1024)  # buffer size is 1024 bytes
            logger.info("UDP data received")
            return self.parse_udp_data(data)

    def parse_udp_data(self, data):
        hex_data = data.hex()
        if len(hex_data) < 40:
            logger.warning("Data length is insufficient.")
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

            # Calculate and validate CRC16
            payload = bytes.fromhex(hex_data[:-4])
            crc_received = hex_data[-4:]
            crc_calculated = self.calculate_crc16_modbus(payload)
            if crc_received.lower() != crc_calculated.lower():
                logger.error(f"CRC check failed: Received CRC={crc_received}, Calculated CRC={crc_calculated}")
                return None

            result = {
                'acceleration': {"x": accel_x, "y": accel_y, "z": accel_z},
                'gyroscope': {"x": gyro_x, "y": gyro_y, "z": gyro_z},
                'magnetometer': {"x": mag_x, "y": mag_y, "z": mag_z},
                'temperature': temp
            }
            logger.info("Data parsed from UDP")
            return result
        except Exception as e:
            logger.error(f"Error parsing UDP data: {e}")
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
        crc16 = ((crc16 & 0xFF) << 8) | (crc16 >> 8)  # Swap bytes
        return f"{crc16:04x}"
