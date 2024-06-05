import requests
import time
import logging

# 替换为你的Phyphox实验的IP地址和端口号
phyphox_url = "http://192.168.100.107/get?accX&accY&accZ&gyroX&gyroY&gyroZ&magX&magY&magZ"

# 清除旧的日志文件内容并确保文件编码为UTF-8
with open('log.txt', 'w', encoding='utf-8') as log_file:
    log_file.write('')

# 设置日志配置，确保使用UTF-8编码
logging.basicConfig(
    filename='log.txt',
    level=logging.INFO,
    format='%(asctime)s [INFO] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    encoding='utf-8'
)

def get_phyphox_data():
    response = requests.get(phyphox_url)
    if response.status_code == 200:
        data = response.json()
        result = {}

        if 'accX' in data['buffer']:
            result['acceleration'] = {
                "x": data['buffer']['accX']['buffer'][0],
                "y": data['buffer']['accY']['buffer'][0],
                "z": data['buffer']['accZ']['buffer'][0]
            }

        if 'gyroX' in data['buffer']:
            result['gyroscope'] = {
                "x": data['buffer']['gyroX']['buffer'][0],
                "y": data['buffer']['gyroY']['buffer'][0],
                "z": data['buffer']['gyroZ']['buffer'][0]
            }

        if 'magX' in data['buffer']:
            result['magnetometer'] = {
                "x": data['buffer']['magX']['buffer'][0],
                "y": data['buffer']['magY']['buffer'][0],
                "z": data['buffer']['magZ']['buffer'][0]
            }

        return result
    else:
        print(f"Failed to get data: {response.status_code}")
        return None

def main():
    print("Starting data logging...")
    while True:
        data = get_phyphox_data()
        if data:
            logging.info("加速度计X: %.2f, Y: %.2f, Z: %.2f", data['acceleration']['x'], data['acceleration']['y'], data['acceleration']['z'])
            logging.info("陀螺仪X: %.2f, Y: %.2f, Z: %.2f", data['gyroscope']['x'], data['gyroscope']['y'], data['gyroscope']['z'])
            logging.info("磁力计X: %.2f, Y: %.2f, Z: %.2f", data['magnetometer']['x'], data['magnetometer']['y'], data['magnetometer']['z'])
        time.sleep(1)  # 每秒获取一次数据

if __name__ == "__main__":
    main()
