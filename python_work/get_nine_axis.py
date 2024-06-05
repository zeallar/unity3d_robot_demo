#/*************************************************
#Author:zhouBL
#Version:
#Description:madgwick算法实现版本，前面几秒需要进行梯度下降校准，数值会很不稳。
#Others:
#created date:6/5/2024 6:05 下午
#modified date:
#*************************************************/
import requests
import time
import logging
import math
import socket
import threading

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

# 初始化四元数
q1, q2, q3, q4 = 1.0, 0.0, 0.0, 0.0

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

def madgwick_update(ax, ay, az, gx, gy, gz, mx, my, mz, beta=0.1, dt=0.01):
    global q1, q2, q3, q4

    # 归一化加速度计测量值
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm == 0.0:
        return q1, q2, q3, q4  # 避免除零
    ax /= norm
    ay /= norm
    az /= norm

    # 归一化磁力计测量值
    norm = math.sqrt(mx * mx + my * my + mz * mz)
    if norm == 0.0:
        return q1, q2, q3, q4  # 避免除零
    mx /= norm
    my /= norm
    mz /= norm

    # 计算中间变量
    _2q1mx = 2.0 * q1 * mx
    _2q1my = 2.0 * q1 * my
    _2q1mz = 2.0 * q1 * mz
    _2q2mx = 2.0 * q2 * mx
    hx = mx * q1 * q1 - _2q2mx * q4 + _2q1my * q3 + mx * q2 * q2 + 2.0 * q2 * my * q4 - mx * q3 * q3 - mx * q4 * q4
    hy = _2q1mx * q3 + my * q1 * q1 + _2q1my * q4 + my * q2 * q2 - _2q2mx * q4 - my * q3 * q3 - my * q4 * q4
    _2bx = math.sqrt(hx * hx + hy * hy)
    _2bz = -_2q1mx * q4 + 2.0 * q2 * mx * q3 + mz * q1 * q1 + _2q1my * q4 - _2q1mz * q3 + mz * q2 * q2 + 2.0 * q2 * mz * q4 - mz * q3 * q3 - mz * q4 * q4

    # 估计重力方向
    _2q1 = 2.0 * q1
    _2q2 = 2.0 * q2
    _2q3 = 2.0 * q3
    _2q4 = 2.0 * q4
    _2q1q3 = 2.0 * q1 * q3
    _2q3q4 = 2.0 * q3 * q4
    q1q1 = q1 * q1
    q1q2 = q1 * q2
    q1q3 = q1 * q3
    q1q4 = q1 * q4
    q2q2 = q2 * q2
    q2q3 = q2 * q3
    q2q4 = q2 * q4
    q3q3 = q3 * q3
    q3q4 = q3 * q4
    q4q4 = q4 * q4

    # 梯度下降算法修正
    s1 = -_2q3 * (2.0 * q2q4 - _2q1 * q3 - ax) + _2q2 * (2.0 * q1q2 + _2q3 * q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
    s2 = _2q4 * (2.0 * q2q4 - _2q1 * q3 - ax) + _2q1 * (2.0 * q1q2 + _2q3 * q4 - ay) - 4.0 * q2 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - 2.0 * _2bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
    s3 = -_2q1 * (2.0 * q2q4 - _2q1 * q3 - ax) + _2q4 * (2.0 * q1q2 + _2q3 * q4 - ay) - 4.0 * q3 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + (-4.0 * _2bx * q3 - 2.0 * _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - 2.0 * _2bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
    s4 = _2q2 * (2.0 * q2q4 - _2q1 * q3 - ax) + _2q3 * (2.0 * q1q2 + _2q3 * q4 - ay) + (-4.0 * _2bx * q4 + 2.0 * _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q3 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q2 - 4.0 * _2bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)

    norm = math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)
    norm = 1.0 / norm
    s1 *= norm
    s2 *= norm
    s3 *= norm
    s4 *= norm

    qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1
    qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - beta * s2
    qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - beta * s3
    qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - beta * s4

    q1 += qDot1 * dt
    q2 += qDot2 * dt
    q3 += qDot3 * dt
    q4 += qDot4 * dt

    norm = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
    norm = 1.0 / norm
    q1 *= norm
    q2 *= norm
    q3 *= norm
    q4 *= norm

    return q1, q2, q3, q4

def quaternion_to_euler(q1, q2, q3, q4):
    # 四元数转欧拉角
    roll = math.atan2(2.0 * (q1 * q2 + q3 * q4), 1.0 - 2.0 * (q2 * q2 + q3 * q3))
    pitch = math.asin(2.0 * (q1 * q3 - q4 * q2))
    yaw = math.atan2(2.0 * (q1 * q4 + q2 * q3), 1.0 - 2.0 * (q3 * q3 + q4 * q4))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

def tcp_server(host='127.0.0.1', port=9995):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print(f"TCP server listening on {host}:{port}")

    while True:
        client_socket, addr = server_socket.accept()
        print(f"Connection from {addr}")
        while True:
            data = get_phyphox_data()
            if data:
                # 转换单位
                ax, ay, az = data['acceleration']['x'], data['acceleration']['y'], data['acceleration']['z']
                gx, gy, gz = math.radians(data['gyroscope']['x']), math.radians(data['gyroscope']['y']), math.radians(data['gyroscope']['z'])
                mx, my, mz = data['magnetometer']['x'], data['magnetometer']['y'], data['magnetometer']['z']

                # 更新四元数
                q1, q2, q3, q4 = madgwick_update(ax, ay, az, gx, gy, gz, mx, my, mz)

                # 转换为欧拉角
                roll, pitch, yaw = quaternion_to_euler(q1, q2, q3, q4)

                # 发送数据给客户端
                euler_angles = f"euler: {roll:.2f},{pitch:.2f},{yaw:.2f}\n"
                client_socket.send(euler_angles.encode('utf-8'))

                # 记录日志
                logging.info("加速度计X: %.2f, Y: %.2f, Z: %.2f", ax, ay, az)
                logging.info("陀螺仪X: %.2f, Y: %.2f, Z: %.2f", gx, gy, gz)
                logging.info("磁力计X: %.2f, Y: %.2f, Z: %.2f", mx, my, mz)
                logging.info("四元数: q1: %.2f, q2: %.2f, q3: %.2f, q4: %.2f", q1, q2, q3, q4)
                logging.info("欧拉角: Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll, pitch, yaw)
            time.sleep(0.01)  # 每0.01秒获取一次数据

def main():
    print("Starting data logging...")
    tcp_thread = threading.Thread(target=tcp_server)
    tcp_thread.start()

if __name__ == "__main__":
    main()
