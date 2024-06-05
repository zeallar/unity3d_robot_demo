#/*************************************************
#Author:zhouBL
#Version:
#Description:https://github.com/diceTZ/Pose  姿态解算版本
#Others:用的手机传感器很准
#created date:6/5/2024 6:05 下午
#modified date:
#*************************************************/
import requests
import time
import logging
import math
import socket
import threading

# Phyphox experiment IP address and port
PHYPHOX_URL = "http://192.168.100.107/get?accX&accY&accZ&gyroX&gyroY&gyroZ&magX&magY&magZ"

# Clear old log file and ensure UTF-8 encoding
with open('log.txt', 'w', encoding='utf-8') as log_file:
    log_file.write('')

# Set up logging configuration
logging.basicConfig(
    filename='log.txt',
    level=logging.INFO,
    format='%(asctime)s [INFO] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    encoding='utf-8'
)

class PoseModule:
    def __init__(self):
        self.run = True
        self.use_mag = True
        self.a_x = self.a_y = self.a_z = 0.0
        self.g_x = self.g_y = self.g_z = 0.0
        self.m_x = self.m_y = self.m_z = 0.0
        self.error_ki = 0.0
        self.error_kp = 0.65
        self.correct_kp = 0.45
        self.error = [0.0, 0.0, 0.0]
        self.error_integral = [0.0, 0.0, 0.0]
        self.quaternion = [1.0, 0.0, 0.0, 0.0]
        self.rotate_matrix = [[0.0] * 3 for _ in range(3)]
        self.mag_world = [0.0, 0.0, 0.0]
        self.acc_world = [0.0, 0.0, 0.0]
        self.mag_correct = [0.0, 0.0, 0.0]
        self.acc_correct = [0.0, 0.0, 0.0]
        self.gyro_correct = [0.0, 0.0, 0.0]
        self.pit = self.rol = self.yaw = 0.0

pose = PoseModule()

def get_phyphox_data():
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

        return result
    else:
        print(f"Failed to get data: {response.status_code}")
        return None

def madgwick_update(pose, cycle):
    if not pose.run:
        return pose.quaternion

    length = math.sqrt(pose.a_x ** 2 + pose.a_y ** 2 + pose.a_z ** 2)
    if length != 0:
        acc_tmp = [pose.a_x / length, pose.a_y / length, pose.a_z / length]
    else:
        return pose.quaternion

    if all(abs(val) < 1050 for val in [pose.a_x, pose.a_y, pose.a_z]):
        if 800.0 < length < 1200.0:
            error = [
                acc_tmp[1] * pose.rotate_matrix[2][2] - acc_tmp[2] * pose.rotate_matrix[1][2],
                acc_tmp[2] * pose.rotate_matrix[0][2] - acc_tmp[0] * pose.rotate_matrix[2][2],
                acc_tmp[0] * pose.rotate_matrix[1][2] - acc_tmp[1] * pose.rotate_matrix[0][2]
            ]

            pose.error = [pose.error[i] + 3.14 * cycle * (error[i] - pose.error[i]) for i in range(3)]

        pose.error_integral = [pose.error_integral[i] + pose.error[i] * pose.error_ki * cycle for i in range(3)]
        pose.error_integral = [min(max(val, -0.035), 0.035) for val in pose.error_integral]

        pose.gyro_correct = [
            (pose.g_x - pose.rotate_matrix[0][2] * pose.correct_kp) * 0.01745329 + (pose.error_kp * pose.error[0] + pose.error_integral[0]),
            (pose.g_y - pose.rotate_matrix[1][2] * pose.correct_kp) * 0.01745329 + (pose.error_kp * pose.error[1] + pose.error_integral[1]),
            (pose.g_z - pose.rotate_matrix[2][2] * pose.correct_kp) * 0.01745329 + (pose.error_kp * pose.error[2] + pose.error_integral[2])
        ]

        q_dot = [
            0.5 * (-pose.quaternion[1] * pose.gyro_correct[0] - pose.quaternion[2] * pose.gyro_correct[1] - pose.quaternion[3] * pose.gyro_correct[2]),
            0.5 * (pose.quaternion[0] * pose.gyro_correct[0] + pose.quaternion[2] * pose.gyro_correct[2] - pose.quaternion[3] * pose.gyro_correct[1]),
            0.5 * (pose.quaternion[0] * pose.gyro_correct[1] - pose.quaternion[1] * pose.gyro_correct[2] + pose.quaternion[3] * pose.gyro_correct[0]),
            0.5 * (pose.quaternion[0] * pose.gyro_correct[2] + pose.quaternion[1] * pose.gyro_correct[1] - pose.quaternion[2] * pose.gyro_correct[0])
        ]
        
        pose.quaternion = [pose.quaternion[i] + q_dot[i] * cycle for i in range(4)]
        norm = math.sqrt(sum(val ** 2 for val in pose.quaternion))
        pose.quaternion = [val / norm for val in pose.quaternion]

        pose.rotate_matrix[0][0] = pose.quaternion[0] ** 2 + pose.quaternion[1] ** 2 - pose.quaternion[2] ** 2 - pose.quaternion[3] ** 2
        pose.rotate_matrix[0][1] = 2 * (pose.quaternion[1] * pose.quaternion[2] + pose.quaternion[0] * pose.quaternion[3])
        pose.rotate_matrix[0][2] = 2 * (pose.quaternion[1] * pose.quaternion[3] - pose.quaternion[0] * pose.quaternion[2])
        pose.rotate_matrix[1][0] = 2 * (pose.quaternion[1] * pose.quaternion[2] - pose.quaternion[0] * pose.quaternion[3])
        pose.rotate_matrix[1][1] = pose.quaternion[0] ** 2 - pose.quaternion[1] ** 2 + pose.quaternion[2] ** 2 - pose.quaternion[3] ** 2
        pose.rotate_matrix[1][2] = 2 * (pose.quaternion[2] * pose.quaternion[3] + pose.quaternion[0] * pose.quaternion[1])
        pose.rotate_matrix[2][0] = 2 * (pose.quaternion[1] * pose.quaternion[3] + pose.quaternion[0] * pose.quaternion[2])
        pose.rotate_matrix[2][1] = 2 * (pose.quaternion[2] * pose.quaternion[3] - pose.quaternion[0] * pose.quaternion[1])
        pose.rotate_matrix[2][2] = pose.quaternion[0] ** 2 - pose.quaternion[1] ** 2 - pose.quaternion[2] ** 2 + pose.quaternion[3] ** 2

        pose.acc_world = [
            sum(pose.rotate_matrix[i][j] * acc_tmp[j] for j in range(3)) for i in range(3)
        ]

        pose.pit = -math.asin(pose.rotate_matrix[0][2])
        pose.rol = math.atan2(pose.rotate_matrix[2][2], pose.rotate_matrix[1][2])
        pose.yaw = math.atan2(pose.rotate_matrix[0][0], pose.rotate_matrix[0][1])

        pose.acc_correct = [
            pose.acc_world[0] * math.cos(pose.yaw) + pose.acc_world[1] * math.sin(pose.yaw),
            -pose.acc_world[0] * math.sin(pose.yaw) + pose.acc_world[1] * math.cos(pose.yaw),
            pose.acc_world[2]
        ]

        if pose.use_mag:
            mag_tmp = [pose.m_x, pose.m_y, pose.m_z]
            length = math.sqrt(sum(val ** 2 for val in mag_tmp))
            if length != 0:
                ref_v = pose.rotate_matrix[2]
                simple_3d_trans(ref_v, mag_tmp, pose.mag_correct)

    return pose.quaternion

def simple_3d_trans(ref, in_vector, out_vector):
    h_tmp_x = math.sqrt(ref[2] ** 2 + ref[1] ** 2)
    h_tmp_y = math.sqrt(ref[2] ** 2 + ref[0] ** 2)
    pn = -1 if ref[2] < 0 else 1
    out_vector[0] = (h_tmp_x * in_vector[0] - pn * ref[0] * in_vector[2])
    out_vector[1] = (pn * h_tmp_y * in_vector[1] - ref[1] * in_vector[2])
    out_vector[2] = ref[0] * in_vector[0] + ref[1] * in_vector[1] + ref[2] * in_vector[2]

def quaternion_to_euler(q1, q2, q3, q4):
    # Quaternion to Euler angles
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
                # Convert units
                pose.a_x, pose.a_y, pose.a_z = data['acceleration']['x'], data['acceleration']['y'], data['acceleration']['z']
                pose.g_x, pose.g_y, pose.g_z = data['gyroscope']['x'], data['gyroscope']['y'], data['gyroscope']['z']
                pose.m_x, pose.m_y, pose.m_z = data['magnetometer']['x'], data['magnetometer']['y'], data['magnetometer']['z']

                # Update quaternion
                q1, q2, q3, q4 = madgwick_update(pose, 0.01)

                # Send data to client
                euler_angles = f"euler: {pose.rol:.2f},{pose.pit:.2f},{pose.yaw:.2f}\n"
                client_socket.send(euler_angles.encode('utf-8'))

                # Log data
                logging.info("Acceleration X: %.2f cm/s², Y: %.2f cm/s², Z: %.2f cm/s²", pose.a_x, pose.a_y, pose.a_z)
                logging.info("Gyroscope X: %.2f °/s, Y: %.2f °/s, Z: %.2f °/s", pose.g_x, pose.g_y, pose.g_z)
                logging.info("Magnetometer X: %.2f, Y: %.2f, Z: %.2f", pose.m_x, pose.m_y, pose.m_z)
                logging.info("Quaternion: q1: %.2f, q2: %.2f, q3: %.2f, q4: %.2f", q1, q2, q3, q4)
                logging.info("Euler angles: Roll: %.2f, Pitch: %.2f, Yaw: %.2f", pose.rol, pose.pit, pose.yaw)
            time.sleep(0.01)  # Get data every 0.01 seconds

def main():
    print("Starting data logging...")
    tcp_thread = threading.Thread(target=tcp_server)
    tcp_thread.start()

if __name__ == "__main__":
    main()
