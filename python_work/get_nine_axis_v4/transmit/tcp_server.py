import socket
import logging
import time
from utils.logger import Logger

logger = Logger(__name__)

class TCPServer:
    def __init__(self, host='127.0.0.1', port=9995):
        self.host = host
        self.port = port

    def start(self, get_euler_angles):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind((self.host, self.port))
        server_socket.listen(1)
        logger.info(f"TCP server listening on {self.host}:{self.port}")

        while True:
            client_socket, addr = server_socket.accept()
            logger.info(f"Connection from {addr}")

            try:
                while True:
                    a_x, a_y, a_z,g_x, g_y, g_z,m_x, m_y, m_z= get_euler_angles()
                    euler_angles = f"euler: {a_x:.2f},{a_y:.2f},{a_z:.2f},{g_x:.2f},{g_y:.2f},{g_z:.2f},{m_x:.2f},{m_y:.2f},{m_z:.2f}\n"
                    # roll, pitch, yaw = get_euler_angles()
                    # euler_angles = f"euler: {roll:.2f},{pitch:.2f},{yaw:.2f}\n"
                    client_socket.send(euler_angles.encode('utf-8'))

                    #logger.info("Euler angles: Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll, pitch, yaw)
                    time.sleep(0.01)
            except ConnectionAbortedError:
                logger.warning(f"Connection to {addr} was aborted")
            except ConnectionResetError:
                logger.warning(f"Connection to {addr} was reset by peer")
            finally:
                client_socket.close()
                logger.info(f"Closed connection to {addr}")
