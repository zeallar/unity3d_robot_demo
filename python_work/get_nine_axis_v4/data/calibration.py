import numpy as np
import json
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from mpl_toolkits.mplot3d import Axes3D
import time
import csv
class Calibration:
    def __init__(self):
        self.GRAVITY=9.802
        self.calibration_data = {'acceleration': [], 'gyroscope': [], 'magnetometer': []}
        self.bias = {'acceleration': {'x': 0, 'y': 0, 'z': 0}, 'gyroscope': {'x': 0, 'y': 0, 'z': 0}, 'magnetometer': {'x': 0, 'y': 0, 'z': 0}}
        self.MAGN_ELLIPSOID_CENTER = np.array([-1.22362, -3.49591, -28.3068])
        self.MAGN_ELLIPSOID_TRANSFORM = np.array([[0.97269, 0.0124029, -0.00955096], 
                                                  [0.0124029, 0.994026, 6.33932e-05], 
                                                  [-0.00955096, 6.33932e-05, 0.943093]])

    def add_data(self, data):
        self.calibration_data['acceleration'].append([data['acceleration']['x'], data['acceleration']['y'], data['acceleration']['z']])
        self.calibration_data['gyroscope'].append([data['gyroscope']['x'], data['gyroscope']['y'], data['gyroscope']['z']])
        self.calibration_data['magnetometer'].append([data['magnetometer']['x'], data['magnetometer']['y'], data['magnetometer']['z']])

    def calculate_bias(self):
        for sensor in ['acceleration','gyroscope']:
            data = np.array(self.calibration_data[sensor])
            self.bias[sensor]['x'] = np.mean(data[:, 0])
            self.bias[sensor]['y'] = np.mean(data[:, 1])
            self.bias[sensor]['z'] = np.mean(data[:, 2])
    #椭球拟合函数
    def ellipsoid_fit(self,mag_data):
        data = np.array(self.calibration_data['magnetometer'])
        def residuals(params, x, y, z):
            xc, yc, zc, a1, a2, a3, a4, a5, a6 = params
            x_p = x - xc
            y_p = y - yc
            z_p = z - zc
            return a1 * x_p ** 2 + a2 * y_p ** 2 + a3 * z_p ** 2 + 2 * a4 * x_p * y_p + 2 * a5 * x_p * z_p + 2 * a6 * y_p * z_p - 1

        x, y, z = mag_data[:, 0], mag_data[:, 1], mag_data[:, 2]
        x_m, y_m, z_m = np.mean(x), np.mean(y), np.mean(z)
        initial_guess = [x_m, y_m, z_m, 1, 1, 1, 0, 0, 0]

        result = least_squares(residuals, initial_guess, args=(x, y, z))
        return result.x

    #数据校正函数
    def correct_magnetometer_data(self, mag_data,params):
        xc, yc, zc, a1, a2, a3, a4, a5, a6 = params
        A = np.array([[a1, a4, a5], [a4, a2, a6], [a5, a6, a3]])
        offset = np.array([xc, yc, zc])

        corrected_data = []
        for mag in mag_data:
            mag_corrected = np.dot(np.linalg.inv(A), (mag - offset))
            corrected_data.append(mag_corrected)
        return np.array(corrected_data)

    #可视化函数
    def plot_magnetometer_data(self, mag_data, title='Magnetometer Data'):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(mag_data[:, 0], mag_data[:, 1], mag_data[:, 2], c='r', marker='o')
        ax.set_xlabel('Mag X')
        ax.set_ylabel('Mag Y')
        ax.set_zlabel('Mag Z')
        ax.set_title(title)
        plt.show()
    #数据采集3d可视化
    def visualize_mag_data(self):
        plt.ion()  # 开启交互模式
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Real-time Magnetometer Data')

        while True:
            if self.calibration_data['magnetometer']:
                data = np.array(self.calibration_data['magnetometer'])
                x, y, z = data[:, 0], data[:, 1], data[:, 2]

                # 清空图形
                ax.clear()

                # 绘制所有数据点
                ax.scatter(x, y, z, c='r', marker='o')

                # 更新图形
                plt.draw()
                plt.pause(0.01)

            # 如果用户关闭窗口，则退出循环
            if not plt.fignum_exists(fig.number):
                break

        # 关闭交互模式
        plt.ioff()
        plt.show()
    #存储偏移值
    def save_calibration_data(self, filename):
        with open(filename, 'w') as f:
            json.dump(self.bias, f, indent=4)
        print(f"Calibration data saved to {filename}")
    #加载偏移
    def load_calibration_data(self, filename):
        try:
            with open(filename, 'r') as f:
                loaded_data = json.load(f)
                for sensor_type in self.bias:
                    for axis in self.bias[sensor_type]:
                        self.bias[sensor_type][axis] = loaded_data[sensor_type][axis]
            print(f"Calibration data loaded from {filename}")
        except FileNotFoundError:
            print(f"Error: Calibration data file '{filename}' not found.")
        except json.JSONDecodeError:
            print(f"Error: Failed to decode JSON in '{filename}'. File may be corrupted.")

    #校准参数保存和加载函数
    def save_calibration_params(self, params, filename='calibration_params.json'):
        with open(filename, 'w') as f:
            json.dump(params.tolist(), f)

    def load_calibration_params(self, filename='calibration_params.json'):
        with open(filename, 'r') as f:
            params = json.load(f)
        return np.array(params)


    def get_bias(self):
        return self.bias
    def get_mag_data(self):
        data = np.array(self.calibration_data['magnetometer'])
        return data
    def apply_calibration(self, data):
        data['acceleration']['x'] -= self.bias['acceleration']['x']
        data['acceleration']['y'] -= self.bias['acceleration']['y']
        data['acceleration']['z'] -= self.bias['acceleration']['z']+self.GRAVITY
        data['gyroscope']['x'] -= self.bias['gyroscope']['x']
        data['gyroscope']['y'] -= self.bias['gyroscope']['y']
        data['gyroscope']['z'] -= self.bias['gyroscope']['z']
  
        # Compensate magnetometer error
        magnetom = np.array([data['magnetometer']['x'], data['magnetometer']['y'], data['magnetometer']['z']])
        magnetom_tmp = magnetom - self.MAGN_ELLIPSOID_CENTER
        magnetom_corrected = np.dot(self.MAGN_ELLIPSOID_TRANSFORM, magnetom_tmp)
        
        # data['magnetometer']['x'] = magnetom_corrected[0]
        # data['magnetometer']['y'] = magnetom_corrected[1]
        # data['magnetometer']['z'] = magnetom_corrected[2]
        return data
    # 将磁力计数据保存为CSV文件
    def save_magnetometer_data_to_csv(self, filename):
        data = np.array(self.calibration_data['magnetometer'])
        if data.size == 0:
            print("No magnetometer data to save.")
            return

        # 保存为三列
        with open(filename, 'w', newline='') as file:
            writer = csv.writer(file)
            #writer.writerow(['Mag X', 'Mag Y', 'Mag Z'])
            writer.writerows(data)

        print(f"Magnetometer data saved to {filename}")
