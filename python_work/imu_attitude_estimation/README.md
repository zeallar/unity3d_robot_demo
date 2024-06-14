# IMU Attitude Estimation

这个项目实现了一个高低通滤波器，用于实时姿态估计。它通过串口接收IMU数据，并进行数据校准、姿态估计和可视化。

## 目录结构

- `data/`: 存储示例IMU数据
- `src/`: 包含主要的代码文件
  - `calibration.py`: 数据校准函数
  - `quaternion.py`: 四元数计算函数
  - `high_low_pass_filter.py`: 高低通滤波器函数
  - `real_time_processing.py`: 实时数据处理和可视化脚本
- `run.py`: 运行主脚本

## 使用方法

1. 安装依赖包：
   ```bash
   pip install -r requirements.txt
2. 运行主脚本：
   python run.py
确保正确设置串口参数，以匹配实际的IMU设备。