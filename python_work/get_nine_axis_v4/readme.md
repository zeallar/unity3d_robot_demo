待完成
1、数据校准问题
    参考1：https://github.com/Crazepony/crazepony-firmware-none/blob/master/User_Src/IMU.c
    参考2：https://github.com/avem-labs/Avem/blob/develop/libs/module/avm_mpu6050.c
2、LSM6DS3TR+QMC6309调试


#### **阶段工作汇报-20240606**

**云喇叭项目**

已完成：

1、基本功能调试。

待完成：

1、esim复位问题。

​	进度：卡在广和通那边了。

2、测试secureboot。

​	进度：待测试secureboot启用方案可行性。

**运动环项目**

已完成：

1、3d游戏模型搭建，并用传感器完成走跑跳等基础动作。

2、实现九轴数据融合算法。

​	已用手机九轴数据验证了算法的可靠性，并能实现目标姿态与手机姿态同步。

待完成：

1、产品九轴传感器数据校准，以及数据融合算法针对性调试。

6/13/2024 2:15 下午
经测试
madgwick.py方法可行，基于https://github.com/Mayitzin/ahrs 源码修改而来