using UnityEngine;

public class AttitudeEstimation
{
    private float alpha = 0.98f; // 滤波器系数

    public Quaternion FuseSensorData(Vector3 mag, Vector3 accel, Vector3 gyro)
    {
        // 使用互补滤波器融合传感器数据
        Vector3 gravity = accel.normalized;
        Vector3 magneticNorth = mag.normalized;
        Vector3 crossProduct = Vector3.Cross(gravity, magneticNorth);
        Quaternion rotation = Quaternion.LookRotation(crossProduct, gravity);

        // 简单的互补滤波器示例
        Quaternion gyroDelta = Quaternion.Euler(gyro * Time.deltaTime);
        rotation = Quaternion.Slerp(rotation, gyroDelta * rotation, alpha);

        return rotation.normalized;
    }
}
