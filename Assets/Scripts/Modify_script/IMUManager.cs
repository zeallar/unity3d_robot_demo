using UnityEngine;
using System.Threading.Tasks;

public class IMUManager : MonoBehaviour
{
    public Vector3 accelOffset;
    public Vector3 gyroOffset;
    public Vector3 magOffset;

    public async Task InitializeIMU()
    {
        await Task.Run(() =>
        {
            Vector3 accelOffsetTemp = CalculateOffset(GetAccelData);
            Vector3 gyroOffsetTemp = CalculateOffset(GetGyroData);

            // 在主线程上设置偏移量和存储
            MainThreadInvoker.Enqueue(() =>
            {
                accelOffset = accelOffsetTemp;
                gyroOffset = gyroOffsetTemp;

                PlayerPrefs.SetFloat("accelOffsetX", accelOffset.x);
                PlayerPrefs.SetFloat("accelOffsetY", accelOffset.y);
                PlayerPrefs.SetFloat("accelOffsetZ", accelOffset.z);
                PlayerPrefs.SetFloat("gyroOffsetX", gyroOffset.x);
                PlayerPrefs.SetFloat("gyroOffsetY", gyroOffset.y);
                PlayerPrefs.SetFloat("gyroOffsetZ", gyroOffset.z);
            });
        });

        await Task.Run(() =>
        {
            Vector3 magOffsetTemp = CalculateMagOffset();

            // 在主线程上设置磁力计偏移量和存储
            MainThreadInvoker.Enqueue(() =>
            {
                magOffset = magOffsetTemp;

                PlayerPrefs.SetFloat("magOffsetX", magOffset.x);
                PlayerPrefs.SetFloat("magOffsetY", magOffset.y);
                PlayerPrefs.SetFloat("magOffsetZ", magOffset.z);
            });
        });
    }

    private Vector3 CalculateOffset(System.Func<Vector3> getDataMethod)
    {
        Vector3 offset = Vector3.zero;
        int sampleCount = 20;

        for (int i = 0; i < sampleCount; i++)
        {
            offset += getDataMethod();
        }

        return offset / sampleCount;
    }

    private Vector3 CalculateMagOffset()
    {
        int sampleCount = 20;
        int mag_x_min = int.MaxValue, mag_y_min = int.MaxValue, mag_z_min = int.MaxValue;
        int mag_x_max = int.MinValue, mag_y_max = int.MinValue, mag_z_max = int.MinValue;

        for (int i = 0; i < sampleCount; i++)
        {
            Vector3 magData = GetMagData();
            if (magData.x < mag_x_min) mag_x_min = (int)magData.x;
            if (magData.x > mag_x_max) mag_x_max = (int)magData.x;
            if (magData.y < mag_y_min) mag_y_min = (int)magData.y;
            if (magData.y > mag_y_max) mag_y_max = (int)magData.y;
            if (magData.z < mag_z_min) mag_z_min = (int)magData.z;
            if (magData.z > mag_z_max) mag_z_max = (int)magData.z;
        }

        return new Vector3(
            ((mag_x_max - mag_x_min) / 2 - mag_x_max),
            ((mag_y_max - mag_y_min) / 2 - mag_y_max),
            0.0f // Assuming Z offset is 0
        );
    }

    public Vector3 GetAccelData()
    {
        float accelX = 0, accelY = 0, accelZ = 0;

        // 在主线程上获取加速度数据
        MainThreadInvoker.Enqueue(() =>
        {
            SensorData.Instance.GetAccelData(out accelX, out accelY, out accelZ);
        });

        // 等待数据获取完成
        while (accelX == 0 && accelY == 0 && accelZ == 0) { }

        return new Vector3(accelX, accelY, accelZ);
    }

    public Vector3 GetGyroData()
    {
        float gyroX = 0, gyroY = 0, gyroZ = 0;

        // 在主线程上获取陀螺仪数据
        MainThreadInvoker.Enqueue(() =>
        {
            SensorData.Instance.GetGyroData(out gyroX, out gyroY, out gyroZ);
        });

        // 等待数据获取完成
        while (gyroX == 0 && gyroY == 0 && gyroZ == 0) { }

        return new Vector3(gyroX, gyroY, gyroZ);
    }

    public Vector3 GetMagData()
    {
        float magX = 0, magY = 0, magZ = 0;

        // 在主线程上获取磁力计数据
        MainThreadInvoker.Enqueue(() =>
        {
            SensorData.Instance.GetMagData(out magX, out magY, out magZ);
        });

        // 等待数据获取完成
        while (magX == 0 && magY == 0 && magZ == 0) { }

        return new Vector3(magX, magY, magZ);
    }
}
