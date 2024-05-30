using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SensorData : MonoBehaviour
{
    private static SensorData instance;

    public static SensorData Instance
    {
        get
        {
            if (instance == null)
            {
                instance = new GameObject("SensorData").AddComponent<SensorData>();
            }
            return instance;
        }
    }

    private float temp;
    private float accelX, accelY, accelZ;
    private float gyroX, gyroY, gyroZ;
    private float magX, magY, magZ;

    private readonly object tempLock = new object();
    private readonly object accelLock = new object();
    private readonly object gyroLock = new object();
    private readonly object magLock = new object();

    public bool hasNewAccelData = false;  // 新增标志位
    public void SetTemp(float temp)
    {
        lock (tempLock)
        {
            this.temp = temp;
            //Debug.Log($"温度: {temp}");
        }
    }

    public void SetAccelData(float accelX, float accelY, float accelZ)
    {
        lock (accelLock)
        {
            if (!IsZeroData(accelX, accelY, accelZ))
            {
                this.accelX = accelX;
                this.accelY = accelZ; // 模块的z和y与Unity3D的不同，互换一下。
                this.accelZ = accelY;
                hasNewAccelData = true; // 设置标志位

                string accelLog = $"设置 加速度X: {accelX / 100}, Y: {this.accelY / 100}, Z: {this.accelZ / 100}";
                //Debug.Log(accelLog);
                //Logger.Info(accelLog);
            }
        }
    }

    public void SetGyroData(float gyroX, float gyroY, float gyroZ)
    {
        lock (gyroLock)
        {
            if (!IsZeroData(gyroX, gyroY, gyroZ))
            {
                //this.gyroX = gyroX;
                //this.gyroY = gyroY;
                //this.gyroZ = gyroZ;

                this.gyroX = gyroX / 100;
                this.gyroY = gyroY / 100;
                this.gyroZ = gyroZ / 100;

                string gyroLog = $"陀螺仪X: {this.gyroX}, Y: {this.gyroY}, Z: {this.gyroZ}";
                Debug.Log(gyroLog);
                Logger.Info(gyroLog);
            }
        }
    }

    public void SetMagData(float magX, float magY, float magZ)
    {
        lock (magLock)
        {
            if (!IsZeroData(magX, magY, magZ))
            {
                this.magX = magX;
                this.magY = magY;
                this.magZ = magZ;

                magX /= 100f;
                magY /= 100f;
                magZ /= 100f;
                string magLog = $"磁力计X: {magX}, Y: {magY}, Z: {magZ}";
                //Debug.Log(magLog);
                //Logger.Info(magLog);
            }
        }
    }

    public void GetTemp(out float temp)
    {
        lock (tempLock)
        {
            temp = this.temp;
        }
    }

    public void GetAccelData(out float accelX, out float accelY, out float accelZ)
    {
        lock (accelLock)
        {
            accelX = this.accelX;
            accelY = this.accelY;
            accelZ = this.accelZ;
        }
    }

    public void GetGyroData(out float gyroX, out float gyroY, out float gyroZ)
    {
        lock (gyroLock)
        {
            gyroX = this.gyroX;
            gyroY = this.gyroY;
            gyroZ = this.gyroZ;
        }
    }

    public void GetMagData(out float magX, out float magY, out float magZ)
    {
        lock (magLock)
        {
            magX = this.magX;
            magY = this.magY;
            magZ = this.magZ;
        }
    }

    private bool IsZeroData(float x, float y, float z)
    {
        return x == 0 && y == 0 && z == 0;
    }
}