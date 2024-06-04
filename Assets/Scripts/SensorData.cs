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

    public bool hasNewlData = false;  // 新增标志位
    public bool hasNewlgyroData = false;  // 新增标志位
    public bool hasNewlmagData = false;  // 新增标志位
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
            if (!IsZeroData(accelX, accelY, accelZ)&& hasNewlData == false)
            {
                this.accelX = accelX/100;
                this.accelY = accelY/100; // 模块的z和y与Unity3D的不同，互换一下。
                this.accelZ = accelZ/100;
                hasNewlData = true; // 设置标志位

                string accelLog = $"加速度X: {this.accelX}, Y: {this.accelY}, Z: {this.accelZ}";
                Debug.Log(accelLog);
                //Logger.Info(accelLog);
            }
        }
    }

    public void SetGyroData(float gyroX, float gyroY, float gyroZ)
    {
        lock (gyroLock)
        {
            if (!IsZeroData(gyroX, gyroY, gyroZ) && hasNewlgyroData == false)
            {
                //this.gyroX = gyroX;
                //this.gyroY = gyroY;
                //this.gyroZ = gyroZ;

                this.gyroX = gyroX / 100;
                this.gyroY = gyroY / 100;
                this.gyroZ = gyroZ / 100;
                hasNewlgyroData = true; // 设置标志位
                string gyroLog = $"陀螺仪X: {this.gyroX}, Y: {this.gyroY}, Z: {this.gyroZ}";
                //Debug.Log(gyroLog);
                Logger.Info(gyroLog);
            }
        }
    }

    public void SetMagData(float magX, float magY, float magZ)
    {
        lock (magLock)
        {
            if (!IsZeroData(magX, magY, magZ) && hasNewlmagData == false)
            {
                magX /= 100f;
                magY /= 100f;
                magZ /= 100f;
                this.magX = magX;
                this.magY = magY;
                this.magZ = magZ;


                hasNewlmagData = true; // 设置标志位
                string magLog = $"磁力计X: {this.magX}, Y: {this.magY}, Z: {this.magZ}";
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
            hasNewlData = false; // 设置标志位
        }
    }

    public void GetGyroData(out float gyroX, out float gyroY, out float gyroZ)
    {
        lock (gyroLock)
        {
            gyroX = this.gyroX;
            gyroY = this.gyroY;
            gyroZ = this.gyroZ;
            hasNewlgyroData = false; // 设置标志位
        }
    }

    public void GetMagData(out float magX, out float magY, out float magZ)
    {
        lock (magLock)
        {
            magX = this.magX;
            magY = this.magY;
            magZ = this.magZ;
            hasNewlmagData = false; // 设置标志位
        }
    }

    private bool IsZeroData(float x, float y, float z)
    {
        return x == 0 && y == 0 && z == 0;
    }
}