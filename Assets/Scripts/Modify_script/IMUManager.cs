using UnityEngine;
using System.Collections.Generic;

public class IMUManager : MonoBehaviour
{
    public Vector3 accelOffset;
    public Vector3 gyroOffset;
    public Vector3 magOffset;
    public bool imuInitialized = false;

    public void SetAccelGyroOffsets(List<Vector3> AccelDataList, List<Vector3> GyrolDataList, List<Vector3> MagDataList)
    {
        accelOffset = CalculateOffset(AccelDataList);
        accelOffset.z -= 16384;
        gyroOffset = CalculateOffset(GyrolDataList);
        SetMagOffset(MagDataList);
        Logger.Info($"accelOffset:{accelOffset},gyroOffset:{gyroOffset}");
        imuInitialized = true;

    }

    private void SetMagOffset(List<Vector3> MagDataList)
    {
        magOffset = Vector3.zero;
        float magXScaleFactor = 1.0f;
        float magYScaleFactor = 1.0f;
        if (MagDataList == null || MagDataList.Count == 0)
        {
            Debug.LogError("Magnetometer data list is empty.");
            return;
        }

        float magXMin = float.MaxValue, magYMin = float.MaxValue, magZMin = float.MaxValue;
        float magXMax = float.MinValue, magYMax = float.MinValue, magZMax = float.MinValue;

        foreach (var magData in MagDataList)
        {
            if (magData.x < magXMin) magXMin = magData.x;
            if (magData.x > magXMax) magXMax = magData.x;
            if (magData.y < magYMin) magYMin = magData.y;
            if (magData.y > magYMax) magYMax = magData.y;
            if (magData.z < magZMin) magZMin = magData.z;
            if (magData.z > magZMax) magZMax = magData.z;
        }

        magXScaleFactor = (magYMax - magYMin) / (magXMax - magXMin);
        magYScaleFactor = (magXMax - magXMin) / (magYMax - magYMin);

        if (magXScaleFactor < 1) magXScaleFactor = 1;
        if (magYScaleFactor < 1) magYScaleFactor = 1;

        magOffset.x = ((magXMax - magXMin) / 2 - magXMax) * magXScaleFactor;
        magOffset.y = ((magYMax - magYMin) / 2 - magYMax) * magYScaleFactor;
        magOffset.z = 0.0f; // 假设Z轴偏移为0

        // 将结果打印到控制台或存储在某处
        Logger.Info("Magnetometer calibration complete.");
        Logger.Info($"MagOffset: {magOffset}, MagXScaleFactor: {magXScaleFactor}, MagYScaleFactor: {magYScaleFactor}");
    }

    private Vector3 CalculateOffset(List<Vector3> DataList)
    {
        Vector3 offset = Vector3.zero;

        foreach (Vector3 data in DataList)
        {
            offset += data;
        }

        return offset / DataList.Count;
    }

    public Vector3 GetAccelData()
    {
        SensorData.Instance.GetAccelData(out float accelX, out float accelY, out float accelZ);
        //Logger.Info("getDataMethod: " + accelX + "  " + accelY + "  " + accelZ);
        return new Vector3(accelX, accelY, accelZ);
        

    }

    public Vector3 GetGyroData()
    {
        SensorData.Instance.GetGyroData(out float gyroX, out float gyroY, out float gyroZ);
        return new Vector3(gyroX, gyroY, gyroZ);
    }

    public Vector3 GetMagData()
    {
        SensorData.Instance.GetMagData(out float magX, out float magY, out float magZ);
        return new Vector3(magX, magY, magZ);
    }
}