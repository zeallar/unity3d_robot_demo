using UnityEngine;
using System.Collections.Generic;

public class IMUDataProcessor : MonoBehaviour
{
    private IMUManager imuManager;
    private Queue<Vector3> accelDataQueue = new Queue<Vector3>();
    private Queue<Vector3> gyroDataQueue = new Queue<Vector3>();
    private Queue<Vector3> magDataQueue = new Queue<Vector3>();
    private const int windowSize = 10;

    public void InitializeProcessor()
    {
        imuManager = GetComponent<IMUManager>();
    }

    public void ProcessData(out Vector3 accelData, out Vector3 gyroData, out Vector3 magData)
    {
        Vector3 rawAccel = imuManager.GetAccelData();
        Vector3 rawGyro = imuManager.GetGyroData();
        Vector3 rawMag = imuManager.GetMagData();

        accelData = ApplyWindowFilter(accelDataQueue, rawAccel);
        gyroData = ApplyWindowFilter(gyroDataQueue, rawGyro);
        magData = ApplyWindowFilter(magDataQueue, rawMag);

        
        accelData = new Vector3(
            (accelData.x - imuManager.accelOffset.x) / 16393.0f,
            (accelData.y - imuManager.accelOffset.y) / 16393.0f,
            (accelData.z - imuManager.accelOffset.z) / 16393.0f
        );

        gyroData = new Vector3(
            (gyroData.x - imuManager.gyroOffset.x) / 57.1f,
            (gyroData.y - imuManager.gyroOffset.y) / 57.1f,
            (gyroData.z - imuManager.gyroOffset.z) / 57.1f
        );

        magData = new Vector3(
            (rawMag.x + imuManager.magOffset.x) * 1.5f,
            (rawMag.y + imuManager.magOffset.y) * 1.5f,
            (rawMag.z + imuManager.magOffset.z) * 1.5f
        );
    }

    private Vector3 ApplyWindowFilter(Queue<Vector3> dataQueue, Vector3 newData)
    {
        if (dataQueue.Count >= windowSize)
        {
            dataQueue.Dequeue();
        }

        dataQueue.Enqueue(newData);

        Vector3 sum = Vector3.zero;
        foreach (Vector3 data in dataQueue)
        {
            sum += data;
        }

        return sum / dataQueue.Count;
    }
}
