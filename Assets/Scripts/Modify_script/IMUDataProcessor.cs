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
            (accelData.x - imuManager.accelOffset.x) / 9.8f,
            (accelData.y - imuManager.accelOffset.y) / 9.8f,
            (accelData.z - imuManager.accelOffset.z) / 9.8f
        );

        gyroData = new Vector3(
            (gyroData.x - imuManager.gyroOffset.x),
            (gyroData.y - imuManager.gyroOffset.y),
            (gyroData.z - imuManager.gyroOffset.z) 
        );

        magData = new Vector3(
            (rawMag.x + imuManager.magOffset.x),
            (rawMag.y + imuManager.magOffset.y) ,
            (rawMag.z + imuManager.magOffset.z) 
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
