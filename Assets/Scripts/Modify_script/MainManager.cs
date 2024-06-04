﻿using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using System.Collections.Generic;

public class MainManager : MonoBehaviour
{
    private IMUManager imuManager;
    private AttitudeCalculator attitudeCalculator;

    private UdpClient udpClient;
    private Thread receiveThread;
    private bool isRunning;

    private string remoteIPAddress = "192.168.100.124";
    private int remotePort = 8888;
    private string localIPAddress = "192.168.100.132";
    private int localPort = 7777;

    private ConcurrentQueue<string> dataQueue = new ConcurrentQueue<string>();
    private Mutex mutex = new Mutex();

    private bool calibrationFlag = false;
    private List<Vector3> accelDataList = new List<Vector3>();
    private List<Vector3> gyroDataList = new List<Vector3>();
    private List<Vector3> magDataList = new List<Vector3>();

    // 定义窗口大小
    private const int windowSize = 10;

    // 队列用于存储传感器数据
    private Queue<Vector3> accelerometerData = new Queue<Vector3>();
    private Queue<Vector3> gyroscopeData = new Queue<Vector3>();
    private Queue<Vector3> magnetometerData = new Queue<Vector3>();

    // 传感器数据（已滤波和转换）
    private Vector3 filteredAccData;
    private Vector3 filteredGyroData;
    private Vector3 filteredMagData;

    private TCPSender tcpSender;
    private MadgwickAHRS madgwick;

    private async void Start()
    {
        udpClient = new UdpClient(new IPEndPoint(IPAddress.Parse(localIPAddress), localPort));
        isRunning = true;
        receiveThread = new Thread(new ThreadStart(RotReceiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();

        // 发送字符 "123"
        SendData("123");

        StartCoroutine(RotProcessData());

        imuManager = GetComponent<IMUManager>();
        if (imuManager == null)
        {
            Debug.LogError("IMUManager component is not assigned.");
            return;
        }
        StartCalibration();
        // 初始化队列
        InitializeQueue(accelerometerData, windowSize);
        InitializeQueue(gyroscopeData, windowSize);
        InitializeQueue(magnetometerData, windowSize);

        tcpSender = gameObject.AddComponent<TCPSender>();
        madgwick = new MadgwickAHRS(); // 这里假设采样频率为256Hz，beta参数为0.05;
    }

    void OnDestroy()
    {
        isRunning = false;
        if (receiveThread != null)
        {
            receiveThread.Abort();
            receiveThread = null;
        }

        tcpSender.OnApplicationQuit();

        if (udpClient != null)
        {
            udpClient.Close();
            udpClient = null;
        }
    }

    void InitializeQueue(Queue<Vector3> queue, int size)
    {
        for (int i = 0; i < size; i++)
        {
            queue.Enqueue(Vector3.zero);
        }
    }

    private void Update()
    {
        if (!imuManager.imuInitialized)
        {
            // 如果初始化未完成，则直接返回
            Debug.Log("IMU is not initialized");
            return;
        }

        mutex.WaitOne();
        try
        {
            if (!SensorData.Instance.hasNewlData)
            {
                return;
            }
            // 读取传感器数据并加入队列
            Vector3 acc = imuManager.GetAccelData();
            Vector3 gyro = imuManager.GetGyroData();
            Vector3 mag = imuManager.GetMagData();

            //零偏校准
            acc -= imuManager.accelOffset;
            acc -= imuManager.gyroOffset;
            // 滤波处理
            filteredAccData = FilterData(accelerometerData, acc);
            filteredGyroData = FilterData(gyroscopeData, gyro);
            filteredMagData = FilterData(magnetometerData, mag);


            // 使用Madgwick滤波器更新四元数并计算欧拉角
            madgwick.MadgwickAHRSupdateIMU(filteredGyroData.x, filteredGyroData.y, filteredGyroData.z, filteredAccData.x, filteredAccData.y, filteredAccData.z);
            Vector3 euler =madgwick.GetEulerAngles();

            // 发送欧拉角
            tcpSender.SendEulerAngles(euler);
            Logger.Info("Euler Angles: " + euler);
        }
        finally
        {
            mutex.ReleaseMutex();
        }
    }

    Vector3 FilterData(Queue<Vector3> queue, Vector3 newData)
    {
        if (queue.Count >= windowSize)
        {
            queue.Dequeue();
        }
        queue.Enqueue(newData);

        Vector3 sum = Vector3.zero;
        foreach (var data in queue)
        {
            sum += data;
        }
        return sum / queue.Count;
    }

    private void RotReceiveData()
    {
        try
        {
            while (isRunning)
            {
                IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Parse(remoteIPAddress), remotePort);
                byte[] receiveBytes = udpClient.Receive(ref remoteEndPoint);
                string receiveString = BitConverter.ToString(receiveBytes).Replace("-", "");
                // 将接收到的数据添加到队列
                dataQueue.Enqueue(receiveString);
            }
        }
        catch (Exception e)
        {
            Debug.LogError("UDP接收数据出错: " + e.Message);
        }
    }

    private IEnumerator RotProcessData()
    {
        while (isRunning)
        {
            if (dataQueue.TryDequeue(out string receiveString))
            {
                // 解析接收到的数据并更新SensorData
                SensorDataUtils.ParseSensorData(receiveString);

                if (calibrationFlag)
                {
                    // 校准标志位为 true 时，存储数据
                    Vector3 accelData = imuManager.GetAccelData();
                    Vector3 gyroData = imuManager.GetGyroData();
                    Vector3 magData = imuManager.GetMagData();
                    if (!IsVectorZero(accelData) && !IsVectorZero(gyroData) && !IsVectorZero(magData))
                    {
                        StoreCalibrationData(accelData, gyroData, magData);
                    }
                }
            }

            yield return null;
        }
    }

    private bool IsVectorZero(Vector3 vector)
    {
        return vector.x == 0 && vector.y == 0 && vector.z == 0;
    }

    private void SendData(string message)
    {
        try
        {
            IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Parse(remoteIPAddress), remotePort);
            byte[] sendBytes = Encoding.ASCII.GetBytes(message);
            udpClient.Send(sendBytes, sendBytes.Length, remoteEndPoint);
            Debug.Log("Sent message: " + message);
        }
        catch (Exception e)
        {
            Debug.LogError("UDP发送数据出错: " + e.Message);
        }
    }

    public void StartCalibration()
    {
        calibrationFlag = true;
        accelDataList.Clear();
        gyroDataList.Clear();
        magDataList.Clear();
        Logger.Info("Calibration started.");
    }

    public void StopCalibration()
    {
        calibrationFlag = false;
        // 在此处理存储的校准数据
        ProcessCalibrationData();
        Logger.Info("Calibration finished.");
    }

    private void StoreCalibrationData(Vector3 accelData, Vector3 gyroData, Vector3 magData)
    {
        if (accelDataList.Count < 20)
        {
            accelDataList.Add(accelData);
            gyroDataList.Add(gyroData);
            magDataList.Add(magData);
        }
        if (accelDataList.Count >= 20)
        {
            Logger.Info("Calibration stop.");
            StopCalibration();
        }
    }

    private void ProcessCalibrationData()
    {
        // 处理校准数据的逻辑
        Debug.Log("Processing calibration data...");
        // 示例：输出校准数据
        imuManager.SetAccelGyroOffsets(accelDataList, gyroDataList, magDataList);
    }
}
