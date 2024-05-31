using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using System.Threading.Tasks;
using System.Collections.Generic;


public class MainManager : MonoBehaviour
{
    private IMUManager imuManager;
    private IMUDataProcessor dataProcessor;
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
        dataProcessor = GetComponent<IMUDataProcessor>();
        attitudeCalculator = GetComponent<AttitudeCalculator>();
        if (imuManager == null || dataProcessor == null || attitudeCalculator == null)
        {
            Debug.LogError("One or more components are not assigned.");
            return;
        }
        StartCalibration();
        // 初始化姿态模块
        attitudeCalculator.InitAttitude();
        await Initialize();
    }

    private async Task Initialize()
    {
        dataProcessor.InitializeProcessor();
    }

    void OnDestroy()
    {
        isRunning = false;
        if (receiveThread != null)
        {
            receiveThread.Abort();
            receiveThread = null;
        }
        if (udpClient != null)
        {
            udpClient.Close();
            udpClient = null;
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
            if (!SensorData.Instance.hasNewlData) {
                return;
            }
            // 将九轴数据转换为实际物理数据
            dataProcessor.ProcessData(out Vector3 accelData, out Vector3 gyroData, out Vector3 magData);
            string accelLog = $"ProcessData after accelData: {accelData}, gyroData: {gyroData}, magData: {magData}";
            Logger.Info(accelLog);
            attitudeCalculator.CalculateAttitude(Time.deltaTime,accelData, gyroData, magData);
            Vector3 eulerAngles = attitudeCalculator.eulerAngles;
            string gyroLog = "Euler Angles: " + eulerAngles;
            Debug.Log(gyroLog);
            Logger.Info(gyroLog);
        }
        finally
        {
            mutex.ReleaseMutex();
        }
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
                        //Logger.Info("Calibration Store.");
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
