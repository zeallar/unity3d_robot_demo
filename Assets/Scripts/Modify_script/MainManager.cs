using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using System.Threading.Tasks;

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
    private bool imuInitialized = false;

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
        await Initialize();
    }

    private async Task Initialize()
    {
        await imuManager.InitializeIMU();
        imuInitialized = true;
        dataProcessor.InitializeProcessor();
        attitudeCalculator.Initialize();
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
        if (!imuInitialized)
        {
            // 如果初始化未完成，则直接返回
            return;
        }
        mutex.WaitOne();
        try
        {
            dataProcessor.ProcessData(out Vector3 accelData, out Vector3 gyroData, out Vector3 magData);
            attitudeCalculator.CalculateAttitude(accelData, gyroData, magData, Time.deltaTime);
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
            }

            yield return null;
        }
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
}
