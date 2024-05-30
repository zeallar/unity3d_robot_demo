using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

public class UDPReceiver : MonoBehaviour
{
    private static UDPReceiver instance;
    public static UDPReceiver Instance => instance;

    private UdpClient udpClient;
    private Thread receiveThread;
    private bool isRunning;
    private bool isReceiving;

    private string remoteIPAddress = "192.168.100.124";
    private int remotePort = 8888;
    private string localIPAddress = "192.168.100.132";
    private int localPort = 7777;

    private ConcurrentQueue<string> dataQueue = new ConcurrentQueue<string>();
    private List<Vector3> accDataBuffer = new List<Vector3>();
    private List<Vector3> gyroDataBuffer = new List<Vector3>();
    private List<Vector3> magDataBuffer = new List<Vector3>();

    void Awake()
    {
        if (instance != null && instance != this)
        {
            Destroy(this.gameObject);
        }
        else
        {
            instance = this;
            DontDestroyOnLoad(this.gameObject);
        }
    }

    void Start()
    {
        udpClient = new UdpClient(new IPEndPoint(IPAddress.Parse(localIPAddress), localPort));
        isRunning = true;
        receiveThread = new Thread(new ThreadStart(ReceiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();

        // 发送字符 "123"
        SendData("123");
    }

    void Update()
    {
        while (dataQueue.TryDequeue(out string data))
        {
            ProcessData(data);
        }
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

    private void ReceiveData()
    {
        try
        {
            IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Parse(remoteIPAddress), remotePort);
            while (isRunning)
            {
                if (isReceiving)
                {
                    byte[] receiveBytes = udpClient.Receive(ref remoteEndPoint);
                    string receiveString = BitConverter.ToString(receiveBytes).Replace("-", "");
                    dataQueue.Enqueue(receiveString);
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogError("UDP接收数据出错: " + e.Message);
        }
    }
    public void clearCacheData() {
        accDataBuffer.Clear();
        gyroDataBuffer.Clear();
        magDataBuffer.Clear();
    }
    private void ProcessData(string receiveString)
    {
        // 解析接收到的数据并更新SensorData
        SensorDataUtils.ParseSensorData(receiveString);
        // 从SensorData获取数据
        SensorData.Instance.GetTemp(out float temp);
        SensorData.Instance.GetAccelData(out float accelX, out float accelY, out float accelZ);
        SensorData.Instance.GetGyroData(out float gyroX, out float gyroY, out float gyroZ);
        SensorData.Instance.GetMagData(out float magX, out float magY, out float magZ);

        // 检查数据是否为零
        if (!IsZeroData(accelX, accelY, accelZ) && !IsZeroData(gyroX, gyroY, gyroZ) && !IsZeroData(magX, magY, magZ))
        {
            // 缓存数据
            accDataBuffer.Add(new Vector3(accelX, accelY, accelZ));
            gyroDataBuffer.Add(new Vector3(gyroX, gyroY, gyroZ));
            magDataBuffer.Add(new Vector3(magX, magY, magZ));
        }
    }

    public void StartReceiving()
    {
        isReceiving = true;
    }

    public void StopReceiving()
    {
        isReceiving = false;
    }

    public void GetBufferedData(out List<Vector3> accData, out List<Vector3> gyroData, out List<Vector3> magData)
    {
        accData = accDataBuffer;
        gyroData = gyroDataBuffer;
        magData = magDataBuffer;
    }

    private bool IsZeroData(float x, float y, float z)
    {
        return x == 0 && y == 0 && z == 0;
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
