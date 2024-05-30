using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;


public class RotUDPReceiver : MonoBehaviour
{
    private UdpClient udpClient;
    private Thread receiveThread;
    private bool isRunning;

    private string remoteIPAddress = "192.168.100.124";
    private int remotePort = 8888;
    private string localIPAddress = "192.168.100.132";
    private int localPort = 7777;


    private ConcurrentQueue<string> dataQueue = new ConcurrentQueue<string>();

    void Start()
    {
        udpClient = new UdpClient(new IPEndPoint(IPAddress.Parse(localIPAddress), localPort));
        isRunning = true;
        receiveThread = new Thread(new ThreadStart(RotReceiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();

        // 发送字符 "123"
        SendData("123");

        StartCoroutine(RotProcessData());
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

    private void RotReceiveData()
    {
        try
        {
            while (isRunning)
            {
                IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Parse(remoteIPAddress), remotePort);
                byte[] receiveBytes = udpClient.Receive(ref remoteEndPoint);
                string receiveString = BitConverter.ToString(receiveBytes).Replace("-", "");
                //Debug.Log("UDP: ReceiveData = " + receiveString + "   length:=" + receiveString.Length);

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
