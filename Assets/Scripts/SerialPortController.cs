using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO.Ports;
using System.Threading;
using System;

public class SerialPortController : MonoBehaviour
{
    private SerialPort serialPort;
    private Thread readThread;
    private bool isRunning;
    private string receivedData;


    public string portName = "COM26"; // 串口名称，根据实际情况修改
    public int baudRate = 115200; // 波特率，需与设备端设置一致
    public int timeoutMilliseconds = 1000; // 超时时间，单位为毫秒
    // Start is called before the first frame update
    void Start()
    {
        // 初始化串口
        serialPort = new SerialPort(portName, baudRate); // 替换为你的串口端口和波特率
        serialPort.ReadTimeout = timeoutMilliseconds;

        try
        {
            serialPort.Open();
            Debug.Log("串口已打开");
        }
        catch (Exception e)
        {
            Debug.LogError("无法打开串口: " + e.Message);
        }
        // 开启读取线程
        isRunning = true;
        readThread = new Thread(ReadSerialPort);
        readThread.Start();

        // 模拟数据包序列
        string[] testPackets = {
        "0000000A00000000000000000000DF2D", // 向前移动
        "0000000A00000000000000000000DF2D", // 继续向前移动
        "00000000000000000000000000002A3E", // 停止
        "00000A0000000000000000000000C3F6", // 跳跃
        "000000000000000A0000000000003B4C", // 左转
        "00000000000000F60000000000003C5D", // 右转
        "000A0000000000000000000000004D7E", // 向右移动
        "00F60000000000000000000000004E8F"  // 向左移动
        };
        foreach (string packet in testPackets)
        {
            SensorDataUtils.ParseSensorData(packet);
            // 模拟延时
            System.Threading.Thread.Sleep(100); // 100毫秒延时
        }

    }

    void Update()
    {
        // 在主线程中处理接收到的数据
        if (!string.IsNullOrEmpty(receivedData))
        {
            SensorDataUtils.ParseSensorData(receivedData);
            receivedData = string.Empty;
        }
    }
    void ReadSerialPort()
    {
        while (isRunning)
        {
            try
            {
                // 读取数据
                string data = serialPort.ReadLine();
                lock (this)
                {
                    receivedData = data;
                }
            }
            catch (TimeoutException)
            {
                // 超时处理
            }
            catch (Exception e)
            {
                Debug.LogError("串口读取错误: " + e.Message);
            }
        }
    }
    void OnApplicationQuit()
    {
        // 停止读取线程
        isRunning = false;
        if (readThread != null && readThread.IsAlive)
        {
            readThread.Join();
        }

        // 关闭串口
        if (serialPort != null && serialPort.IsOpen)
        {
            serialPort.Close();
            Debug.Log("串口已关闭");
        }
    }
  
    public void SendData(string data)
    {
        if (serialPort != null && serialPort.IsOpen)
        {
            serialPort.WriteLine(data);
            Debug.Log("发送的数据: " + data);
        }
        else
        {
            Debug.LogWarning("串口未打开，无法发送数据");
        }
    }

}
