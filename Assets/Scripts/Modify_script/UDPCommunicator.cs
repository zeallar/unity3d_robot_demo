using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

public class UDPCommunicator : MonoBehaviour
{
    private UdpClient udpClient;
    private IPEndPoint remoteEndPoint;
    private bool shouldSendData = false;

    public string remoteIP = "192.168.2.113"; // 远程服务器IP地址
    public int remotePort = 9999; // 远程服务器端口
    public int localPort = 9995; // 本地端口

    void Start()
    {
        remoteEndPoint = new IPEndPoint(IPAddress.Parse(remoteIP), remotePort);
        InitializeUdpClient(localPort);
    }

    void InitializeUdpClient(int port)
    {
        try
        {
            udpClient = new UdpClient(port); // 使用本地端口初始化UDP客户端
            udpClient.BeginReceive(new AsyncCallback(ReceiveCallback), null);
            Debug.Log($"UDP client initialized on local port {port}");
        }
        catch (SocketException e)
        {
            Debug.LogError($"Failed to initialize UDP client on port {port}: {e.Message}");
            if (e.SocketErrorCode == SocketError.AddressAlreadyInUse)
            {
                Debug.Log("Attempting to use a different port...");
                InitializeUdpClient(port + 1); // 递增端口号重试
            }
        }
    }

    void ReceiveCallback(IAsyncResult ar)
    {
        IPEndPoint localEndPoint = new IPEndPoint(IPAddress.Any, 0);
        byte[] receiveBytes = udpClient.EndReceive(ar, ref localEndPoint);
        string receiveString = Encoding.ASCII.GetString(receiveBytes);
        Debug.Log($"Received: {receiveString}");

        if (receiveString.Trim() == "123")
        {
            shouldSendData = true;
            Logger.Info("Start sending data...");
        }

        udpClient.BeginReceive(new AsyncCallback(ReceiveCallback), null);
    }

    public void SendData(Vector3 eulerAngles)
    {
        if (shouldSendData)
        {
            try
            {
                string message = $"{eulerAngles.x},{eulerAngles.y},{eulerAngles.z}";
                byte[] data = Encoding.ASCII.GetBytes(message);
                udpClient.Send(data, data.Length, remoteEndPoint);
                Logger.Info($"Sent data: {message}");
            }
            catch (SocketException e)
            {
                Debug.LogError($"Failed to send UDP data: {e.Message}");
            }
        }
    }

    void OnApplicationQuit()
    {
        if (udpClient != null) udpClient.Close();
    }
}
