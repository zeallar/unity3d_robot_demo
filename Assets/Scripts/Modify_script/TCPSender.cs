using System;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

public class TCPSender : MonoBehaviour
{
    public string ipAddress = "127.0.0.1";
    public int port = 8080;

    private TcpClient client;
    private NetworkStream stream;

    void Start()
    {
        Connect();
    }

    void Update()
    {
        if (client != null && client.Connected)
        {
            SendEulerAngles();
        }
    }

    void Connect()
    {
        try
        {
            client = new TcpClient(ipAddress, port);
            stream = client.GetStream();
            Debug.Log("Connected to server");
        }
        catch (Exception e)
        {
            Debug.Log("Connection failed: " + e.Message);
        }
    }

    public void SendEulerAngles(Vector3 eulerAngles)
    {
        string message = $"{eulerAngles.x},{eulerAngles.y},{eulerAngles.z}\n";
        byte[] data = Encoding.ASCII.GetBytes(message);
        stream.Write(data, 0, data.Length);
    }

    void OnApplicationQuit()
    {
        stream.Close();
        client.Close();
    }
}
