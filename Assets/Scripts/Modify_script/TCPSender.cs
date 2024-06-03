using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
public class TCPSender : MonoBehaviour
{
    public int port = 9995;

    private TcpListener server;
    private TcpClient client;
    private NetworkStream stream;

    void Start()
    {
        StartServer();
    }



    void StartServer()
    {
        try
        {
            server = new TcpListener(IPAddress.Parse("127.0.0.1"), port);
            server.Start();
            server.BeginAcceptTcpClient(new AsyncCallback(OnClientConnected), null);
            Debug.Log("Server started");
        }
        catch (Exception e)
        {
            Debug.Log("Server start failed: " + e.Message);
        }
    }

    void OnClientConnected(IAsyncResult result)
    {
        client = server.EndAcceptTcpClient(result);
        stream = client.GetStream();
        Debug.Log("Client connected");
    }

    public void SendEulerAngles(Vector3 eulerAngles)
    {
        if (client != null && client.Connected)
        {
            string message = $"simples:{eulerAngles.x},{eulerAngles.y},{eulerAngles.z}\r\n";
            byte[] data = Encoding.ASCII.GetBytes(message);
            try
            {
                stream.Write(data, 0, data.Length);
            }
            catch (Exception e)
            {
                Debug.Log("Failed to send data: " + e.Message);
            }
        }
    }

    public void OnApplicationQuit()
    {
        if (stream != null)
            stream.Close();
        if (client != null)
            client.Close();
        if (server != null)
            server.Stop();
    }
}
