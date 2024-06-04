using System;
using UnityEngine;


public static class SensorDataUtils
{
    public static void ParseSensorData(string hexData)
    {
        // Convert hex string to byte array
        byte[] data = HexStringToByteArray(hexData);

        if (data.Length < 20) return; // 数据长度不够
        // 解析数据
        float accelX = Convert.ToInt16(hexData.Substring(0, 4), 16);
        float accelY = Convert.ToInt16(hexData.Substring(4, 4), 16);
        float accelZ = Convert.ToInt16(hexData.Substring(8, 4), 16);
        float gyroX = Convert.ToInt16(hexData.Substring(12, 4), 16);
        float gyroY = Convert.ToInt16(hexData.Substring(16, 4), 16);
        float gyroZ = Convert.ToInt16(hexData.Substring(20, 4), 16);
        float magX = Convert.ToInt16(hexData.Substring(24, 4), 16);
        float magY = Convert.ToInt16(hexData.Substring(28, 4), 16);
        float magZ = Convert.ToInt16(hexData.Substring(32, 4), 16);
        float temp = Convert.ToInt16(hexData.Substring(36, 4), 16);

        //Debug.Log("temp = " + temp);
        // 计算和校验 CRC16
        byte[] payload = new byte[data.Length - 2];
        Array.Copy(data, 0, payload, 0, data.Length - 2);
        string crcReceived = BitConverter.ToString(data, data.Length - 2).Replace("-", "");
        string crcCalculated = CalculateCRC16Modbus(payload);
        if (!crcReceived.Equals(crcCalculated, StringComparison.OrdinalIgnoreCase))
        {
            Debug.LogError("CRC校验失败: 接收到的CRC=" + crcReceived + " 计算得到的CRC=" + crcCalculated);
            return;
        }
        // 更新共享数据
        SensorData.Instance.SetTemp(temp);
        SensorData.Instance.SetAccelData(accelX, accelY, accelZ);
        SensorData.Instance.SetGyroData(gyroX, gyroY, gyroZ);
        SensorData.Instance.SetMagData(magX, magY, magZ);
    }

    public static string CalculateCRC16Modbus(byte[] data)
    {
        uint i, j;
        uint crc16 = 0xFFFF;
        uint crc_temp;
        for (i = 0; i < data.Length; i++)
        {
            crc16 ^= data[i];
            for (j = 0; j < 8; j++)
            {
                if ((crc16 & 0x0001) != 0)
                {
                    crc16 >>= 1; // 右移一位
                    crc16 ^= 0xA001; // 异或多项式0xA001
                }
                else
                {
                    crc16 >>= 1; // 右移一位
                }
            }
        }
        crc_temp = (crc16 & 0x00ff) << 8;//改一下，让低位在前，删除这两行就是高位在前
        crc16 = crc_temp + (crc16 >> 8);
        return crc16.ToString("X4");
    }
    private static byte[] HexStringToByteArray(string hex)
    {
        int numChars = hex.Length;
        byte[] bytes = new byte[numChars / 2];
        for (int i = 0; i < numChars; i += 2)
        {
            bytes[i / 2] = Convert.ToByte(hex.Substring(i, 2), 16);
        }
        return bytes;
    }


}
