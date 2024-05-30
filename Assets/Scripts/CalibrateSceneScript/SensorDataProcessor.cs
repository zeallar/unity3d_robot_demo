using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class SensorDataProcessor : MonoBehaviour
{
    public Vector3 accOffset;
    public Vector3 gyroOffset;
    public Vector3 magOffset;
    public float thresholdFactor = 3; // 用于自适应阈值的因子

    // 静态字典用于存储每个动作的偏移量
    private static Dictionary<string, Vector3> accOffsets = new Dictionary<string, Vector3>();
    private static Dictionary<string, Vector3> gyroOffsets = new Dictionary<string, Vector3>();
    private static Dictionary<string, Vector3> magOffsets = new Dictionary<string, Vector3>();
    void Start()
    {
        if (HasAnyOffsets())
        {
            LoadOffsets();
            LogOffsets("Startup");
        }
    }

    public void ProcessData(List<Vector3> accData, List<Vector3> gyroData, List<Vector3> magData, string action)
    {
        // 计算静止状态下的平均值作为偏移量
        accOffset = GetMean(accData);
        gyroOffset = GetMean(gyroData);
        magOffset = GetMean(magData);

        // 保存偏移量到字典
        accOffsets[action] = accOffset;
        gyroOffsets[action] = gyroOffset;
        magOffsets[action] = magOffset;

        SaveOffsets();
        LogOffsets("Calibration Complete "+ action);

        // 偏移校正
        List<Vector3> accDataCorrected = CorrectOffset(accData, accOffset);
        List<Vector3> gyroDataCorrected = CorrectOffset(gyroData, gyroOffset);
        List<Vector3> magDataCorrected = CorrectOffset(magData, magOffset);

        // 移动平均滤波器（窗口大小为3）
        int windowSize = 3;
        List<Vector3> accDataFiltered = ApplyMovingAverageFilter(accDataCorrected, windowSize);
        List<Vector3> gyroDataFiltered = ApplyMovingAverageFilter(gyroDataCorrected, windowSize);
        List<Vector3> magDataFiltered = ApplyMovingAverageFilter(magDataCorrected, windowSize);

        // 阈值处理
        List<Vector3> accDataFinal = ApplyAdaptiveThreshold(accDataFiltered, thresholdFactor);
        List<Vector3> gyroDataFinal = ApplyAdaptiveThreshold(gyroDataFiltered, thresholdFactor);
        List<Vector3> magDataFinal = ApplyAdaptiveThreshold(magDataFiltered, thresholdFactor);

        // 输出处理后的数据
        //Debug.Log("加速度数据校正后：" + string.Join(", ", accDataFinal.Select(v => v.ToString()).ToArray()));
        //Debug.Log("陀螺仪数据校正后：" + string.Join(", ", gyroDataFinal.Select(v => v.ToString()).ToArray()));
        //Debug.Log("磁力计数据校正后：" + string.Join(", ", magDataFinal.Select(v => v.ToString()).ToArray()));

        //Logger.Info("加速度数据阈值处理前：" + string.Join(", ", accDataFiltered.Select(v => v.ToString()).ToArray()));
        //Logger.Info("陀螺仪数据阈值处理前：" + string.Join(", ", gyroDataFiltered.Select(v => v.ToString()).ToArray()));
        //Logger.Info("磁力计数据阈值处理前：" + string.Join(", ", magDataFiltered.Select(v => v.ToString()).ToArray()));



        Logger.Info("加速度数据校正后：" + string.Join(", ", accDataFinal.Select(v => v.ToString()).ToArray()));
        Logger.Info("陀螺仪数据校正后：" + string.Join(", ", gyroDataFinal.Select(v => v.ToString()).ToArray()));
        Logger.Info("磁力计数据校正后：" + string.Join(", ", magDataFinal.Select(v => v.ToString()).ToArray()));
    }

    private void SaveOffsets()
    {
        foreach (var kvp in accOffsets)
        {
            PlayerPrefs.SetString($"{kvp.Key}_accOffset", kvp.Value.ToString());
            PlayerPrefs.SetString($"{kvp.Key}_gyroOffset", gyroOffsets[kvp.Key].ToString());
            PlayerPrefs.SetString($"{kvp.Key}_magOffset", magOffsets[kvp.Key].ToString());
        }
        PlayerPrefs.Save();
    }

    private void LoadOffsets()
    {
        foreach (var action in new[] { "请保持静止", "请向前一步", "请后退一步", "请左转", "请右转" })
        {
            if (PlayerPrefs.HasKey($"{action}_accOffset"))
            {
                accOffsets[action] = ParseVector3(PlayerPrefs.GetString($"{action}_accOffset"));
                gyroOffsets[action] = ParseVector3(PlayerPrefs.GetString($"{action}_gyroOffset"));
                magOffsets[action] = ParseVector3(PlayerPrefs.GetString($"{action}_magOffset"));
            }
        }
    }

    public List<Vector3> ApplyPreprocessing(List<Vector3> data, Vector3 offset)
    {
        var correctedData = CorrectOffset(data, offset);
        var filteredData = ApplyMovingAverageFilter(correctedData, 3);
        return ApplyAdaptiveThreshold(filteredData, thresholdFactor); // 使用自适应阈值法
    }
    private bool HasAnyOffsets()
    {
        foreach (var action in new[] { "请保持静止", "请向前一步", "请后退一步", "请左转", "请右转" })
        {
            if (PlayerPrefs.HasKey($"{action}_accOffset"))
            {
                return true;
            }
        }
        return false;
    }
    private void LogOffsets(string context)
    {
        Logger.Info($"{context}: 当前所有偏移量数据如下:");
        foreach (var action in accOffsets.Keys)
        {
            Logger.Info($"{action} - 加速度偏移量: {accOffsets[action]}");
            // 如果需要，也可以记录gyroOffsets和magOffsets
            Logger.Info($"{action} - 陀螺仪偏移量: {gyroOffsets[action]}");
            Logger.Info($"{action} - 磁力计偏移量: {magOffsets[action]}");
        }
    }
    public Vector3 GetMean(List<Vector3> data)
    {
        if (data.Count == 0) return Vector3.zero;
        float sumX = 0, sumY = 0, sumZ = 0;
        foreach (var point in data)
        {
            sumX += point.x;
            sumY += point.y;
            sumZ += point.z;
        }
        return new Vector3(sumX / data.Count, sumY / data.Count, sumZ / data.Count);
    }

    public List<Vector3> CorrectOffset(List<Vector3> data, Vector3 offset)
    {
        return data.Select(point => new Vector3(point.x - offset.x, point.y - offset.y, point.z - offset.z)).ToList();
    }

    public List<Vector3> ApplyMovingAverageFilter(List<Vector3> data, int windowSize)
    {
        List<Vector3> filteredData = new List<Vector3>();
        for (int i = 0; i <= data.Count - windowSize; i++)
        {
            float windowSumX = 0, windowSumY = 0, windowSumZ = 0;
            for (int j = 0; j < windowSize; j++)
            {
                windowSumX += data[i + j].x;
                windowSumY += data[i + j].y;
                windowSumZ += data[i + j].z;
            }
            filteredData.Add(new Vector3(windowSumX / windowSize, windowSumY / windowSize, windowSumZ / windowSize));
        }
        return filteredData;
    }

    public List<Vector3> ApplyAdaptiveThreshold(List<Vector3> data, float factor)
    {
        if (data.Count == 0) return data;

        Vector3 mean = GetMean(data);
        Vector3 varianceSum = Vector3.zero;

        //foreach (var point in data)
        //{
        //    varianceSum.x += Mathf.Pow(point.x - mean.x, 2);
        //    varianceSum.y += Mathf.Pow(point.y - mean.y, 2);
        //    varianceSum.z += Mathf.Pow(point.z - mean.z, 2);
        //}

        Vector3 standardDeviation = new Vector3(
            Mathf.Sqrt(varianceSum.x / data.Count),
            Mathf.Sqrt(varianceSum.y / data.Count),
            Mathf.Sqrt(varianceSum.z / data.Count)
        );

        Vector3 threshold = standardDeviation * factor;
        Logger.Info("阈值数据：" + " x:" + standardDeviation.x + " y:" + standardDeviation.y + " z:" + standardDeviation.z + "data.Count" + data.Count);
        List<Vector3> filteredData = new List<Vector3>();
        foreach (var point in data)
        {
            Vector3 filteredPoint = new Vector3(
                Mathf.Abs(point.x) < threshold.x ? 0 : point.x,
                Mathf.Abs(point.y) < threshold.y ? 0 : point.y,
                Mathf.Abs(point.z) < threshold.z ? 0 : point.z
            );
            filteredData.Add(filteredPoint);
        }
        //Logger.Info("输入数据：" + string.Join(", ", data.Select(v => v.ToString()).ToArray()));
        //Logger.Info("输出数据：" + string.Join(", ", filteredData.Select(v => v.ToString()).ToArray()));

        return filteredData;
        //return data.Select(point => new Vector3(
        //    Mathf.Abs(point.x) < threshold.x ? 0 : point.x,
        //    Mathf.Abs(point.y) < threshold.y ? 0 : point.y,
        //    Mathf.Abs(point.z) < threshold.z ? 0 : point.z)).ToList();
    }
    private Vector3 ParseVector3(string value)
    {
        value = value.Trim(new char[] { '(', ')' });
        string[] sArray = value.Split(',');
        return new Vector3(
            float.Parse(sArray[0]),
            float.Parse(sArray[1]),
            float.Parse(sArray[2]));
    }

    public bool HasActionOffset(string action)
    {
        return accOffsets.ContainsKey(action) && gyroOffsets.ContainsKey(action) && magOffsets.ContainsKey(action);
    }

    // 获取存储的偏移量
    public static Vector3 GetAccOffset(string action)
    {
        return accOffsets.ContainsKey(action) ? accOffsets[action] : Vector3.zero;
    }

    public static Vector3 GetGyroOffset(string action)
    {
        return gyroOffsets.ContainsKey(action) ? gyroOffsets[action] : Vector3.zero;
    }

    public static Vector3 GetMagOffset(string action)
    {
        return magOffsets.ContainsKey(action) ? magOffsets[action] : Vector3.zero;
    }
}
