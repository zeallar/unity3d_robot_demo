using System;
using System.IO;

public static class Logger
{
    private static readonly object _lock = new object();
    private static string _logFilePath = "log.txt";

    public static void Initialize(string logFilePath)
    {
        _logFilePath = logFilePath;
    }

    public static void Info(string message)
    {
        WriteLog("INFO", message);
    }

    public static void Warning(string message)
    {
        WriteLog("WARNING", message);
    }

    public static void Error(string message)
    {
        WriteLog("ERROR", message);
    }

    private static void WriteLog(string level, string message)
    {
        lock (_lock)
        {
            using (StreamWriter writer = new StreamWriter(_logFilePath, true))
            {
                writer.WriteLine($"{DateTime.Now:yyyy-MM-dd HH:mm:ss} [{level}] {message}");
            }
        }
    }
}
