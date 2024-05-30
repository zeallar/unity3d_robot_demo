using System;
using System.Collections.Generic;
using UnityEngine;

public static class MainThreadInvoker
{
    private static readonly Queue<Action> _executionQueue = new Queue<Action>();

    public static void Update()
    {
        lock (_executionQueue)
        {
            while (_executionQueue.Count > 0)
            {
                _executionQueue.Dequeue().Invoke();
            }
        }
    }

    public static void Enqueue(Action action)
    {
        lock (_executionQueue)
        {
            _executionQueue.Enqueue(action);
        }
    }
}

public class MainThreadInvokerUpdater : MonoBehaviour
{
    void Update()
    {
        MainThreadInvoker.Update();
    }
}
