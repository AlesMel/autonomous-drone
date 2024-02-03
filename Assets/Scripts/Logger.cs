using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Logger
{

    private static readonly bool shouldLog = false;

    public static void LogMessage(string message, bool isWarning = false)
    {
        if (shouldLog)
        {
            if (isWarning)
            {
                Debug.LogWarning(message);
            }
            else
            {
                Debug.Log(message);
            }
        }

    }
}

