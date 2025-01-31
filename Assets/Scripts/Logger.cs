using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Logger
{

    private static readonly bool shouldLog = false;
    static string PadRight(string str, int totalWidth)
    {
        return str.PadRight(totalWidth);
    }
    public static void LogMessage(string message, bool isWarning = false, bool forceMessage = false)
    {
        if (shouldLog || forceMessage)
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

