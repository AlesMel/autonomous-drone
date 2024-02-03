using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class HelperFunctions
{

    public static float Sigmoid(float value, float coefficient = 1)
    {
        return 2 / (1 + Mathf.Exp(-value * coefficient)) - 1;
    }

    public static Vector3 Sigmoid(Vector3 vector, float coefficient = 1)
    {
        vector.x = Sigmoid(vector.x, coefficient);
        vector.y = Sigmoid(vector.y, coefficient);
        vector.z = Sigmoid(vector.z, coefficient);
        return vector;
    }

    /// Calculates a normalized reward falling off from 1 as error increases.
    public static float Reward(float error, float coefficient = 1)
    {
        return Mathf.Min(1, -2 / (1 + Mathf.Exp(-error * coefficient)) + 2);
    }

    public static void LogMessage(bool isWarning, string message)
    {
        if (isWarning)
        {
            Debug.LogWarning(message);
        } else
        {
            Debug.Log(message);
        }
    }
}

