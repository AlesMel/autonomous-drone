using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class HelperFunctions
{
    public static float ShiftedSigmoid(float value, float coefficient = 1)
    {
        return 2 / (1 + Mathf.Exp(-value * coefficient)) - 1;
    }

    public static Vector3 Sigmoid(Vector3 vector, float coefficient = 1)
    {
        vector.x = ShiftedSigmoid(vector.x, coefficient);
        vector.y = ShiftedSigmoid(vector.y, coefficient);
        vector.z = ShiftedSigmoid(vector.z, coefficient);
        return vector;
    }

    /// Calculates a normalized reward falling off from 1 as error increases.
    public static float Reward(float error, float coefficient = 1)
    {
        return Mathf.Min(1, -2 / (1 + Mathf.Exp(-error * coefficient)) + 2);
    }

    public static float GetReward(float normalizedError)
    {
        return Mathf.Pow(1 - Mathf.Pow(normalizedError, 2), 2);
    }

    /// Calculates a weighted reward falling off from 1 as error increases.
    public static float WeightedReward(float error, float weight = 1, float coefficient = 1)
    {
        return 1 - weight + weight * Reward(error, coefficient);
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

    public static float QuantizeValue(float value, float stepSize = 0.001f)
    {
        return Mathf.Round(value / stepSize) * stepSize;
    }

    public static Vector3 QuantizeVector3(Vector3 vector, float stepSize = 0.0001f)
    {
        return new Vector3(
            QuantizeValue(vector.x, stepSize),
            QuantizeValue(vector.y, stepSize),
            QuantizeValue(vector.z, stepSize)
        );
    }

}

