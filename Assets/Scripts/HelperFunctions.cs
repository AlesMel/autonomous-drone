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
}
