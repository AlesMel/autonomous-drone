using System.Collections;
using System.Collections.Generic;
using Unity.MLAgentsExamples;
using UnityEngine;

public class Wind : MonoBehaviour
{

    [SerializeField]
    public float m_maxWindStrength = 0.0f;
    private float windStrength = 0.0f;
    [Range(1f, 5)]
    [SerializeField]
    public int changeEverySeconds = 1;

    double[] strengths = new double[]
        {
            0.207034, 0.1983741, 0.3085291, 0.1402218, 0.1605727, 0.007303238
        };

    // Array of quaternions (as a 2D array)
    double[,] quaternions = new double[,]
    {
            {-0.44248, -0.53861, 0.54458, -0.46642},
            {-0.70032, -0.36117, 0.55703, 0.26236},
            {0.87174, -0.00822, -0.32782, 0.36405},
            {0.47080, 0.39237, -0.71069, 0.34541},
            {0.52734, -0.68472, 0.36934, 0.34156},
            {0.42358, -0.86486, 0.04888, 0.26497}
    };

    private int currentIndex = 0;

    private void Start()
    {
        // Start changing rotation every 3 seconds, repeating every 3 seconds
        InvokeRepeating("ChangeRotation", 0f, changeEverySeconds);
    }
    private void ChangeRotation()
    {
        // Generate random strength
        /*windStrength = Random.Range(0, m_maxWindStrength);
        // Generate random angles for each axis
        float xAngle = Random.Range(0, 360);
        float yAngle = Random.Range(0, 360);
        float zAngle = Random.Range(0, 360);

        // Set the new random rotation
        transform.rotation = Quaternion.Euler(xAngle, yAngle, zAngle);*/

        // For plotting
        // Use predefined strength
        windStrength = (float)strengths[currentIndex];
        // Use predefined quaternion
        Quaternion rot = new Quaternion((float)quaternions[currentIndex, 0], (float)quaternions[currentIndex, 1],
                                        (float)quaternions[currentIndex, 2], (float)quaternions[currentIndex, 3]);
        transform.rotation = rot;

        Debug.Log(windStrength + ", " + transform.rotation);

        currentIndex++;
        if (currentIndex >= strengths.Length)
            currentIndex = 0;


    }

    private void OnTriggerStay(Collider other)
    {
        var hitObject = other.gameObject;
        if (hitObject != null && hitObject.name == "Body")
        {
            var rb = hitObject.GetComponentInParent<Rigidbody>();
            var dir = transform.up;
            rb.AddForce(dir * windStrength);
        }
    }
}
