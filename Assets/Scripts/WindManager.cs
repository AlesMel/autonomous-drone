using System.Collections.Generic;
using UnityEngine;

public class WindManager : MonoBehaviour
{
    public float maxWindStrength = 0.1f;
    public Vector2 windChangeIntervalRange = new Vector2(1f, 5f);
    private Vector3 currentWindForce;
    private float nextWindChangeTime;

    private List<Rigidbody> affectedRigidbodies;

    private void Awake()
    {
        affectedRigidbodies = new List<Rigidbody>();

        foreach (Transform child in transform)
        {
            // Try to get the Rigidbody component on the child
            Rigidbody childRb = child.GetComponent<Rigidbody>();
            affectedRigidbodies.Add(childRb);
        }

    }

    void Start()
    {
        ChangeWind();
    }


    void FixedUpdate()
    {
        if (Time.time >= nextWindChangeTime)
        {
            ChangeWind();
        }
    }

    void ChangeWind()
    {
        float windStrength = Random.Range(0, maxWindStrength);
        Vector3 windDirection = Random.insideUnitSphere.normalized;

        currentWindForce = windDirection * windStrength;
        nextWindChangeTime = Time.time + Random.Range(windChangeIntervalRange.x, windChangeIntervalRange.y);

        foreach (Rigidbody rb in affectedRigidbodies)
        {
            rb.AddForceAtPosition(currentWindForce, rb.transform.position * Random.Range(-0.3f, 0.3f), ForceMode.Impulse);
        }
    }
}

