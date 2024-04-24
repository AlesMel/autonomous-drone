using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Wind : MonoBehaviour
{

    [SerializeField]
    public float m_maxWindStrength = 0.0f;
    private float windStrength = 0.0f;
    [Range(1f, 5)]
    [SerializeField]
    public int changeEverySeconds = 1;


    private void Start()
    {
        // Start changing rotation every 3 seconds, repeating every 3 seconds
        InvokeRepeating("ChangeRotation", 0f, changeEverySeconds);
    }
    private void ChangeRotation()
    {
        // Generate random strength
        windStrength = Random.Range(0, m_maxWindStrength);

        // Generate random angles for each axis
        float xAngle = Random.Range(0, 360);
        float yAngle = Random.Range(0, 360);
        float zAngle = Random.Range(0, 360);

        // Set the new random rotation
        transform.rotation = Quaternion.Euler(xAngle, yAngle, zAngle);
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
