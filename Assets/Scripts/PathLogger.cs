using System.Collections;
using System.Collections.Generic;
using System.Text;
using System.IO;
using UnityEngine;

public class PathLogger : MonoBehaviour
{
    [SerializeField]
    private GameObject trackingObject;
    private List<string> data = new List<string>();

    private Rigidbody rb;  // Rigidbody component of the tracking object

    void Start()
    {
        if (trackingObject != null)
        {
            rb = trackingObject.GetComponent<Rigidbody>();
            if (rb == null)
            {
                Debug.LogError("No Rigidbody component found on the tracking object.");
            }
        }
    }

    void FixedUpdate()
    {
        if (rb != null)
        {
            Vector3 position = trackingObject.transform.position;
            Vector3 velocity = rb.velocity;
            Vector3 angularVelocity = rb.angularVelocity;
            float velocityMagnitude = velocity.magnitude;
            float angularVelocityMagnitude = angularVelocity.magnitude;
            // Record position, velocity, angular velocity, and their magnitudes
            data.Add($"{position.x},{position.y},{position.z},{velocity.x},{velocity.y},{velocity.z},{angularVelocity.x},{angularVelocity.y},{angularVelocity.z},{velocityMagnitude},{angularVelocityMagnitude}");
        }
    }

    private void OnDisable()
    {
        WriteDataToCSV();
    }

    private void WriteDataToCSV()
    {
        StringBuilder sb = new StringBuilder();
        sb.AppendLine("x,y,z,vx,vy,vz,ax,ay,az,v_mag,a_mag");  // Header row for CSV
        foreach (string line in data)
        {
            sb.AppendLine(line);
        }
        string filePath = Application.persistentDataPath + "/positions.csv";
        File.WriteAllText(filePath, sb.ToString());
        Debug.Log("Data saved to " + filePath);
    }
}
