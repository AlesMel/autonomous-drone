using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CageTriggerDetection : MonoBehaviour
{

    public List<Rigidbody> rigidbodiesToCheck;

    // Start is called before the first frame update
    void Start()
    {
        // Initialize the list
        rigidbodiesToCheck = new List<Rigidbody>();

        // Add the Platform's Rigidbody
        //rigidbodiesToCheck.Add(GameObject.Find("Platform").GetComponent<Rigidbody>());

        // Add each Wall's Rigidbody
        foreach (Transform child in transform)
        {
            if (child.CompareTag("Wall"))
            {
                rigidbodiesToCheck.Add(child.GetComponent<Rigidbody>());
            }
        }
    }
    public bool IsDronePresent()
    {
        foreach (Rigidbody rb in rigidbodiesToCheck)
        {
            if (rb.GetComponent<Collider>().bounds.Contains(GameObject.FindGameObjectWithTag("Drone").transform.position))
            {
                return true;
            }
        }
        return false;
    }
}
