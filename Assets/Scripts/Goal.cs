using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Goal : MonoBehaviour
{
    public Material goalReached;
    public Material goalUnreached;
    private MeshRenderer goalMesh;

    private void Start()
    {
        goalMesh = GetComponent<MeshRenderer>();
    }

    private void OnTriggerEnter(Collider other)
    {
        GameObject collisionObject = other.gameObject;
        Debug.Log("Triggered!");
        if (collisionObject.tag == "Drone")
        {
            goalMesh.material = goalReached;
        }
    }

    private void OnTriggerExit(Collider other)
    {
        goalMesh.material = goalUnreached;
    }

}
