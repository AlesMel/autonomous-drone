using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Goal : MonoBehaviour
{
    public Material goalReached;
    public Material goalUnreached;

    private MeshRenderer goalMesh;
    private Transform parentTransform;
    private Vector3 parentSize;

    private float triggerEnterTime;
    private bool isInTrigger = false;
    // Result for check if drone was in the position for required time 
    public bool shouldContinue = false;

    [SerializeField] private float xOffset;
    [SerializeField] private float yOffset;
    [SerializeField] private float zOffset;
    [SerializeField] private string collisionTag;
    [SerializeField] private bool randomSpawn = false;

    [Header("Settings for agent reward system")]
    [SerializeField] private float requiredSeconds;
    private void Start()
    {
        goalMesh = GetComponent<MeshRenderer>();
        parentTransform = transform.parent;
        if (parentTransform != null)
        {
            parentSize = GetObjectSize(parentTransform);
            SpawnObject();
        }
        else
        {
            Debug.LogError("Parent transform not found!");  
        }
    }

    private void FixedUpdate()
    {
        if (isInTrigger && Time.time - triggerEnterTime >= requiredSeconds)
        {
            shouldContinue = true;
            SpawnObject();
        }
    }

    private Vector3 GetObjectSize(Transform transform)
    {
        Collider cageCollider = transform.GetComponent<BoxCollider>();

        if (cageCollider != null)
        {
            return cageCollider.bounds.size;
        } else
        {
            return Vector3.zero;
        }
    }

    private void ResetPositionControlParameters()
    {
        shouldContinue= false;
        isInTrigger= false;
    }

    public void SpawnObject()
    {
        if (randomSpawn)
        {
            ResetPositionControlParameters();
            Debug.Log("Parent size: " + parentSize.ToString());
            float spawnX = Random.Range(-parentSize.x / 2 + xOffset, parentSize.x / 2 - xOffset);
            float spawnY = Random.Range(0 + yOffset, parentSize.y / 2 - yOffset);
            float spawnZ = Random.Range(-parentSize.z / 2 + zOffset, parentSize.z / 2 - zOffset);
            Vector3 spawnPosition = new Vector3(spawnX, spawnY, spawnZ);
            Debug.Log("Spawning position " + spawnPosition.ToString());
            transform.position = spawnPosition;
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        GameObject collisionObject = other.gameObject;

        Debug.Log("Triggered!");
        if (collisionObject.tag == collisionTag)
        {
            goalMesh.material = goalReached;
            triggerEnterTime = Time.time;
            isInTrigger = true;
        }
    }

    private void OnTriggerExit(Collider other)
    {
        GameObject collisionObject = other.gameObject;
        if (collisionObject.tag == collisionTag)
        {
            goalMesh.material = goalUnreached;
            isInTrigger = false;
        }
    }

}
