using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

public class Goal : MonoBehaviour
{

    public Action ReachedGoalEvent;
    public Action GoalTouchedEvent;

    public Material goalReached;
    public Material goalUnreached;
    public Material goalStayed;

    private MeshRenderer goalMesh;
    private Transform parentTransform;
    private Vector3 parentSize;

    private float triggerEnterTime;
    private bool isInTrigger = false;

    [SerializeField] private float xOffset;
    [SerializeField] private float yOffset;
    [SerializeField] private float zOffset;
    //[SerializeField] 
    private string collisionTag = "DronePart";
    [SerializeField] private bool randomSpawn = false;

    [Header("Settings for agent reward system")]
    [SerializeField] private float requiredSeconds;

    public Vector3 worldVelocity = Vector3.zero;
    public Vector3 worldLookDirection => Vector3.ProjectOnPlane(
    -transform.position, Vector3.up).normalized;

    private GameObject parentObject;

    private void Awake()
    {
        goalMesh = GetComponent<MeshRenderer>();
        parentTransform = transform.parent;
        parentObject = transform.parent.gameObject;
/*        if (parentTransform != null)
        {
            //parentSize = GetObjectSize(parentTransform);
            parentSize = GetParentSize(transform.parent.gameObject);
            SpawnObjectInSphere();
        }
        else
        {
            Debug.LogError("Parent transform not found!");  
        }*/
    }

    private void FixedUpdate()
    {
        if (isInTrigger)
        {
            if (Time.time - triggerEnterTime >= requiredSeconds / 2)
            {
                goalMesh.material = goalStayed;
            }
            if (Time.time - triggerEnterTime >= requiredSeconds)
            {
                ReachedGoalEvent.Invoke();
            }
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
        Logger.LogMessage("Triggered by tag: " + collisionObject.tag);
        if (collisionObject.tag == collisionTag)
        {
            //GoalTouchedEvent?.Invoke();
            Logger.LogMessage("Triggered!" + collisionObject.tag);
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
            CancelInvoke();
            goalMesh.material = goalUnreached;
            isInTrigger = false;
        }
    }

    public void ResetGoal()
    {
        if (goalMesh!= null)
        {
            goalMesh.material = goalUnreached;
            isInTrigger = false;
        }
    }

    public Vector3 GetParentSize(GameObject parentObject)
    {
        Renderer parentRenderer = parentObject.GetComponent<Renderer>();
        Collider parentCollider = parentObject.GetComponent<Collider>();

        if (parentRenderer != null)
        {
            return parentRenderer.bounds.size;
        }
        else if (parentCollider != null)
        {
            return parentCollider.bounds.size;
        }
        else
        {
            // Handle the case where the parent does not have a Renderer or Collider
            Debug.LogWarning("Parent object does not have a Renderer or Collider to determine size.");
            return Vector3.one; // Default size, you may handle this case differently.
        }
    }

    #region (Crosses) Spehere spawning

    public float GetSphereDiameter(GameObject sphereObject)
    {
        MeshFilter meshFilter = sphereObject.GetComponent<MeshFilter>();
        if (meshFilter != null)
        {
            Vector3 boundsSize = meshFilter.mesh.bounds.size;
            // Assuming the object is a perfect sphere, so we can take any dimension
            return boundsSize.x * sphereObject.transform.localScale.x; // Scale factor considered
        }
        else
        {
            Debug.LogWarning("Object does not have a MeshFilter.");
            return 0f;
        }
    }

    public void SpawnObjectInSphere()
    {
        if (randomSpawn)
        {
            float diameter = GetSphereDiameter(parentObject);
            Debug.Log("Diameter " + diameter);
            float radius = diameter / 2;
            Vector3 center = parentObject.transform.position;

            Vector3 randomPointInsideSphere = Random.insideUnitSphere * radius;
            Vector3 spawnPosition = center + randomPointInsideSphere;

            Debug.Log("Spawning position: " + spawnPosition);
            transform.position = spawnPosition;
        }
    }

    #endregion
}
