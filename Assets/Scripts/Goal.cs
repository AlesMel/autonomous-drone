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

    [SerializeField] private float xOffset;
    [SerializeField] private float yOffset;
    [SerializeField] private float zOffset;
    //[SerializeField] 
    private string collisionTag = "DronePart";
    private bool randomSpawn = true;

    [Header("Settings for agent reward system")]
    [SerializeField] private float requiredSeconds;

    public Vector3 worldVelocity = Vector3.zero;
    public Vector3 worldLookDirection => Vector3.ProjectOnPlane(
    -transform.position, Vector3.up).normalized;

    private GameObject parentObject;
    protected Bounds bounds;

    private void Awake()
    {
        Initialize();
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
        bounds = new Bounds(transform.position, Vector3.one * 10);

    }

    public void Initialize()
    {
        goalMesh = GetComponent<MeshRenderer>();
        parentTransform = transform.parent;
        parentObject = transform.parent.gameObject;
    }

    private void FixedUpdate()
    {
/*        if (isInTrigger)
        {
            if (Time.time - triggerEnterTime >= requiredSeconds / 2)
            {
                goalMesh.material = goalStayed;
            }
            if (Time.time - triggerEnterTime >= requiredSeconds)
            {
                ReachedGoalEvent.Invoke();
            }
        }*/
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

    public void SpawnObject()
    {
        if (randomSpawn)
        {
            // Calculate the spawn positions based on the bounds argument
            float spawnX = Random.Range(bounds.center.x - bounds.extents.x + xOffset, bounds.center.x + bounds.extents.x - xOffset);
            float spawnY = Random.Range( yOffset, bounds.center.y + bounds.extents.y - yOffset);
            float spawnZ = Random.Range(bounds.center.z - bounds.extents.z + zOffset, bounds.center.z + bounds.extents.z - zOffset);

            // Construct the spawn position vector
            Vector3 spawnPosition = new Vector3(spawnX, spawnY, spawnZ);
            // Set the transform's position to the calculated spawn position
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
        }
    }

    private void OnTriggerExit(Collider other)
    {
        GameObject collisionObject = other.gameObject;
        if (collisionObject.tag == collisionTag)
        {
            CancelInvoke();
            goalMesh.material = goalUnreached;
        }
    }

    public void ResetGoal()
    {
        if (goalMesh!= null)
        {
            goalMesh.material = goalUnreached;
        }
        // SpawnObject();
    }


    #region (Crosses) Spehere spawning
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
