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
    public List<Vector3> spawnPoints; // List of predefined spawn points

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
        bounds = new Bounds(transform.position, Vector3.one * 5);

    }

    public void Initialize()
    {
        goalMesh = GetComponent<MeshRenderer>();
        parentTransform = transform.parent;
        parentObject = transform.parent.gameObject;
    }

    private void FixedUpdate()
    {

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
            Vector3 spawnPosition = spawnPoints[Random.Range(0, spawnPoints.Count)];

/*            // Define the range for spawning positions on each axis
            float xMin = bounds.center.x - bounds.extents.x + xOffset;
            float xMax = bounds.center.x + bounds.extents.x - xOffset;
            float yMin = bounds.center.y - bounds.extents.y + yOffset;
            float yMax = bounds.center.y + bounds.extents.y - yOffset;
            float zMin = bounds.center.z - bounds.extents.z + zOffset;
            float zMax = bounds.center.z + bounds.extents.z - zOffset;

            // Define the radius within which to exclude positions near the origin
            float exclusionRadius = 2.0f; // Adjust as needed

            // Ensure that spawn positions for x and y are not too close to the origin
            float spawnX = Random.Range(xMin, xMax);
            float spawnY = Random.Range(yMin, yMax);

            // Check if the spawn positions are within the exclusion radius
            while (Mathf.Abs(spawnX) < exclusionRadius && Mathf.Abs(spawnY) < exclusionRadius)
            {
                // Reselect spawn positions until they are outside the exclusion radius
                spawnX = Random.Range(xMin, xMax);
                spawnY = Random.Range(yMin, yMax);
            }

            // Randomly select spawn positions within the defined range for the z axis
            float spawnZ = Random.Range(zMin, zMax);


            // Construct the spawn position vector
            Vector3 spawnPosition = new Vector3(spawnX, spawnY, spawnZ);
            // Set the transform's position to the calculated spawn position*/
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
        SpawnObject();
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
