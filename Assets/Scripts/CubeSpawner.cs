using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CubeSpawner : MonoBehaviour
{
    [Header("Falling object prefab")]
    public GameObject cubePrefab; // Assign the cube prefab in the inspector
    public Vector3 spawnPosition = new Vector3(0, 10, 0); // Position above which cubes will spawn
    public float minForce = 5f;
    public float maxForce = 20f;
    public float minInterval = 1f;
    public float maxInterval = 3f;
    public float despawnHeight = -5f; // Y position below which cubes will be destroyed

    private List<GameObject> spawnedCubes = new List<GameObject>();

    private void Start()
    {
        StartCoroutine(SpawnAndManageCubes());
    }

    private IEnumerator SpawnAndManageCubes()
    {
        while (true) // Infinite loop to keep spawning and managing cubes
        {
            yield return new WaitForSeconds(Random.Range(minInterval, maxInterval)); // Wait for a random interval

            GameObject cube = Instantiate(cubePrefab, spawnPosition, Quaternion.identity); // Instantiate the cube
            Rigidbody rb = cube.GetComponent<Rigidbody>();

            if (rb != null)
            {
                float forceMagnitude = Random.Range(minForce, maxForce);
                Vector3 forceDirection = Vector3.up; // Modify as needed
                rb.AddForce(forceDirection * forceMagnitude, ForceMode.Impulse);
            }

            spawnedCubes.Add(cube); // Add the cube to the list for tracking

            // Cleanup cubes that are below the despawn height
            spawnedCubes.RemoveAll(cube => cube == null); // Remove null references from the list
            for (int i = spawnedCubes.Count - 1; i >= 0; i--)
            {
                if (spawnedCubes[i].transform.position.y < despawnHeight)
                {
                    Destroy(spawnedCubes[i]);
                    spawnedCubes.RemoveAt(i);
                }
            }
        }
    }
}
