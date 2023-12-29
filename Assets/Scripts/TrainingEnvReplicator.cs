using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TrainingEnvReplicator : MonoBehaviour
{

    [SerializeField] private GameObject environment;
    [SerializeField] private int numberOfEnvironments;
    [SerializeField] private float separation = 10.0f;
    private string trainingAreaName;

    private void Awake()
    {
        trainingAreaName = environment.name;

    }

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {

    }

    private void ReplicateEnvironment()
    {
        for (int i = 0; i < numberOfEnvironments; i++)
        {
            for (int j = 0; j < numberOfEnvironments; j++)
            {

            }
        }
    }
}
