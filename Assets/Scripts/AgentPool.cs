using System.Collections;
using System.Collections.Generic;
using System.Diagnostics.Tracing;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.SideChannels;
using UnityEngine;
using UnityEngine.Pool;

public class AgentPool : MonoBehaviour
{
    [SerializeField] private FullTrainingEnviroment trainingEnviromentPrefab;
    private List<GameObject> agents = new List<GameObject>();
    AgentSideChannel agentSideChannel;
    public string targetTag = "TrainingField";

    private IObjectPool<FullTrainingEnviroment> objectPool;

    // throw an exception if we try to return an existing item, already in the pool
    [SerializeField] private bool collectionCheck = true;

    // extra options to control the pool capacity and maximum size
    private int defaultCapacity = 20;
    private int maxSize = 200;

    public void Awake()
    {
        objectPool = new ObjectPool<FullTrainingEnviroment>(CreateProjectile,
        OnGetFromPool, OnReleaseToPool, OnDestroyPooledObject,
        collectionCheck, defaultCapacity, maxSize);

        agentSideChannel = new AgentSideChannel();
        Application.logMessageReceived += agentSideChannel.SendDebugStatementToPython;
        agentSideChannel.OnCustomMessageReceived += AdjustPopulationSize;
        SideChannelManager.RegisterSideChannel(agentSideChannel);
    }

    public void AdjustPopulationSize(int targetPopulationSize)
    {
        Logger.LogMessage($"Adjusting population size to {targetPopulationSize}.", true);

        int difference = 200 - objectPool.CountInactive - targetPopulationSize;
        if (difference < 0)
        {
            // Too many active agents, return some to the pool
            for (int i = 0; i < -difference; i++)
            {
                FullTrainingEnviroment trainingEnviroment = objectPool.Get();
                if (trainingEnviroment == null)
                    return;

            }
        }
        else if (difference > 0)
        {
            // Not enough active agents, get some from the pool
            for (int i = 0; i < difference; i++)
            {
                FullTrainingEnviroment trainingEnviroment = objectPool.Get();
                if (trainingEnviroment == null)
                    return;
                trainingEnviroment.Deactivate();
            }
        }
    }


    void OnDestroy()
    {

        Logger.LogMessage("Destroyed...");

        agentSideChannel.OnCustomMessageReceived -= AdjustPopulationSize;

        SideChannelManager.UnregisterSideChannel(agentSideChannel);
        Application.logMessageReceived -= agentSideChannel.SendDebugStatementToPython;
    }

    #region Pooling functions
    // invoked when creating an item to populate the object pool
    private FullTrainingEnviroment CreateProjectile()
    {
        FullTrainingEnviroment projectileInstance = Instantiate(trainingEnviromentPrefab);
        projectileInstance.ObjectPool = objectPool;
        return projectileInstance;
    }

    // invoked when returning an item to the object pool
    private void OnReleaseToPool(FullTrainingEnviroment pooledObject)
    {
        pooledObject.gameObject.SetActive(false);
    }

    // invoked when retrieving the next item from the object pool
    private void OnGetFromPool(FullTrainingEnviroment pooledObject)
    {
        pooledObject.gameObject.SetActive(true);
    }

    // invoked when we exceed the maximum number of pooled items (i.e. destroy the pooled object)
    private void OnDestroyPooledObject(FullTrainingEnviroment pooledObject)
    {
        Destroy(pooledObject.gameObject);
    }

    #endregion

}