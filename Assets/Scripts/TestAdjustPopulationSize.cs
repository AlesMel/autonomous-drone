using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestAdjustPopulationSize : MonoBehaviour
{

    [SerializeField] AgentManager agentManager;
    [SerializeField] int numberOfAgents = 10;

    // Start is called before the first frame update
    void Start()
    {
        agentManager= GetComponent<AgentManager>();
        agentManager.Initialize();
    }

    // Update is called once per frame
    void Update()
    {
        agentManager.AdjustPopulationSize(numberOfAgents);
    }
}
