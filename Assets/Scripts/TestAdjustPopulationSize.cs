using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestAdjustPopulationSize : MonoBehaviour
{

    private AgentManager agentManager;
    [SerializeField] int numberOfAgents = 100;

    // Start is called before the first frame update
    void Start()
    {
        agentManager = GetComponent<AgentManager>();
    }

    // Update is called once per frame
    void Update()
    {
        agentManager.AdjustPopulationSize(numberOfAgents);
    }
}
