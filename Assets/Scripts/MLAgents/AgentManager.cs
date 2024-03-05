using System.Collections;
using System.Collections.Generic;
using System.Diagnostics.Tracing;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.SideChannels;
using UnityEngine;

public class AgentManager : MonoBehaviour
{
    private List<GameObject> agents = new List<GameObject>();
    AgentSideChannel agentSideChannel;
    [SerializeField] public string targetTag;
    [SerializeField] private bool enableSideChannel = true;

    public void Awake()
    {
        if (enableSideChannel)
        { 
            agentSideChannel = new AgentSideChannel();
            Application.logMessageReceived += agentSideChannel.SendDebugStatementToPython;
            agentSideChannel.OnCustomMessageReceived += AdjustPopulationSize;
            SideChannelManager.RegisterSideChannel(agentSideChannel);
        }
        Initialize();
       /* foreach(GameObject agent in agents)
        {
            agent.SetActive(true);
        }*/
        //GameObject dummyAgent = transform.Find("DummyDrone").gameObject;
       // dummyAgent.SetActive(false);
    }

    public void Initialize()
    {
        // Find and add all child GameObjects with the specified tag
        AddAllChildrenWithTag(transform);
        Logger.LogMessage($"Total agents in playground: {agents.Count}", true, true);
    }


    private void AddAllChildrenWithTag(Transform parent)
    {
        foreach (Transform child in parent)
        {
            // Check if the child GameObject's tag matches the target tag
            if (child.CompareTag(targetTag))
            {
                agents.Add(child.gameObject);
            }

            // Recursively search for more children with the tag
            AddAllChildrenWithTag(child);
        }
    }

    public void AdjustPopulationSize(int targetPopulationSize)
    {
        // agentSideChannel.SendBoolToPython(false);


        // Refresh the lists to ensure they're up to date with the current state
        var activeAgents = agents.Where(agent => agent.activeSelf).ToList();
        var inactiveAgents = agents.Where(agent => !agent.activeSelf).ToList();

        int currentActiveCount = activeAgents.Count;
        int difference = targetPopulationSize - currentActiveCount;
        // Debug.LogWarning($"Difference is {difference}.");
        Logger.LogMessage($"Adjusting population size to {targetPopulationSize} from {currentActiveCount}", true, false);

        if (difference < 0)
        {
            // Too many active agents, deactivate some
            for (int i = 0; i < -difference; i++)
            {
                if (activeAgents.Count > 0)
                {
                    // Corrected to deactivate from the end of the list for efficiency
                    int lastIndex = activeAgents.Count - 1;
                    Logger.LogMessage($"{activeAgents[lastIndex].name} got deactivated");
                    activeAgents[lastIndex].gameObject.SetActive(false);
                    activeAgents.RemoveAt(lastIndex); // Remove the now inactive agent from the active list
                }
            }
        }
        else if (difference > 0)
        {
            // Not enough active agents, activate some
            for (int i = 0; i < difference; i++)
            {
                if (inactiveAgents.Count > 0)
                {
                    // Activate from the start of the list or any specific strategy
                    inactiveAgents[0].gameObject.SetActive(true);
                    Logger.LogMessage($"{inactiveAgents[0].name} got activated");
                    inactiveAgents.RemoveAt(0); // Remove the now active agent from the inactive list
                }
            }
        }

        // agentSideChannel.SendBoolToPython(true);
    }

    void OnDestroy()
    {
        Logger.LogMessage("Destroyed...");
        if (enableSideChannel)
        { 
            agentSideChannel.OnCustomMessageReceived -= AdjustPopulationSize;
            SideChannelManager.UnregisterSideChannel(agentSideChannel);
            Application.logMessageReceived -= agentSideChannel.SendDebugStatementToPython;
        }
    }

}