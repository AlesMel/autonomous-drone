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
    public string targetTag = "TrainingField";

    public void Awake()
    {
        agentSideChannel = new AgentSideChannel();
        Application.logMessageReceived += agentSideChannel.SendDebugStatementToPython;
        agentSideChannel.OnCustomMessageReceived += AdjustPopulationSize;
        SideChannelManager.RegisterSideChannel(agentSideChannel);
        Initialize();
    }

    public void Initialize()
    {
        // Find and add all child GameObjects with the specified tag
        AddAllChildrenWithTag(transform);
        Logger.LogMessage($"Total agents in playground: {agents.Count}", true);
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

        Logger.LogMessage($"Adjusting population size to {targetPopulationSize}.", true);

        // Refresh the lists to ensure they're up to date with the current state
        var activeAgents = agents.Where(agent => agent.gameObject.activeSelf).ToList();
        var inactiveAgents = agents.Where(agent => !agent.gameObject.activeSelf).ToList();

        int currentActiveCount = activeAgents.Count;
        int difference = targetPopulationSize - currentActiveCount;
        // Debug.LogWarning($"Difference is {difference}.");

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


    /*public void AdjustPopulationSize(int popSize)
    {
        int targetPopulationSize = popSize;

        Debug.LogError($"Total agents in playground {agents.Length}");

        List<BaseAgent> activeAgents = GetActiveAgents();
        int currentActiveCount = activeAgents.Count;

        Debug.LogError($"Current active agents {currentActiveCount}");
        Debug.LogError($"Adjusing population size to {popSize}");

        // Adjust the number of active agents to match the target population size
        if (targetPopulationSize < currentActiveCount)
        {
            while (targetPopulationSize != currentActiveCount)
            {
                activeAgents[currentActiveCount - 1].gameObject.SetActive(false);
                currentActiveCount--;
            }
        }
        else if (targetPopulationSize > currentActiveCount)
        {
            while (targetPopulationSize != currentActiveCount)
            {
                agents[currentActiveCount].gameObject.SetActive(true);
                currentActiveCount++;
                Debug.Log($"{agents[currentActiveCount].name} got activated");
            }
        }
    }*/

    void OnDestroy()
    {

        Logger.LogMessage("Destroyed...");

        agentSideChannel.OnCustomMessageReceived -= AdjustPopulationSize;

        SideChannelManager.UnregisterSideChannel(agentSideChannel);
        Application.logMessageReceived -= agentSideChannel.SendDebugStatementToPython;
    }

}