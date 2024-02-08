using System.Collections;
using System.Collections.Generic;
using System.Diagnostics.Tracing;
using Unity.MLAgents;
using Unity.MLAgents.SideChannels;
using UnityEngine;

public class AgentManager : MonoBehaviour
{
    private BaseAgent[] agents;

    public void Start()
    {
        // Find all agents in the scene
        agents = FindObjectsOfType<BaseAgent>(true);
        // Optionally, do something with each agent
        foreach (BaseAgent agent in agents)
        {
            Logger.LogMessage("Found agent: " + agent.gameObject.name);
        }

    }

    public void AdjustPopulationSize(int targetPopulationSize)
    {
        Debug.LogError($"Total agents in playground {agents.Length}");

        int currentActiveCount = 0;

        foreach (BaseAgent agent in agents)
        {
            if (agent.gameObject.activeSelf)
            {
                currentActiveCount++;
            }
        }
        Debug.LogError($"Current active agents {currentActiveCount}");

        // Adjust the number of active agents to match the target population size
        if (targetPopulationSize < currentActiveCount)
        {
            // Too many agents: disable some
            foreach (BaseAgent agent in agents)
            {
                if (targetPopulationSize <= 0) break; // No more agents need to be active
                if (agent.gameObject.activeSelf)
                {
                    targetPopulationSize--;
                }
                else
                {
                    agent.gameObject.SetActive(false);
                }
            }
        }
        else if (targetPopulationSize > currentActiveCount)
        {
            // Too few agents: enable some
            foreach (BaseAgent agent in agents)
            {
                Debug.LogError($"{agent.name} is getting checked: {agent.gameObject.activeSelf}");
                if (currentActiveCount >= targetPopulationSize)
                {
                    Debug.LogError($"currentActive: {currentActiveCount}, target: {targetPopulationSize}");
                    break; // Reached target number of active agents
                }
                if (!agent.gameObject.activeSelf)
                {
                    Debug.LogError($"Activated: {agent.transform.parent.name}");
                    agent.gameObject.SetActive(true);
                    currentActiveCount++;
                }
            }
        }
        Debug.LogError($"Adjusted population size to {targetPopulationSize}");
    }

}