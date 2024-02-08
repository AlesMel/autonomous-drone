using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents.SideChannels;
using UnityEngine;

public class SideChannelInitiator : MonoBehaviour
{
    AgentSideChannel sideChannel;
    // Start is called before the first frame update
    void Start()
    {
        sideChannel = new AgentSideChannel();
        Application.logMessageReceived += sideChannel.SendDebugStatementToPython;
        SideChannelManager.RegisterSideChannel(sideChannel);

    }

    // Update is called once per frame
    void OnDestroy()
    {
        Application.logMessageReceived -= sideChannel.SendDebugStatementToPython;

        SideChannelManager.UnregisterSideChannel(sideChannel);
    }
}
