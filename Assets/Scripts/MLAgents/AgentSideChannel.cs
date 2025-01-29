using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.SideChannels;
using System.Text;
using System;

public class AgentSideChannel : SideChannel
{
    public delegate void MessageReceivedEventHandler(int populationSize);
    public event MessageReceivedEventHandler OnCustomMessageReceived;

    public AgentSideChannel()
    {
        ChannelId = new Guid("58caf6d4-08f6-4996-8aba-72ed74670acd");
        Debug.LogError("Side channel Initialized!");
    }

    protected override void OnMessageReceived(IncomingMessage msg)
    {
        var populationSize = msg.ReadInt32();
        // Debug.LogError("From Python : " + populationSize);
        OnCustomMessageReceived?.Invoke(populationSize);
    }

    public void SendBoolToPython(bool value)
    {
        using (var msgOut = new OutgoingMessage())
        {
            msgOut.WriteBoolean(value);
            QueueMessageToSend(msgOut);
        }
    }
    public void SendDebugStatementToPython(string logString, string stackTrace, LogType type)
    {
       /* if (type != LogType.Error)
        {
            var stringToSend = type.ToString() + ": " + logString + "\n" + stackTrace;
            using (var msgOut = new OutgoingMessage())
            {
                msgOut.WriteString(stringToSend);
                QueueMessageToSend(msgOut);
            }
        }*/
    }
}