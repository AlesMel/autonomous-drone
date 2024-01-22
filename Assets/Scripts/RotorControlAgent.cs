using DroneProject;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class RotorControlAgent : BaseAgent
{
    public override void CollectObservations(VectorSensor sensor)
    {
        base.CollectObservations(sensor);
    }

    public override void Initialize()
    {
        base.Initialize();
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        base.OnActionReceived(actionBuffers);
        AddRewards();
    }

    public override void OnEpisodeBegin()
    {
        base.OnEpisodeBegin();
    }

    public override void AddRewards()
    {
        base.AddRewards();
        float stabilityError = drone.worldAngularVelocity.magnitude;
        float stabilityReward = HelperFunctions.Reward(stabilityError, 2);

        AddReward(stabilityReward);

    }
}
