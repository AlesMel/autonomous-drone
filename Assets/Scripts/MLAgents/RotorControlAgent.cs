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

        // Dot product between velocity and goal position
        var velocityDotGoal = Vector3.Dot(drone.localVelocity, VectorToNextCheckpoint());
        // Calculate the reward based on alignment with goal direction
        var alignmentReward = velocityDotGoal;
        var distanceReward = (thresholdDistance - VectorToNextCheckpoint().magnitude);

        float stabilityError = drone.worldAngularVelocity.magnitude * (-1.0f);

        AddReward(alignmentReward * 0.1f + distanceReward + stabilityError * 1f);   
        if (drone.collisionStay)
        {
            Logger.LogMessage("Incrementing collision!", true, false);
            drone.collisionCount++;
        }

        if (drone.isInGoal)
        {
            AddReward(touchedGoalReward);
        }

        if (drone.transform.up.y < drone.tipOverThreshold)
        {
            SetReward(-tipOverPenalty);
        }

        if (drone.collisionCount >= 150) 
        {
            SetReward(-crashedPenalty);
            EndCurrentEpisode("Crashed" + drone.collisionCount);
        }

        if (!drone.isInEnviroment)
        {
            SetReward(-leftEnviromentPenalty);
            EndCurrentEpisode("Left enviroment");
        }
    }
}
