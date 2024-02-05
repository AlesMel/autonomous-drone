using DroneProject;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class RotorControlAgent : BaseAgent
{
    [SerializeField]
    public float diameter = 101.0f;

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
        /*        // Dot product between velocity and goal position
                var velocityDotGoal = Vector3.Dot(drone.localVelocity, VectorToNextCheckpoint());
                // Calculate the reward based on alignment with goal direction
                var alignmentReward = velocityDotGoal * (0.15f / MaxStep);
                // Calculate the reward based on proximity to the goal
                var distanceReward = Mathf.Clamp01(1f - Vector3.Distance(goal.transform.position, drone.worldPosition) / thresholdDistance);
                distanceReward *= 0.1f / MaxStep; // Adjust the reward factor as needed

                // Combine alignment, distance, velocity, and direction rewards
                var totalReward = alignmentReward + distanceReward;

                // Calculate the dot product between the agent's forward direction and the direction to the checkpoint
                float dotProduct = Vector3.Dot(Vector3.forward, VectorToNextCheckpoint());
                if (dotProduct > 0.91f && velocityDotGoal > 1.5f)
                {
                    totalReward += (1.0f / MaxStep);
                }
                else
                {
                    totalReward -= (1.0f / MaxStep);
                }
        */
        // Dot product between velocity and goal position
        var velocityDotGoal = Vector3.Dot(drone.localVelocity, VectorToNextCheckpoint());
        // Calculate the reward based on alignment with goal direction
        var alignmentReward = velocityDotGoal * (0.15f / MaxStep);
        var distanceReward = (thresholdDistance - VectorToNextCheckpoint().magnitude);
        var angularPenalty = drone.localAngularVelocity.magnitude * (-1);
        AddReward(alignmentReward * 0.0f + distanceReward + angularPenalty * 0);
    }
}
