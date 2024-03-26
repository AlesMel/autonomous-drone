using DroneProject;
using System.Collections;
using System.Collections.Generic;
using System.Numerics;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.Sentis.Layers;
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
        decisionInterval = GetComponent<DecisionRequester>().DecisionPeriod;
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        if (drone.m_ResetFlag)
        {
            drone.m_ResetFlag = false;
            //Debug.Log($"Skipped with {transform.name} tf: {transform.position.x:F4}, {transform.position.y:F4}, {transform.position.z:F4}, ang vel: {drone.droneRigidBody.angularVelocity.x:F4}, {drone.droneRigidBody.angularVelocity.y:F4}, {drone.droneRigidBody.angularVelocity.z:F4}");
            return;
        }
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
/*        if (bounds.Contains(drone.droneRigidBody.position))
        {*/

            float positionError = VectorToNextCheckpoint().magnitude; // 1 - / thresholdDistance;
            float positionReward = HelperFunctions.Reward(positionError, 2f);
            float rotationError =  AngleToGoal();
            float rotationReward = HelperFunctions.Reward(rotationError, 10f);
            //float stabilityError = HelperFunctions.Reward(drone.worldAngularVelocity.magnitude / drone.maxAngularVelocity, 10);
            float stabilityError = drone.worldAngularVelocity.magnitude;
            float stabilityReward = HelperFunctions.Reward(stabilityError, 1f);
            float velocity = HelperFunctions.Reward(drone.localVelocity.magnitude, 1f);
            float straightPositionReward = 1 - drone.transform.up.y;
        // Debug.Log(rotationError);
        /*  
          AddReward(straightPositionReward * -0.5f);
           // + rotationErrors*/
            /*AddReward(-0.5f * rotationError);
            AddReward(-0.01f * stabilityError);
            AddReward(1f * positionReward);*/
            AddReward(positionReward);
/*        } 
        else
        {
            EndEpisode();
        }*/
    }
}
