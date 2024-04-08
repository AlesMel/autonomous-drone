using DroneProject;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Numerics;
using TMPro;
using Unity.Mathematics;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.Sentis.Layers;
using UnityEngine;
using Vector3 = UnityEngine.Vector3;

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

        if (!bounds.Contains(drone.transform.position))
        {
            AddReward(-leftEnviromentPenalty);
            EndCurrentEpisode("left");
        }

        AddRewards();
    }

    public override void OnEpisodeBegin()
    {   
        base.OnEpisodeBegin();  
    }
    private float GetVelocityReward(float rewardConstant)
    {
        float currentVelocity = drone.localVelocity.magnitude;
        float normalizedVelocity = currentVelocity / drone.droneRigidBody.maxLinearVelocity;
        float stabilityError = currentVelocity;
        float stabilityReward = HelperFunctions.Reward(stabilityError, rewardConstant);
        return stabilityReward;
    }

    private float GetAngularReward(float rewardConstant)
    {
        float currentAngularVelocity = drone.worldAngularVelocity.magnitude;
        float normalizedAngularVelocity = currentAngularVelocity / drone.droneRigidBody.maxAngularVelocity;
        float stabilityError = currentAngularVelocity;
        float stabilityReward = HelperFunctions.Reward(stabilityError, rewardConstant);
        return stabilityReward;
    }

    public override void AddRewards()
    {
        base.AddRewards();

        // Add small reward for staying alive
        // AddReward(+1.0f / MaxStep);
                
        float currentDistance = VectorToNextCheckpoint().magnitude;
        float currentAngularVelocity = drone.worldAngularVelocity.magnitude;
        float normalizedDistance = currentDistance / maxCheckpointDistance;
        float normalizedAngularVelocity = currentAngularVelocity / drone.droneRigidBody.maxAngularVelocity;
        float normalizedAngleError = Mathf.Abs(AngleToGoal());

        /*        if (bounds.Contains(drone.droneRigidBody.position))
                {*/
        var totalReward = 0.0f;

        float positionError = currentDistance; // 1 - ;
        float positionReward = HelperFunctions.Reward(positionError, 5f);
        float rotationError = normalizedAngleError;
        float rotationReward = HelperFunctions.Reward(rotationError, 10f);
        float stabilityError = currentAngularVelocity;
        float stabilityReward = HelperFunctions.Reward(stabilityError, 1f);
        totalReward = stabilityReward * positionReward;
        //Debug.Log($"stabilityReward: {stabilityReward}, rotationReward: {rotationReward}, positionReward: {positionReward}");
        // Debug.Log(-2.0f * stabilityError / drone.droneRigidBody.maxAngularVelocity);
        // totalReward = positionReward * GetAngularReward(15f);
        // totalReward = (1.0f / (currentDistance + 1.0f)) - 0.4f * currentAngularVelocity;
        // totalReward = (1.0f / normalizedDistance) * Mathf.Cos(rotationError) * Mathf.Exp(-normalizedAngularVelocity / 0.1f);

        // Debug.Log($"{(1.0f / (currentDistance + 1.0f))} and {normalizedAngularVelocity}");

        /*        totalReward += (1 - normalizedDistance);
                totalReward += 0.8f * (-normalizedAngularVelocity);*/


        //Debug.Log(totalReward);
        /*        float distanceGain = previousDistance - currentDistance;
                totalReward += distanceGain;
                totalReward -= 0.2f * normalizedAngularVelocity;
                previousDistance = currentDistance;*/
        // Debug.Log(totalReward);

        // totalReward += -1f * rotationError;


        AddReward(totalReward);
    }

    private void OnTriggerStay(Collider other)
    {
        if (other.CompareTag("Goal"))
        {
            Goal goal = other.GetComponent<Goal>(); // Assuming Goal is the script attached to your goal objects
            if (goal != null && goals[currentCheckpointIndex] == goal)
            {
                // AddReward(stayGoalReward);
            }
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Goal") && !reachedGoal)
        {
            Goal goal = other.GetComponent<Goal>(); // Assuming Goal is the script attached to your goal objects
            if (goal != null && goals[currentCheckpointIndex] == goal)
            {
                // Debug.Log($"Reached1 {VectorToNextCheckpoint()}");
                // currentCheckpointIndex += 1;
                //AddReward(touchedGoalReward);
                reachedGoal = true;
            }
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Ground"))
        {
            // AddReward(-collisionPenalty);
            // EndCurrentEpisode("Crashed!");
            //isColliding = true;

        }
    }

}
