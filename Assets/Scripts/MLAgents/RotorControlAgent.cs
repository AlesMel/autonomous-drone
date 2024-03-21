using DroneProject;
using System.Collections;
using System.Collections.Generic;
using System.Numerics;
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

        // Dot product between velocity and goal position
        var velocityDotGoal = UnityEngine.Vector3.Dot(drone.localVelocity, VectorToNextCheckpoint());
        // Calculate the reward based on alignment with goal direction
        var alignmentReward = velocityDotGoal;

        float rewardMultiplier = 1.0f;
        // float distanceReward = (-1.0f) * Mathf.InverseLerp(maxCheckpointDistance, 0.0f, VectorToNextCheckpoint().magnitude);
        float distancePenalty = Mathf.Pow(VectorToNextCheckpoint().magnitude / maxCheckpointDistance, 2);  // Smoothed penalty

        //float distanceReward = Unity.Mathematics.math.exp(-VectorToNextCheckpoint().magnitude);

        // Debug.Log($"{VectorToNextCheckpoint()}");
        // float stabilityError = (-1.0f) * Mathf.InverseLerp(drone.maxAngularVelocity, 0.0f, drone.worldAngularVelocity.magnitude);

        /*if (drone.worldAngularVelocity.magnitude > 1.0f)
        {
            stabilityError = -0.1f;
        }*/
        /*float distanceError = HelperFunctions.Reward(VectorToNextCheckpoint().magnitude);
        float stabilityError =  HelperFunctions.Reward(drone.worldAngularVelocity.magnitude, 2);

        float wholeReward = distanceError * stabilityError;*/
        // AddReward(wholeReward);
        float positionError = VectorToNextCheckpoint().magnitude;
        float positionReward = HelperFunctions.Reward(positionError, 3f);
        float rotationError = AngleToGoal();
        float rotationReward = HelperFunctions.Reward(rotationError, 1f);
        float stabilityError = HelperFunctions.Reward(drone.worldAngularVelocity.magnitude, 1f);

        AddReward(positionReward);
        AddReward(isInGoal ? 0:-leftGoalPenalty );
        // Debug.Log($"{transform.name} - STEP: {StepCount} alignmentReward: {alignmentReward}, distanceReward: {distanceReward}, stabilityError: {stabilityError} tf: {transform.position}, ang vel: {drone.droneRigidBody.angularVelocity}, rew: {wholeReward}");
        // Debug.Log($"{transform.name} - STEP: {StepCount}, tf: {transform.position.x:F4}, {transform.position.y:F4}, {transform.position.z:F4}, ang vel: {drone.droneRigidBody.angularVelocity.x:F4}, {drone.droneRigidBody.angularVelocity.y:F4}, {drone.droneRigidBody.angularVelocity.z:F4}, tsholdist: {thresholdDistance}");

    }
}
