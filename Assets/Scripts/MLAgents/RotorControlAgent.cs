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

        float distanceReward = (VectorToNextCheckpoint().magnitude) / maxCheckpointDistance;
        distanceReward = HelperFunctions.Reward(distanceReward);

        float stabilityError = drone.worldAngularVelocity.magnitude;
        stabilityError = HelperFunctions.Reward(stabilityError, 1);

        float wholeReward = distanceReward * stabilityError;
        AddReward(wholeReward);
        // Debug.Log($"{transform.name} - STEP: {StepCount} alignmentReward: {alignmentReward}, distanceReward: {distanceReward}, stabilityError: {stabilityError} tf: {transform.position}, ang vel: {drone.droneRigidBody.angularVelocity}, rew: {wholeReward}");
        // Debug.Log($"{transform.name} - STEP: {StepCount}, tf: {transform.position.x:F4}, {transform.position.y:F4}, {transform.position.z:F4}, ang vel: {drone.droneRigidBody.angularVelocity.x:F4}, {drone.droneRigidBody.angularVelocity.y:F4}, {drone.droneRigidBody.angularVelocity.z:F4}, tsholdist: {thresholdDistance}");

    }
}
