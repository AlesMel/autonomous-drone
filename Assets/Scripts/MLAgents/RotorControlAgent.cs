using DroneProject;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Numerics;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.Sentis.Layers;
using UnityEngine;
using UnityEngine.InputSystem;
using Vector3 = UnityEngine.Vector3;

public class RotorControlAgent : BaseAgent
{
    [HideInInspector] public GoalGenerator goalGenerator;

    private Vector3[] velocityBuffer;
    private int velocityBufferIndex;
    private const int velocityBufferSize = 10;
    private float velocityError;

    protected override void Awake()
    {
        base.Awake();
        goalGenerator = GetComponent<GoalGenerator>();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        base.CollectObservations(sensor);
        SetDroneTarget(goalGenerator.worldVelocity, goalGenerator.worldLookDirection);

        // Length of 3+1 = 4 + 9 (base) = 13 observations
        // Debug.Log($"{StepCount} + tlv: {targetLocalVelocity} + tda: {targetDirectionAngle}");
        sensor.AddObservation(targetLocalVelocity);
        sensor.AddObservation(targetDirectionAngle);
    }

    public override void Initialize()
    {
        base.Initialize();
        velocityBuffer = new Vector3[velocityBufferSize];
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // rigidBody issue
        if (drone.m_ResetFlag)
        {
            drone.m_ResetFlag = false;
            return;
        }
        base.OnActionReceived(actionBuffers);
        goalGenerator.ManagedUpdate(Time.fixedDeltaTime);

        AddRewards();
    }

    public override void OnEpisodeBegin()
    {   
        base.OnEpisodeBegin();
        goalGenerator.ResetGoal();
        Array.Clear(velocityBuffer, 0, velocityBufferSize);
        velocityBufferIndex = 0;
    }

    public override void AddRewards()
    {
        base.AddRewards();
        velocityBuffer[velocityBufferIndex] = goalGenerator.worldVelocity;
        // Circular buffer
        velocityBufferIndex++;
        velocityBufferIndex %= velocityBufferSize;

        float velocityErrorMSQ = Mathf.Infinity;
        Vector3 velocity = drone.worldVelocity;

        for (int i = 0; i < velocityBufferSize; i++)
        {
            velocityErrorMSQ = Mathf.Min(velocityErrorMSQ, (velocityBuffer[i] - velocity).sqrMagnitude);
        }

        velocityError = Mathf.Sqrt(velocityErrorMSQ);
        //Debug.Log("ve: " + velocityError);
        float velocityReward = HelperFunctions.Reward(velocityError, 1.0f);

        float orientationError = Mathf.Abs(localLookAngle);
        float orientationReward = HelperFunctions.Reward(orientationError, 2.0f);

        float stabilityError = drone.worldAngularVelocity.magnitude;
        float stabilityReward = HelperFunctions.Reward(stabilityError, 2.0f);

        AddReward(velocityReward * stabilityReward * orientationReward);
    }


    private void OnDrawGizmosSelected()
    {
        if (drone != null)
        {
            Vector3 position = drone.transform.position;
            Gizmos.color = Color.green;
            Gizmos.DrawRay(position, goalGenerator.worldVelocity);
        }
    }
}
