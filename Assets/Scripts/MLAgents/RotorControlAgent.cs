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
        goalGenerator = GetComponentInParent<GoalGenerator>();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        base.CollectObservations(sensor);
        SetDroneTarget(goalGenerator.worldVelocity, goalGenerator.worldLookDirection);

        // Length of 3+1 = 4 + 9 (base) = 13 observations
        // Debug.Log(targetDirectionAngle + " " + targetLocalVelocity);
        sensor.AddObservation(targetDirectionAngle);
        sensor.AddObservation(HelperFunctions.Sigmoid(targetLocalVelocity, 0.5f));
            
        DefaultPhysicsObservations(sensor);

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

        AddRewards();
    }

    public override void OnEpisodeBegin()
    {   
        base.OnEpisodeBegin();
        Array.Clear(velocityBuffer, 0, velocityBufferSize);
        velocityBufferIndex = 0;
    }

    public override void AddRewards()
    {

        base.AddRewards();
        if (bounds.Contains(drone.droneRigidBody.position))
        {
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
            float velocityCoefficient = goalGenerator.isMoving ? 1 : 10;
            float velocityReward = HelperFunctions.Reward(velocityError, velocityCoefficient);

            float orientationError = Mathf.Abs(localLookAngle);
            float orientationReward = HelperFunctions.Reward(orientationError, 10.0f);

            float stabilityError = drone.worldAngularVelocity.magnitude;
            float stabilityReward = HelperFunctions.Reward(stabilityError, 2.0f);

            AddReward(velocityReward * stabilityReward * orientationReward);
        } 
        else
        {
            EndEpisode();
        }
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
