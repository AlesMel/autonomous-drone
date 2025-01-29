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
using Unity.MLAgentsExamples;
using Unity.Sentis.Layers;
using UnityEngine;
using static UnityEngine.GraphicsBuffer;
using Vector3 = UnityEngine.Vector3;
using Random = UnityEngine.Random;
using Quaternion = UnityEngine.Quaternion;
using UnityEngine.InputSystem;
using Unity.Sentis;
using System.Threading;

public class RotorControlAgent : BaseAgent
{
    public event Action CollisionTimeoutEvent;

    private float timeInObjective = 0.0f;
    private float totalDistance = 0.0f;
    private Vector3 previousPosition = Vector3.zero;

    DirectionIndicator m_DirectionIndicator;
    // Because dron can move erratically, this will be static
    OrientationCubeController m_OrientationCube;
    //The current target walking speed. Clamped because a value of zero will cause NaNs
    private float m_TargetWalkingSpeed = m_maxWalkingSpeed;
    const float m_maxWalkingSpeed = 15;
    public float TargetWalkingSpeed
    {
        get { return m_TargetWalkingSpeed; }
        set { m_TargetWalkingSpeed = Mathf.Clamp(value, .1f, m_maxWalkingSpeed); }
    }

    void UpdateOrientationObjects()
    {
        m_OrientationCube.UpdateOrientation(drone.transform, goals[currentCheckpointIndex].transform);
    }

    public override void Initialize()
    {
        base.Initialize();
        decisionInterval = GetComponent<DecisionRequester>().DecisionPeriod;
        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();
    }

    private void FixedUpdate()
    {
        UpdateOrientationObjects();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        base.CollectObservations(sensor);
        var cubeForward = m_OrientationCube.transform.forward;

        //velocity we want to match
        var velGoal = cubeForward * TargetWalkingSpeed;
        //ragdoll's avg vel
        var avgVel = drone.localVelocity;

        //Get velocities in the context of our orientation cube's space
        //Note: You can get these velocities in world space as well but it may not train as well.
        sensor.AddObservation(0.0f);
        sensor.AddObservation(HelperFunctions.Sigmoid(new Vector3(-0.12f, -0.42f, -0.90f), 0.5f));
        sensor.AddObservation(HelperFunctions.Sigmoid(drone.localVelocity, 0.5f));
        sensor.AddObservation(HelperFunctions.Sigmoid(drone.localAngularVelocity));
        sensor.AddObservation(drone.inclination);

        //sensor.AddObservation(VectorToNextCheckpoint() / maxCheckpointDistance);
        //sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(goals[currentCheckpointIndex].transform.position));
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

        if (transform.up.y < drone.tipOverThreshold)
        {
            AddReward(-100f);
            EndCurrentEpisode("Ttipped over");
        }

        if (!bounds.Contains(drone.transform.position))
        {
            AddReward(-100f);
            EndCurrentEpisode("left");
        }

        AddRewards();
    }

    public override void OnEpisodeBegin()
    {   
        base.OnEpisodeBegin();
        CancelInvoke();
        thresholdDistance = VectorToNextCheckpoint().magnitude;

        totalDistance = 0.0f;
        previousPosition = transform.position;
        previousDistance = thresholdDistance;
    }

    public float GetMatchingReward(float target, float actual, float sensitivity)
    {
        float a = 1; //Max Reward
        float b = target;
        float c = sensitivity;
        return a * Mathf.Exp(-(Mathf.Pow((actual - b), 2) / (2 * c * c))); //Guassian Function centered at target
    }

    public override void AddRewards()
    {
        base.AddRewards();

        Vector3 currentPosition = transform.position;
        float currentDistance = currentPosition.magnitude;

        float currentAngularVelocity = drone.worldAngularVelocity.magnitude;
        float normalizedDistance = currentDistance / maxCheckpointDistance;
        float normalizedAngularVelocity = currentAngularVelocity / drone.droneRigidBody.maxAngularVelocity;
        float normalizedAngleError = Mathf.Abs(AngleToGoal());
        float totalReward = 0.0f;

        totalReward += HelperFunctions.Reward(currentAngularVelocity, 5f);
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
                AddReward(1.0f);
                reachedGoal = true;
                EndEpisode();
            }
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Ground"))
        {

            Invoke(nameof(NotifyTimeout), 1.0f);

        }
    }

    private void OnCollisionExit(Collision collision)
    {
        if (collision.gameObject.CompareTag("Ground"))
        {
            CancelInvoke();
        }
    }
    private void NotifyTimeout()
    {
        SetReward(-10.0f);
        EndCurrentEpisode("Collision");
    }
}
