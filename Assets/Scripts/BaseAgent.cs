using DroneProject;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class BaseAgent : Agent
{
    public DroneControl drone { get; private set; }
    public int decisionInterval { get; set; }

    public float[] previousActions;
    private const int numberOfActions = 4;
    
    private Vector3 localVelocity;
    private Vector3 droneDefaultPosition;

    protected float lookAngle;

    [SerializeField, Tooltip("Penalty is applied at collision enter and stay")]
    private float collisionPenalty = 0.1f;
    private int collisionCount;


    public override void Initialize()
    {
        decisionInterval = GetComponent<DecisionRequester>().DecisionPeriod;

        drone = GetComponentInChildren<DroneControl>();
        drone.Initialize();
        droneDefaultPosition = drone.transform.position;

        // Check if drone exists
        if (drone == null)
        {
            Debug.LogError("Drone is NULL");
            return;
        }

        previousActions = new float[numberOfActions];

        // Actions
        drone.TipOverEvent += EndEpisode;
        drone.CollisionEvent += OnCollision;
        drone.CollisionTimeoutEvent += EndEpisode;
    }

    private void OnCollision(Collision collision)
    {
        AddReward(-collisionPenalty);
        collisionCount++;
    }

    public override void OnEpisodeBegin()
    {
        // Make sure we remove any of the previous actions values
        Array.Clear(previousActions, 0, numberOfActions);
        drone.transform.position = droneDefaultPosition;
        drone.ResetDrone(droneDefaultPosition);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        base.CollectObservations(sensor);
        sensor.AddObservation(HelperFunctions.Sigmoid(drone.localVelocity));
        sensor.AddObservation(drone.inclination);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        base.OnActionReceived(actionBuffers);
        var actions = actionBuffers.ContinuousActions.Array;
        // Last cycle step
        int step = StepCount % decisionInterval;
        if (step == 0)
        {
            Array.Copy(actions, previousActions, numberOfActions);
        } 
        else
        {
            /*In a real - world scenario, a quadcopter's movements are typically smooth and gradual. 
             * Interpolating actions can simulate this smooth transition by gradually changing from one action to the next, rather than making abrupt changes. 
            This is especially useful in simulations where actions are discrete or updated at intervals.*/
            float t = step / (float)decisionInterval;

            for (int i = 0; i < numberOfActions; i++)
            {
                actions[i] = Mathf.Lerp(previousActions[i], actions[i], t);
            }
        }
        drone.ApplyActions(actions);
    }

    public virtual void AddRewards()
    {
    }


}
