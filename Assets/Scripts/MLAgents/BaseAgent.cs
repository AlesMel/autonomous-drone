using System;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.InputSystem.XR;
using UnityEngine.UIElements;

public class BaseAgent : Agent
{
    public DroneControl drone { get; private set; }
    [SerializeField] public Goal goal;

    public int decisionInterval { get; set; }

    public float[] previousActions;
    private const int numberOfActions = 4;
    
    private Vector3 localVelocity;
    private Vector3 droneDefaultPosition;

    protected float lookAngle;
    [Space, SerializeField, Tooltip("The maximum distance between checkpoint and drone")]
    private float checkpointDistanceThreshold = 101;

    // [SerializeField, Tooltip("Penalty is applied at collision enter")]
    private float collisionPenalty = 10.0f;
    //  [SerializeField, Tooltip("Penalty is applied at collision stay")]
    private float collisionStayPenalty = 1000.0f;
    // [SerializeField, Tooltip("Penalty is applied at tip over event")]
    private float tipOverPenalty = 50.0f;
    // [SerializeField, Tooltip("Penalty is applied at tip over event")]
    private float leftEnviromentPenalty = 100.0f;
    // [Space, SerializeField, Tooltip("Reward is applied at tip over event")]
    private float reachedGoalReward = 10000.0f;
    private float touchedGoalReward = 100.0f;

    protected float thresholdDistance;

    private int collisionCount;

    protected float localLookAngle;
    protected Vector3 localGoalVelocity;
    

    public override void Initialize()
    {
        decisionInterval = GetComponent<DecisionRequester>().DecisionPeriod;

        drone = GetComponentInChildren<DroneControl>();
        drone.Initialize();

        if (goal == null)
        {
            Debug.LogError("Goal is NULL");
        }
        thresholdDistance = VectorToNextCheckpoint().magnitude;
        droneDefaultPosition = drone.transform.position;

        // Check if drone exists
        if (drone == null)
        {
            Debug.LogError("Drone is NULL");
            return;
        }

        previousActions = new float[numberOfActions];

        // Actions
        drone.TipOverEvent += OnTipOverEvent;
        drone.CollisionEvent += OnCollision;
        drone.CollisionTimeoutEvent += OnCollisionTimeout;
        drone.LeftEnviromentEvent += OnLeftEnviromentEvent;
        goal.ReachedGoalEvent += OnReachedGoalEvent;
        goal.GoalTouchedEvent += OnGoalTouchedEvent;
    }
        

    public override void OnEpisodeBegin()
    {
        // Make sure we remove any of the previous actions values
        Array.Clear(previousActions, 0, numberOfActions);
        goal.ResetGoal();
        drone.ResetDrone(droneDefaultPosition);
    }

    public override void CollectObservations(VectorSensor sensor)
    { 
        base.CollectObservations(sensor);
        // direction to goal
        // distance to goal
        /*Vector3 vectorToNextCheckpoint = VectorToNextCheckpoint() / checkpointDistanceThreshold;
        Vector3 goalPosition = goal.transform.position;

        float distanceToGoal = vectorToNextCheckpoint.magnitude; // Vector3.Distance(goal.transform.position, drone.worldPosition);
        float dotToGoal = Vector3.Dot(drone.transform.forward, goal.transform.forward);

        sensor.AddObservation(vectorToNextCheckpoint);
        sensor.AddObservation(distanceToGoal);
        // Smer k cielu (skalarny sucin)
        sensor.AddObservation(dotToGoal);
        // Pozicia ciela
        sensor.AddObservation(goal.transform.position / goalPosition.magnitude);
        // Natocenie drona okolo osi y
        sensor.AddObservation(drone.inclination);
        sensor.AddObservation(HelperFunctions.Sigmoid(drone.localVelocity, 0.25f));
        sensor.AddObservation(HelperFunctions.Sigmoid(drone.localAngularVelocity));
        // Thrusts
        for (int i = 0; i < numberOfActions; i++)
        {
            sensor.AddObservation(drone.thrusts[i] / drone.thrustFactor);
        }*/

        Vector3 vectorToCheckpoint = VectorToNextCheckpoint();
        float distanceToCheckpoint = vectorToCheckpoint.magnitude;
        Vector3 normalizedVectorToCheckpoint = vectorToCheckpoint / distanceToCheckpoint; // Normalize the vector to have a magnitude of 1

        float dotToGoal = Vector3.Dot(drone.transform.forward, vectorToCheckpoint); // Corrected to compare drone's forward direction with the direction towards the goal

        sensor.AddObservation(normalizedVectorToCheckpoint); // Direction to next checkpoint, normalized
        sensor.AddObservation(distanceToCheckpoint / checkpointDistanceThreshold); // Distance to checkpoint, scaled by threshold
        sensor.AddObservation(dotToGoal); // Alignment towards goal
        sensor.AddObservation(drone.inclination); // Drone's inclination
        sensor.AddObservation(HelperFunctions.Sigmoid(drone.localVelocity.magnitude, 0.25f)); // Sigmoid of drone's local velocity magnitude
        sensor.AddObservation(HelperFunctions.Sigmoid(drone.localAngularVelocity.magnitude)); // Sigmoid of drone's local angular velocity magnitude

        for (int i = 0; i < numberOfActions; i++)
        {
            sensor.AddObservation(drone.thrusts[i] / drone.thrustFactor); // Normalized thrusts
        }

        /*Debug.Log("Dot product: " + dotToGoal);
        Debug.Log("Goal position: " + goalPosition/goalPosition.magnitude);
        Debug.Log("Distance: " + distanceToGoal);
        Debug.Log("Inclination" + drone.inclination.ToString());
        Debug.Log("Local velocity: " + drone.localVelocity.ToString());
        Debug.Log("Local angular velocity: " + drone.localAngularVelocity.ToString());*/


        // Vizualizácia vektora forward pre drone
        // Debug.DrawLine(drone.transform.position, drone.transform.position + drone.transform.forward * 2, Color.blue);

        // Vizualizácia vektora forward pre goal
        // Debug.DrawLine(goal.transform.position, goal.transform.position + goal.transform.forward * 2, Color.blue);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        base.OnActionReceived(actionBuffers);
        var actions = actionBuffers.ContinuousActions.Array;
        // Last cycle step
        int step = StepCount % decisionInterval;
       // Logger.LogMessage("Pre lerp: " + string.Join(", ", actions));
      /*  if (step == 0)
        {
            Array.Copy(actions, previousActions, numberOfActions);
        }
        else
        {
            *//*In a real - world scenario, a quadcopter's movements are typically smooth and gradual. 
             * Interpolating actions can simulate this smooth transition by gradually changing from one action to the next, rather than making abrupt changes.
            This is especially useful in simulations where actions are discrete or updated at intervals.*//*
            float t = step / (float)decisionInterval;

            for (int i = 0; i < numberOfActions; i++)
            {
                actions[i] = Mathf.Lerp(previousActions[i], actions[i], t);
            }
        }*/
        Logger.LogMessage("After lerp: " + string.Join(", ", actions));
        drone.ApplyActions(actions);
    }

    protected Vector3 VectorToNextCheckpoint()
    {
        Vector3 checkpointDirection = goal.transform.position - drone.worldPosition;
        Vector3 localCheckpointDirection = transform.InverseTransformDirection(checkpointDirection);
        return localCheckpointDirection;
    }

    private Vector3 OrientationToCheckpoint()
    {
        return transform.InverseTransformDirection(goal.transform.forward);
    }

    private void DrawRays()
    {
        // Calculate the direction in world coordinates
        Vector3 worldDirectionToGoal = VectorToNextCheckpoint();

        // Draw the ray from the drone's current position to the goal
        Debug.DrawRay(drone.worldPosition, worldDirectionToGoal, Color.black);
        Debug.DrawLine(OrientationToCheckpoint(), drone.worldPosition, Color.blue);
    }

    #region Goal settings
    private void SetWorldGoal(Vector3 worldVelocity, Vector3 worldLookDirection)
    {
        SetLocalGoal(drone.WorldToLocal(worldVelocity), drone.WorldToLocal(worldLookDirection));
    }

    private void SetLocalGoal(Vector3 localVelocity, Vector3 localLookDirection)
    {
        // https://forum.unity.com/threads/projection-of-point-on-plane.855958/
        localLookDirection = Vector3.ProjectOnPlane(localLookDirection, Vector3.up);
        SetLocalGoal(localVelocity, Vector3.SignedAngle(Vector3.forward, localLookDirection, Vector3.up) / 180.0f);
        Debug.DrawLine(goal.transform.position, localLookDirection, Color.magenta);
    }

    private void SetLocalGoal(Vector3 localGoalVelocity, float localLookAngle)
    {
        this.localGoalVelocity = localGoalVelocity;
        this.localLookAngle = localLookAngle;
        Debug.Log("Local look angle: " + localLookAngle);
    }

    #endregion


    #region Actions/Invokes

    private void OnTipOverEvent()
    {
        AddReward(-tipOverPenalty);
        EndEpisode();
    }

    private void OnCollisionTimeout()
    {
        AddReward(-collisionStayPenalty);
        EndEpisode();
    }

    private void OnCollision(Collision collision)
    {
        AddReward(-collisionPenalty);
        collisionCount++;
    }

    private void OnReachedGoalEvent()
    {
        AddReward(reachedGoalReward);
        EndEpisode();
    }

    private void OnLeftEnviromentEvent()
    {
        AddReward(-leftEnviromentPenalty);
        EndEpisode();
    }

    private void OnGoalTouchedEvent()
    {
        AddReward(touchedGoalReward);
    }

    #endregion

    #region Virtual methods
    public virtual void AddRewards() { }
    #endregion
}
