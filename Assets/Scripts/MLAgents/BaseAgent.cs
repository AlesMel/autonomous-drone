using System;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.VisualScripting;
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

    protected float lookAngle;
    [Space, SerializeField, Tooltip("The maximum distance between checkpoint and drone")]
    private float checkpointDistanceThreshold = 11;

    // [SerializeField, Tooltip("Penalty is applied at collision enter")]
    protected float crashedPenalty = 200.0f;
    protected float tipOverPenalty = 50.0f;
    protected float leftEnviromentPenalty = 200.0f;
    protected float touchedGoalReward = 15.0f;

    protected float thresholdDistance;

    protected float localLookAngle;
    protected Vector3 localGoalVelocity;

    public bool isInGoal = false;

    protected override void OnDisable()
    {
        base.OnDisable();
        UnsubscribeFromInvokes();
        EndCurrentEpisode("Disabled");
    }

    protected override void OnEnable()
    {
        base.OnEnable();
        Initialize();
    }

    public override void Initialize()
    {
        decisionInterval = GetComponent<DecisionRequester>().DecisionPeriod;

        drone = GetComponentInChildren<DroneControl>();
        drone.Initialize();

        if (goal == null)
        {
            Debug.LogError("Goal is NULL");
        }
        goal.ResetGoal();

        thresholdDistance = VectorToNextCheckpoint().magnitude;

        // Check if drone exists
        if (drone == null)
        {
            Debug.LogError("Drone is NULL");
            return;
        }

        previousActions = new float[numberOfActions];
        SubsribeToInvokes();
    }
    #region Subscribers
    private void SubsribeToInvokes()
    {
        // Actions
/*        drone.TipOverEvent += OnTipOverEvent;
        drone.CollisionEvent += OnCollision;
        drone.CollisionTimeoutEvent += OnCollisionTimeout;
        drone.LeftEnviromentEvent += OnLeftEnviromentEvent;*/
        //drone.ReachedGoalEvent += OnGoalTouchedEvent;
        //drone.LowAltitudeEvent += OnLowAltitudeEvent;
        //goal.ReachedGoalEvent += OnReachedGoalEvent;
    }

    private void UnsubscribeFromInvokes()
    {
        // Actions
/*        drone.TipOverEvent -= OnTipOverEvent;
        drone.CollisionEvent -= OnCollision;
        drone.CollisionTimeoutEvent -= OnCollisionTimeout;
        drone.LeftEnviromentEvent -= OnLeftEnviromentEvent;*/
        //drone.ReachedGoalEvent -= OnGoalTouchedEvent;
        // drone.LowAltitudeEvent += OnLowAltitudeEvent;
        // goal.ReachedGoalEvent -= OnReachedGoalEvent;
    }
    #endregion

    protected void EndCurrentEpisode(string message)
    {
       // Debug.Log("Ended because: " + message + " : STEP : " + StepCount);
        EndEpisode();
        drone.BaseReset();
    }

    public override void OnEpisodeBegin()
    {
        // Make sure we remove any of the previous actions values
        Array.Clear(previousActions, 0, numberOfActions);
        goal.ResetGoal();
    }

    public override void CollectObservations(VectorSensor sensor)
    { 
        base.CollectObservations(sensor);
        Vector3 vectorToCheckpoint = VectorToNextCheckpoint();
        float distanceToCheckpoint = vectorToCheckpoint.magnitude / checkpointDistanceThreshold;
        Vector3 normalizedVectorToCheckpoint = vectorToCheckpoint / checkpointDistanceThreshold; // Normalize the vector to have a magnitude of 1

        float dotToGoal = Vector3.Dot(drone.transform.forward, vectorToCheckpoint); // Corrected to compare drone's forward direction with the direction towards the goal

        sensor.AddObservation(vectorToCheckpoint); // Direction to next checkpoint, normalized
        sensor.AddObservation(vectorToCheckpoint.magnitude); // Distance to checkpoint, scaled by threshold
        sensor.AddObservation(dotToGoal); // Alignment towards goal
        sensor.AddObservation(drone.inclination); // Drone's inclination
        sensor.AddObservation(HelperFunctions.Sigmoid(drone.localVelocity, 0.5f)); // Sigmoid of drone's local velocity magnitude
        sensor.AddObservation(HelperFunctions.Sigmoid(drone.localAngularVelocity)); // Sigmoid of drone's local angular velocity magnitude

        // rotor's current actions
        sensor.AddObservation(drone.actions); // 4 actions (4 rotors)
        DrawRays();
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        base.OnActionReceived(actionBuffers);
        var actions = actionBuffers.ContinuousActions.Array;
        // Last cycle step
        int step = StepCount % decisionInterval;
        Logger.LogMessage("Before lerp: " + string.Join(", ", actions), forceMessage: false);

        /*       
          if (step == 0)
        {
            Logger.LogMessage("Into lerping step 0", forceMessage: true);
            Array.Copy(actions, previousActions, actions.Length);
        }
        else
        {


        }*/
        float t = step / (float) decisionInterval;

        for (int i = 0; i < actions.Length; i++)
        {
            actions[i] = Mathf.Lerp(previousActions[i], actions[i], 0.33f);
        }
        Array.Copy(actions, previousActions, actions.Length);

        Logger.LogMessage("After lerp: " + string.Join(", ", actions), forceMessage: false);
        drone.ApplyActions(actions);
    }

    protected Vector3 VectorToNextCheckpoint()
    {
        Vector3 checkpointDirection = goal.transform.localPosition - drone.transform.localPosition;
        Vector3 localCheckpointDirection = transform.InverseTransformDirection(checkpointDirection);
        return localCheckpointDirection;
    }

    private void DrawRays()
    {
        // Calculate the direction in world coordinates
        Vector3 worldDirectionToGoal = VectorToNextCheckpoint();

        // Draw the ray from the drone's current position to the goal
        Debug.DrawRay(drone.worldPosition, worldDirectionToGoal, Color.cyan);
        //Debug.DrawLine(OrientationToCheckpoint(), drone.worldPosition, Color.blue);
    }

    #region Goal settings

    private Vector3 OrientationToCheckpoint()
    {
        return transform.InverseTransformDirection(goal.transform.forward);
    }
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

   /* private void OnTipOverEvent()
    {
        AddReward(-tipOverPenalty / (StepCount + 1));
        EndCurrentEpisode("TipOver");
    }

    private void OnCollisionTimeout()
    {
        AddReward(-collisionStayPenalty / (StepCount + 1));
        EndCurrentEpisode("Collision Timeout");
    }

    private void OnCollision(Collision collision)
    {
        // AddReward(-collisionPenalty); //  / (StepCount + 1)
        collisionCount++;
    }

    private void OnReachedGoalEvent()
    {
        AddReward(touchedGoalReward);
        EndCurrentEpisode("Reached Goal");
    }

    private void OnLeftEnviromentEvent()
    {
        AddReward(-leftEnviromentPenalty);
        EndCurrentEpisode("Left Enviroment");
    }

    private void OnGoalTouchedEvent()
    {
        AddReward(touchedGoalReward);
        // Debug.Log("Touched goal!");
    }

    // Probably bugged trigger, so we need to check it like this
    private void OnLowAltitudeEvent()
    {
        //AddReward(-collisionStayPenalty);
        EndCurrentEpisode("Low Altitutde");
    }*/

    #endregion

    #region Virtual methods
    public virtual void AddRewards() { }
    #endregion
}
