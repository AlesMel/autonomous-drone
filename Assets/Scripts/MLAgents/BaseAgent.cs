using DroneProject;
using System;
using System.Collections;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.InputSystem.XR;
using UnityEngine.UIElements;

public class BaseAgent : Agent
{
    new private Rigidbody rigidbody;
    public DroneControl drone { get; private set; }
    [SerializeField] public Goal goal;

    public int decisionInterval { get; set; }
    private const int numberOfActions = 4;

    protected float lookAngle;

    // [SerializeField, Tooltip("Penalty is applied at collision enter")]
    protected float crashedPenalty = 1.0f;
    protected float tipOverPenalty = 500000.0f;
    protected float leftEnviromentPenalty = 500000.0f;
    protected float touchedGoalReward = 10.0f;
    protected float collisionPenalty = 500000.0f;
    protected float stepPenalty = 1.0f;
    protected float thresholdDistance;

    protected float localLookAngle;
    protected Vector3 localGoalVelocity;
    protected bool isFrozen = false;

    protected bool m_enableFlag;
    private bool isColliding = false;

    private new void Awake()
    {
        decisionInterval = GetComponent<DecisionRequester>().DecisionPeriod;
        rigidbody = GetComponent<Rigidbody>();
        drone = GetComponent<DroneControl>();

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
    }


    protected override void OnDisable()
    {
        base.OnDisable();
    }

    protected override void OnEnable()
    {
        base.OnEnable();
    }

    public override void Initialize()
    {

    }

    protected void EndCurrentEpisode(string message)
    {
        Logger.LogMessage("AGENT: " + transform.name + "Ended because: " + message + " : STEP : " + StepCount  + " REWARD: " + GetCumulativeReward(), forceMessage: true);
        EndEpisode();
    }

    public override void OnEpisodeBegin()
    {
        base.OnEpisodeBegin();
        drone.BaseReset();
    }

    public override void CollectObservations(VectorSensor sensor)
    { 
        base.CollectObservations(sensor);
        float angularVelocity = HelperFunctions.QuantizeValue(drone.worldAngularVelocity.magnitude);
        var (vectorToCheckpoint, orientation) = VectorAndOrientationToNextCheckpoint();

        //Observe drone velocity
        sensor.AddObservation(HelperFunctions.QuantizeVector3(drone.localVelocity));
        sensor.AddObservation(HelperFunctions.QuantizeVector3(drone.worldAngularVelocity));
        //Where is the next checkpoint
        sensor.AddObservation(HelperFunctions.QuantizeVector3(vectorToCheckpoint));
        //Orientation of the next checkpoint
        sensor.AddObservation(HelperFunctions.QuantizeVector3(orientation));
        sensor.AddObservation(drone.inclination);
        /*Debug.Log($"VTNC: {drone.transform.parent.name}: {vectorToCheckpoint.sqrMagnitude}");
        Debug.Log($"ORIENTANTION: {drone.transform.parent.name}: {orientation}");
*/

        //Total Observation = 3+3+3 = 9
        //float dotToGoal = Vector3.Dot(drone.transform.forward, VectorToNextCheckpoint()); // Corrected to compare drone's forward direction with the direction towards the goal

        /*Debug.Log($"Agent:                      {transform.parent.name}" +
            $"VectorToNextCheckpoint:           {vectorToCheckpoint}\n" +
            $"VectorToNextCheckpoint.magnitude: {vectorToCheckpoint.magnitude}\n" +
            $"dotToGoal:                        {0}\n" +
            $"drone.inclination:                {drone.inclination}\n" +
            $"drone.localVelocity:              {drone.localVelocity}\n" +
            $"drone.localAngularvelocity:       {drone.localAngularVelocity}");*/

        // rotor's current actions
        //sensor.AddObservation(drone.actions); // 4 actions (4 rotors)
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        base.OnActionReceived(actionBuffers);

        // rigidBody issue

        var actions = actionBuffers.ContinuousActions.Array;

        drone.ApplyActions(actions);
        //AddReward(-1.0f);

        if (transform.up.y < drone.tipOverThreshold && !drone.isTippedOver) 
        {
            drone.isTippedOver = true;
            //AddReward(-tipOverPenalty / (StepCount + 1));
            EndCurrentEpisode("Ttipped over");
        }
        DrawRays();
    }
    protected (Vector3 localDirectionToGoal, Vector3 orientation) VectorAndOrientationToNextCheckpoint()
    {
        Vector3 checkpointDirection = goal.transform.position - transform.position;

        float angleToGoal = Vector3.Angle(Vector3.forward, checkpointDirection);
        Vector3 crossProduct = Vector3.Cross(Vector3.forward, checkpointDirection);

        return (checkpointDirection, crossProduct);
    }

    protected Vector3 VectorToNextCheckpoint()
    {
        Vector3 checkpointDirection = goal.transform.position - transform.position;
        return checkpointDirection;
    }

    private void DrawRays()
    {
        // Calculate the direction in world coordinates
        Vector3 worldDirectionToGoal = VectorToNextCheckpoint();

        // Draw the ray from the drone's current position to the goal
        Debug.DrawRay(drone.transform.position, worldDirectionToGoal, Color.cyan);
        //Debug.DrawLine(OrientationToCheckpoint(), drone.worldPosition, Color.blue);
    }

    #region Collisions And Triggers
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Ground"))
        {
            // AddReward(-collisionPenalty / (StepCount + 1));
            EndCurrentEpisode("Crashed!");
            //isColliding = true;

        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Goal"))
        {
           // AddReward(touchedGoalReward);
        }
    }

    private void OnTriggerStay(Collider other)
    {
        if (other.CompareTag("Goal"))
        {
           //AddReward(touchedGoalReward);
        }

    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Enviroment") && drone.isInEnviroment)
        {
            drone.isInEnviroment = false;
            //AddReward(-leftEnviromentPenalty / (StepCount + 1));
            EndCurrentEpisode("Left enviroment!");
        }
    }

    private void OnCollisionExit(Collision collision)
    {
        if (collision.gameObject.CompareTag("Ground"))
        {
            isColliding = false;
        }
    }
    #endregion

    #region Actions/Invokes


    #endregion

    #region Virtual methods
    public virtual void AddRewards() { }
    #endregion
}
