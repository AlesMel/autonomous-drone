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
    protected float tipOverPenalty = 1000.0f;
    protected float leftEnviromentPenalty = 1000.0f;
    protected float collisionPenalty = 1000.0f;
    protected float stepPenalty = 1.0f;

    protected float touchedGoalReward = 5.0f;

    protected float thresholdDistance;
    // 
    protected float maxCheckpointDistance = 69.282032f;
    protected float localLookAngle;
    protected Vector3 localGoalVelocity;
    protected bool isFrozen = false;

    protected bool m_enableFlag;
    private bool isColliding = false;

    private new void Awake()
    {
        /*        drone = GetComponent<DroneControl>();
                decisionInterval = GetComponent<DecisionRequester>().DecisionPeriod;

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
                }*/
        drone = GetComponent<DroneControl>();
        decisionInterval = GetComponent<DecisionRequester>().DecisionPeriod;

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

    public override void Initialize()
    {

    }

    protected void EndCurrentEpisode(string message)
    {
        // AddReward(-(MaxStep - StepCount));
        // Logger.LogMessage("AGENT: " + transform.name + "Ended because: " + message + " : STEP : " + StepCount + " REWARD: " + GetCumulativeReward(), forceMessage: true);
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
        sensor.AddObservation(drone.localVelocity / drone.maxLinearVelocity);
        sensor.AddObservation(drone.worldAngularVelocity / drone.maxAngularVelocity);
        //Where is the next checkpoint
        sensor.AddObservation(vectorToCheckpoint / maxCheckpointDistance);
        //Orientation of the next checkpoint
        //sensor.AddObservation(orientation);
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

        if (transform.up.y < drone.tipOverThreshold && !drone.isTippedOver) 
        {
            drone.isTippedOver = true;
            // AddTerminationPenalty(-tipOverPenalty);
            EndCurrentEpisode("Ttipped over");
        }
        //DrawRays();
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
            // AddTerminationPenalty(-collisionPenalty);
            EndCurrentEpisode("Crashed!");
            //isColliding = true;

        }
    }

    // Give penalty based on steps, after which the episode is ended
    private void AddTerminationPenalty(float penalty)
    {
        AddReward(penalty ); // * MaxStep / (StepCount + 1)
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Goal"))
        {
           AddReward(touchedGoalReward);
        }
    }

    private void OnTriggerStay(Collider other)
    {
        if (other.CompareTag("Goal"))
        {
           AddReward(touchedGoalReward);
        }

    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Enviroment") && drone.isInEnviroment)
        {
            drone.isInEnviroment = false;
            // AddTerminationPenalty(-leftEnviromentPenalty);
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

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var actions = actionsOut.ContinuousActions;

        float thrust = drone.worldVelocity.y * -0.25f;
        Vector3 incl = drone.inclination;
        float pitch = incl.z * -0.025f;
        float roll = incl.x * 0.025f;
        actions[0] = thrust + roll + pitch;
        actions[1] = thrust - roll - pitch;
        actions[2] = thrust - roll + pitch;
        actions[3] = thrust + roll - pitch;
    }
    #endregion

    #region Actions/Invokes


    #endregion

    #region Virtual methods
    public virtual void AddRewards() { }
    #endregion
}
