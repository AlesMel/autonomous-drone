using DroneProject;
using System;
using System.Collections;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.Sentis.Layers;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.XR;
using UnityEngine.UIElements;
using static TMPro.SpriteAssetUtilities.TexturePacker_JsonArray;
using static UnityEngine.GraphicsBuffer;

public class BaseAgent : Agent
{
    new private Rigidbody rigidbody;
    public DroneControl drone { get; private set; }
    [SerializeField] public Goal goal;

    public int decisionInterval { get; set; }
    private const int numberOfActions = 4;
    private float[] prevActions;

    protected float lookAngle;

    // [SerializeField, Tooltip("Penalty is applied at collision enter")]
    protected float crashedPenalty = 1.0f;
    protected float tipOverPenalty = 10;
    protected float leftEnviromentPenalty = 0;
    protected float collisionPenalty = 100;
    protected float stepPenalty = 1.0f;
    protected float terminationPenalty = 100000000;
    protected float touchedGoalReward = 1f;
    protected float stayGoalReward = 0.1f;
    protected float leftGoalPenalty = 0.0f;
    protected float thresholdDistance;
    // 
    protected float maxCheckpointDistance = 69.282032f;
    protected float localLookAngle;
    protected Vector3 localGoalVelocity;
    protected bool isFrozen = false;

    protected bool m_enableFlag;
    private bool isColliding = false;
    protected bool isInGoal = false;
    protected bool reachedGoal = false;
    protected int numReachedGoal = 0;
    protected Bounds bounds;
    protected float previousDistance = 0.0f;

    private new void Awake()
    {
        drone = GetComponent<DroneControl>();
        decisionInterval = GetComponent<DecisionRequester>().DecisionPeriod;
        prevActions = new float[4];

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
        bounds = new Bounds(transform.position, Vector3.one * 64);

    }

    public override void Initialize()
    {

    }

    protected void EndCurrentEpisode(string message)
    {
        // AddReward(-(MaxStep - StepCount));
        // AddReward(-terminationPenalty);
        EndEpisode();
    }

    public override void OnEpisodeBegin()
    {
        base.OnEpisodeBegin();
        drone.BaseReset();
        Array.Clear(prevActions, 0, 4);
        reachedGoal = false;
    }

    public float AngleToGoal()
    {
        float rotation = Vector3.Angle(Vector3.ProjectOnPlane(drone.transform.forward, Vector3.up), Vector3.ProjectOnPlane(VectorToNextCheckpoint(), Vector3.up));
        return rotation / 180.0f;
    }

    public override void CollectObservations(VectorSensor sensor)
    { 
        base.CollectObservations(sensor);

        sensor.AddObservation(AngleToGoal());
        //sensor.AddObservation(drone.localVelocity / drone.maxLinearVelocity);
        //sensor.AddObservation(drone.localAngularVelocity / drone.maxAngularVelocity);
        sensor.AddObservation(HelperFunctions.Sigmoid(drone.localVelocity, 0.25f));
        sensor.AddObservation(HelperFunctions.Sigmoid(drone.localAngularVelocity));
        sensor.AddObservation(HelperFunctions.Sigmoid(VectorToNextCheckpoint()));
        // sensor.AddObservation(drone.inclination);
        // sensor.AddObservation(drone.actions); // 4 actions (4 rotors)
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        base.OnActionReceived(actionBuffers);
        var actions = actionBuffers.ContinuousActions.Array;

        int step = StepCount % decisionInterval;

        if (step == 0)
        {
            // Last cycle step: buffer and apply actions as is.
            Array.Copy(actions, prevActions, 4);
        }
        else
        {
            // Interpolate: previous cycle's actions -> current actions.
            float t = step / (float)decisionInterval;

            for (int i = 0; i < 4; i++)
            {
                actions[i] = Mathf.Lerp(prevActions[i], actions[i], t);
            }
        }


        drone.ApplyActions(actions);

        if (transform.up.y < drone.tipOverThreshold && !drone.isTippedOver) 
        {
            drone.isTippedOver = true;
            EndCurrentEpisode("Ttipped over");
        }
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
        //return drone.transform.InverseTransformPoint(goal.transform.position);
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
        //AddReward(penalty); // * MaxStep / (StepCount + 1)
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Goal") && !reachedGoal)
        {
            AddReward(touchedGoalReward);
            reachedGoal = true;
        }
    }

    private void OnTriggerStay(Collider other)
    {
        if (other.CompareTag("Goal"))
        {
            AddReward(stayGoalReward);
           isInGoal = true;
        }

    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Goal"))
        {
            isInGoal = false;
        }

        if (other.CompareTag("Enviroment") && drone.isInEnviroment)
        {
            drone.isInEnviroment = false;
            // AddTerminationPenalty(-leftEnviromentPenalty);
            // EndCurrentEpisode("Left enviroment!");
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
