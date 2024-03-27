using DroneProject;
using System;
using System.Collections;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.Sentis;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.XR;
using UnityEngine.UIElements;

public class BaseAgent : Agent
{
    new private Rigidbody rigidbody;
    [HideInInspector] public DroneControl drone { get; private set; }

    public int decisionInterval { get; set; }
    private const int numberOfActions = 4;

    protected Vector3 targetLocalVelocity;
    protected float targetDirectionAngle;

    protected float localLookAngle;
    protected Vector3 localGoalVelocity;
    protected bool isFrozen = false;

    protected bool m_enableFlag;
    protected Bounds bounds;

    // Buffer storing recent target velocities to assess drone's tracking accuracy over time, compensating for response lag and smoothing erratic target movements.

    protected override void Awake()
    {
        base.Awake();
        drone = GetComponent<DroneControl>();

        decisionInterval = GetComponent<DecisionRequester>().DecisionPeriod;

        // Check if drone exists
        if (drone == null)
        {
            Debug.LogError("Drone is NULL");
            return;
        }
        bounds = new Bounds(transform.position, Vector3.one * 64);

    }

    protected void DefaultPhysicsObservations(VectorSensor sensor)
    {
        // Length of 3+3+3 = 9
        sensor.AddObservation(HelperFunctions.Sigmoid(drone.localVelocity, 0.5f));
        sensor.AddObservation(HelperFunctions.Sigmoid(drone.worldAngularVelocity, 1f));
        /*sensor.AddObservation(drone.localVelocity / drone.maxLinearVelocity);
        sensor.AddObservation(drone.worldAngularVelocity / drone.maxAngularVelocity);*/
        sensor.AddObservation(drone.inclination);
    }

    public override void Initialize()
    {

    }

    public void SetDroneTarget(Vector3 worldTargetVelocity, Vector3 localLookDirection)
    {
        targetLocalVelocity = drone.WorldToLocal(worldTargetVelocity);
        targetDirectionAngle = Vector3.SignedAngle(Vector3.forward, Vector3.ProjectOnPlane(drone.WorldToLocal(localLookDirection), Vector3.up), Vector3.up) / 180.0f;
    }

    protected void EndCurrentEpisode(string message)
    {
        // AddReward(-(MaxStep - StepCount));
        // AddReward(-terminationPenalty / (StepCount+1));
        // Logger.LogMessage("AGENT: " + transform.name + "Ended because: " + message + " : STEP : " + StepCount + " REWARD: " + GetCumulativeReward(), forceMessage: true);
        // AddReward(-0.1f * (MaxStep-StepCount));

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
        //DefaultPhysicsObservations(sensor);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        base.OnActionReceived(actionBuffers);

        // small reward
        // AddReward(0.1f / (MaxStep));

        var actions = actionBuffers.ContinuousActions.Array;

        drone.ApplyActions(actions);

        if (transform.up.y < drone.tipOverThreshold && !drone.isTippedOver) 
        {
            drone.isTippedOver = true;
            EndCurrentEpisode("Ttipped over");
        }
    }


    #region Collisions And Triggers
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Ground"))
        {
            EndCurrentEpisode("Crashed!");
        }
    }

    // Give penalty based on steps, after which the episode is ended
    private void AddTerminationPenalty(float penalty)
    {
        //AddReward(penalty); // * MaxStep / (StepCount + 1)
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Goal"))
        {
        }

        if (other.CompareTag("Enviroment"))
        {
            // AddTerminationPenalty(-leftEnviromentPenalty);
            // EndCurrentEpisode("Left enviroment!");
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
