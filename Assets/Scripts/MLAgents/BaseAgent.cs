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
    public DroneControl drone { get; private set; }
    public Goal[] goals;

    public int decisionInterval { get; set; }
    private const int numberOfActions = 4;
    private float[] prevActions;

    protected float lookAngle;

    // [SerializeField, Tooltip("Penalty is applied at collision enter")]
    protected float crashedPenalty = 1.0f;  
    protected float tipOverPenalty = 0;
    protected float leftEnviromentPenalty = 0;
    protected float collisionPenalty = 2000;
    protected float stepPenalty = 1.0f;
    protected float terminationPenalty = 100000000;
    protected float touchedGoalReward = 1.0f; // 500 good
    protected float stayGoalReward = 10.0f; // 30 good
    protected float leftGoalPenalty = 0.0f;
    protected float thresholdDistance;
    // 
    protected float maxCheckpointDistance = 16.0f;
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
    protected int currentCheckpointIndex = 0;
    private Coroutine delayCoroutine;

    private new void Awake()
    {
        drone = GetComponent<DroneControl>();
        decisionInterval = GetComponent<DecisionRequester>().DecisionPeriod;
        prevActions = new float[4];

        thresholdDistance = VectorToNextCheckpoint().magnitude;
        // Check if drone exists
        if (drone == null)
        {
            Debug.LogError("Drone is NULL");
            return;
        }
        bounds = new Bounds(transform.position, Vector3.one * maxCheckpointDistance);
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
        previousDistance = thresholdDistance;
        currentCheckpointIndex = 0;
    }

    public float AngleToGoal()
    {
        /*float rotation = Vector3.Angle(Vector3.ProjectOnPlane(drone.transform.forward, Vector3.up), Vector3.ProjectOnPlane(VectorToNextCheckpoint(), Vector3.up));
        return rotation / 180.0f;*/
        var targetAngleVector = Vector3.ProjectOnPlane(-goals[currentCheckpointIndex].transform.position, Vector3.up).normalized;
        var localVector = drone.WorldToLocal(targetAngleVector);
        var angle = Vector3.SignedAngle(Vector3.forward, localVector, Vector3.up) / 180f;
        return angle;
    }

    public override void CollectObservations(VectorSensor sensor)
    { 
        base.CollectObservations(sensor);

        // Debug.Log($"{transform.name} at {StepCount} {VectorToNextCheckpoint() / maxCheckpointDistance}");

        //sensor.AddObservation(drone.WorldToLocal(goal.transform.forward));

        sensor.AddObservation(AngleToGoal());
        sensor.AddObservation(VectorToNextCheckpoint() / maxCheckpointDistance);// 

/*        sensor.AddObservation(drone.localVelocity / drone.droneRigidBody.maxLinearVelocity);
        sensor.AddObservation(drone.localAngularVelocity / drone.droneRigidBody.maxAngularVelocity);*/
        sensor.AddObservation(HelperFunctions.Sigmoid(drone.localVelocity, 0.5f));
        sensor.AddObservation(HelperFunctions.Sigmoid(drone.localAngularVelocity));

        sensor.AddObservation(drone.inclination);
    }

    private void OnTipOver()
    {
        AddReward(-tipOverPenalty);
        EndEpisode();
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

        if (transform.up.y < drone.tipOverThreshold)
        {
            AddReward(-tipOverPenalty);
            EndCurrentEpisode("Ttipped over");
        }

        drone.ApplyActions(actions);
    }
    protected Vector3 GlobalVectorToNextCheckpoint()
    {
        Vector3 checkpointDirection = goals[currentCheckpointIndex].transform.position - drone.worldPosition;
        return checkpointDirection;
    }

    protected Vector3 VectorToNextCheckpoint()
    {
        Vector3 checkpointDirection = GlobalVectorToNextCheckpoint();
        Vector3 localCheckpointDireciton = drone.transform.InverseTransformDirection(checkpointDirection);
        return localCheckpointDireciton;
        //return drone.WorldToLocal(checkpointDirection);
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


    // Give penalty based on steps, after which the episode is ended
    private void AddTerminationPenalty(float penalty)
    {
        //AddReward(penalty); // * MaxStep / (StepCount + 1)
    }





    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Goal"))
        {
            isInGoal = false;
            if (delayCoroutine != null)
                StopCoroutine(delayCoroutine);
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

    private IEnumerator InGoalCoroutine()
    {
        reachedGoal = true;
        yield return new WaitForSeconds(2f); // Wait for 2 seconds
        currentCheckpointIndex++; // Increment the checkpoint index after 2 seconds
        reachedGoal = false;
    }


    public override void Heuristic(in ActionBuffers actionsOut)
    {
        base.Heuristic(actionsOut);
        var actions = actionsOut.ContinuousActions;

        float thrust = drone.worldVelocity.y * -0.25f;
        Vector3 incl = drone.inclination;
        float pitch = incl.z * -0.025f;
        float roll = incl.x * 0.025f;
        actions[0] = thrust + roll + pitch;
        actions[1] = thrust - roll - pitch;
        actions[2] = thrust - roll + pitch;
        actions[3] = thrust + roll - pitch;
        //actions[i] = (actions[i] + 1) * 0.5f;
    }
    #endregion

    #region Actions/Invokes


    #endregion

    #region Virtual methods
    public virtual void AddRewards() { }
    #endregion
}
