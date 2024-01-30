using DroneProject;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine.InputSystem;
using UnityEngine;
using System;

public class DroneAgent : Agent
{
    public int DecisionInterval { get; set; }

    [Header("Drone")]
    public GameObject drone;

    public Goal goal;
    public DroneControl droneControl;
    private CageTriggerDetection triggerDetection;

    [Header("Sensors")]
    public VectorSensor vectorSensor;

    private Rigidbody droneRigidBody;
    EnvironmentParameters resetParams;

    [Header("Penalties")]
    [SerializeField, Tooltip("Penalty after coliding")]
    private float collisionPenalty = 0.1f;
    private int collisionCount = 0;

    // [SerializeField, Tooltip("Penalty after tipping over")]
    private float tipOverPenalty = 10.0f;

    // Drone's initial coordinate system
    private Vector3 startingPosition;
    private Quaternion startingRotation;

    protected float localLookAngle;
    private Vector3 localGoalVelocity;

    // Buffer for action interpolation.
    private float[] previousActions;

    public override void Initialize()
    {
        drone = transform.Find("Drone").gameObject;
        droneControl = drone.GetComponent<DroneControl>();

        goal = GetComponentInChildren<Goal>();

        // Check if essential components are found
        if (drone == null || goal == null || droneControl == null)
        {
            Debug.LogError("Essential component(s) missing in DroneAgent. Please check Drone, Goal, and DroneControl assignments.");
            return;
        }

        droneRigidBody = drone.GetComponent<Rigidbody>();
        if (droneRigidBody == null)
        {
            Debug.LogError("Rigidbody component missing in the drone GameObject.");
            return;
        }
        
        startingPosition = droneControl.transform.position;
        startingRotation = droneControl.transform.rotation;

        resetParams = Academy.Instance.EnvironmentParameters;
        ResetEnv();
        InitSubscribers();
    }
   
    private void SetWorldGoal(Vector3 worldVelocity, Vector3 worldLookDirection)
    {
        SetLocalGoal(droneControl.WorldToLocal(worldVelocity), droneControl.WorldToLocal(worldLookDirection));
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

    private void InitSubscribers()
    {
        droneControl.TipOverEvent += OnTipOver;
        droneControl.CollisionEvent += OnCollision;
        droneControl.CollisionTimeoutEvent += EndEpisode;

    }

    private void OnCollision(Collision collision)
    {
        AddReward(-collisionPenalty);
        collisionCount++;
    }

    private void OnTipOver()
    {
        AddReward(-tipOverPenalty);
        EndEpisode();
    }

    private void ResetEnv()
    {
        //goal.SpawnObject();
        drone.transform.position= startingPosition; 
        drone.transform.rotation= startingRotation;
    }
    private Vector3 VectorToNextCheckpoint()
    {
        Vector3 checkpointDirection = goal.transform.position - droneControl.worldPosition;
        Vector3 localCheckpointDirection = transform.InverseTransformDirection(checkpointDirection);
        return localCheckpointDirection;
    }

    private Vector3 OrientationToCheckpoint()
    {
        return transform.InverseTransformDirection(goal.transform.forward);
    }


    public override void CollectObservations(VectorSensor sensor)
    {
        SetWorldGoal(goal.worldVelocity, goal.worldLookDirection);
        /*      base.CollectObservations(sensor);

              var velocity = droneControl.localVelocity;
              //var angularVelocity = HelperFunctions.Sigmoid(droneControl.localAngularVelocity);
              var inclination = droneControl.inclination;

              // how far is the checkpoint
              sensor.AddObservation(velocity);
              //sensor.AddObservation(angularVelocity);
              sensor.AddObservation(inclination);
              //Orientation of the next checkpoint
              sensor.AddObservation(OrientationToCheckpoint());

              Debug.Log("Vector: " + VectorToNextCheckpoint());*/
        // 17
        sensor.AddObservation(VectorToNextCheckpoint()); // direction to the next checkpoint
        //sensor.AddObservation(VectorToNextCheckpoint().magnitude); // Relative Distance to Goal
        //sensor.AddObservation(droneControl.worldForwardXZ); // horizontal orientation of the drone
        sensor.AddObservation(HelperFunctions.Sigmoid(droneControl.localVelocity));
        sensor.AddObservation(HelperFunctions.Sigmoid(droneControl.localAngularVelocity));
        sensor.AddObservation(droneControl.inclination);
        //sensor.AddObservation(droneControl.rotationY.eulerAngles.y / 360.0f); // Drone's Yaw Rotation


    }

    private void FixedUpdate()
    {
        DrawRays();

    }

    private void DrawRays()
    {
        // Calculate the direction in world coordinates
        Vector3 worldDirectionToGoal = VectorToNextCheckpoint();

        // Draw the ray from the drone's current position to the goal
        Debug.DrawRay(droneControl.worldPosition, worldDirectionToGoal, Color.black);
        Debug.DrawLine(OrientationToCheckpoint(), droneControl.worldPosition, Color.blue);

    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        base.OnActionReceived(actions);
        droneControl.ApplyActions(actions.ContinuousActions.Array);

        var current_actions = actions.ContinuousActions.Array;
        int step = StepCount % DecisionInterval;
        Debug.Log("Step: " + step);
        if (step == 0)
        {
            // Last cycle step: buffer and apply actions as is.
            Array.Copy(current_actions, previousActions, 4);
        }
        else
        {
            // Interpolate: previous cycle's actions -> current actions.
            float t = step / (float)DecisionInterval;

            for (int i = 0; i < 4; i++)
            {
                current_actions[i] = Mathf.Lerp(previousActions[i], current_actions[i], t);
            }
        }

        droneControl.ApplyActions(current_actions);
/*        if (triggerDetection.IsDronePresent()) 
        {
            SetReward(-10f / Academy.Instance.StepCount);
            // EndEpisode();
        }
        else if (goal.shouldContinue)
        {
            AddReward(0.1f);
        }*/
    }

    private void AddRewards()
    {
        var distance = VectorToNextCheckpoint().magnitude/8;
        float stabilityError = droneControl.worldAngularVelocity.magnitude;
        AddReward(-distance);

    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
      /*  // var actions = droneControl.actions;
        *//*        for (int i = 0; i < actions.Length; i++)
                {
                    actions[i] = 1.0f; // Base thrust needed to hover
                }
                // Forward and Backward - Adjust front and back rotors
                if (Input.GetKey(KeyCode.W))
                {
                    actions[0] = actions[1] = 1.1f; // Increase front rotors' thrust
                }
                else if (Input.GetKey(KeyCode.S))
                {
                    actions[2] = actions[3] = 1.1f; // Increase back rotors' thrust
                }

                // Left and Right - Adjust left and right rotors
                if (Input.GetKey(KeyCode.A))
                {
                    actions[1] = actions[3] = 1.1f; // Increase right rotors' thrust
                }
                else if (Input.GetKey(KeyCode.D))
                {
                    actions[0] = actions[2] = 1.1f; // Increase left rotors' thrust
                }*/
        /*     var actions = droneControl.actions;

             // Rotation - Adjust rotational torque
             if (Input.GetKey(KeyCode.Q))
             {
                 // To rotate left, increase thrust in clockwise rotors and decrease in counterclockwise rotors
                 actions[0] = 2f; // Assuming rotor 0 spins clockwise
                 actions[1] = 4f; // Assuming rotor 1 spins counterclockwise
                 actions[2] = 4f; // Assuming rotor 3 spins clockwise
                 actions[3] = 2f; // Assuming rotor 2 spins counterclockwise
             }
             else if (Input.GetKey(KeyCode.E))
             {
                 // To rotate right, do the opposite
                 actions[0] = 4f; // Assuming rotor 1 spins counterclockwise
                 actions[1] = 2f; // Assuming rotor 0 spins clockwise
                 actions[2] = 2f; // Assuming rotor 2 spins counterclockwise
                 actions[3] = 4f; // Assuming rotor 3 spins clockwise
             }

             // roll
             if (Input.GetKey(KeyCode.A))
             {
                 // To rotate left, increase thrust in clockwise rotors and decrease in counterclockwise rotors
                 actions[0] = 4f; // Assuming rotor 0 spins clockwise
                 actions[1] = 4f; // Assuming rotor 1 spins counterclockwise
                 actions[2] = 1f; // Assuming rotor 3 spins clockwise
                 actions[3] = 1f; // Assuming rotor 2 spins counterclockwise
             }
             else if (Input.GetKey(KeyCode.D))
             {
                 // To rotate right, do the opposite
                 actions[0] = 1f; // Assuming rotor 1 spins counterclockwise
                 actions[1] = 1f; // Assuming rotor 0 spins clockwise
                 actions[2] = 4f; // Assuming rotor 2 spins counterclockwise
                 actions[3] = 4f; // Assuming rotor 3 spins clockwise
             }

             // Ascent and Descent
             if (Input.GetKey(KeyCode.UpArrow))
             {
                 actions[0] = actions[1] = actions[2] = actions[3] = 4.0f; // Increase all rotors' thrust for ascent
             }
             else if (Input.GetKey(KeyCode.DownArrow))
             {
                 actions[0] = actions[1] = actions[2] = actions[3] = 0.9f; // Decrease all rotors' thrust for descent
             }*//*
        // Reading joystick input

        DroneInputs inputs = GetComponentInChildren<DroneInputs>(); // Assuming DroneInputs is on the same GameObject

        float pitch = inputs.Cyclic.y; // Assuming Y axis of cyclic controls pitch
        float roll = inputs.Cyclic.x;  // Assuming X axis of cyclic controls roll
        float yaw = inputs.Pedals;
        float throttle = inputs.Throttle;

        float[] actions = new float[4];
        actions[0] = actions[1] = actions[2] = actions[3] = throttle; // Base throttle for all rotors

        // Apply pitch adjustment
        if (pitch != 0)
        {
            actions[0] += pitch; // Front Left Rotor
            actions[1] += pitch; // Front Right Rotor
            actions[2] -= pitch; // Rear Left Rotor
            actions[3] -= pitch; // Rear Right Rotor
        }

        // Apply roll adjustment
        if (roll != 0)
        {
            actions[0] -= roll; // Front Left Rotor
            actions[1] += roll; // Front Right Rotor
            actions[2] += roll; // Rear Left Rotor
            actions[3] -= roll; // Rear Right Rotor
        }

        // Apply yaw adjustment
        if (yaw != 0)
        {
            actions[0] -= yaw; // Front Left Rotor
            actions[1] += yaw; // Front Right Rotor
            actions[2] -= yaw; // Rear Left Rotor
            actions[3] += yaw; // Rear Right Rotor
        }

        Debug.Log("Actions: " + actions.ToString());
        droneControl.ApplyActions(actions);
*/
    }
}
