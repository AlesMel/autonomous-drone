using DroneProject;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class DroneAgent : Agent
{
    [Header("Drone")]
    public GameObject drone;

    public Goal goal;
    public DroneControl droneControl;
    private CageTriggerDetection triggerDetection;

    [Header("Sensors")]
    public VectorSensor vectorSensor;

    private Rigidbody droneRigidBody;
    EnvironmentParameters resetParams;


    // Drone's initial coordinate system
    private Vector3 startingPosition;
    private Quaternion startingRotation;


    // Start is called before the first frame update
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
    }

    private void ResetEnv()
    {
        //goal.SpawnObject();
        drone.transform.position= startingPosition; 
        drone.transform.rotation= startingRotation;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        base.CollectObservations(sensor);

        var displacement = HelperFunctions.Sigmoid(drone.transform.position - goal.transform.position);
        var velocity = HelperFunctions.Sigmoid(droneControl.localVelocity);
        var angularVelocity = HelperFunctions.Sigmoid(droneControl.localAngularVelocity);
        var inclination = droneControl.inclination;

        sensor.AddObservation(displacement);
        sensor.AddObservation(velocity);
        sensor.AddObservation(angularVelocity);
        sensor.AddObservation(inclination);

       // Debug.Log("Displacement " + displacement.ToString());

    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        base.OnActionReceived(actions); 
       // droneControl.ApplyActions(actions.ContinuousActions.Array);

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

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var actions = droneControl.actions;
        /*        for (int i = 0; i < actions.Length; i++)
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

        // Rotation - Adjust rotational torque
        if (Input.GetKey(KeyCode.Q))
        {
            // To rotate left, increase thrust in clockwise rotors and decrease in counterclockwise rotors
            actions[0] += 0.05f; // Assuming rotor 0 spins clockwise
            actions[1] -= 0.025f; // Assuming rotor 1 spins counterclockwise
            actions[3] += 0.05f; // Assuming rotor 3 spins clockwise
            actions[2] -= 0.025f; // Assuming rotor 2 spins counterclockwise
        }
        else if (Input.GetKey(KeyCode.E))
        {
            // To rotate right, do the opposite
            actions[1] += 0.05f; // Assuming rotor 1 spins counterclockwise
            actions[0] -= 0.025f; // Assuming rotor 0 spins clockwise
            actions[2] += 0.05f; // Assuming rotor 2 spins counterclockwise
            actions[3] -= 0.025f; // Assuming rotor 3 spins clockwise
        }

        // Ascent and Descent
        if (Input.GetKey(KeyCode.UpArrow))
        {
            actions[0] = actions[1] = actions[2] = actions[3] = 1.2f; // Increase all rotors' thrust for ascent
        }
        else if (Input.GetKey(KeyCode.DownArrow))
        {
            actions[0] = actions[1] = actions[2] = actions[3] = 0.9f; // Decrease all rotors' thrust for descent
        }

        droneControl.ApplyActions(actions);
    }
}
