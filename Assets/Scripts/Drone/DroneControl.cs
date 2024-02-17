using DroneProject;
using System;
using System.Collections;
using Unity.MLAgents;
using Unity.Sentis.Layers;
using Unity.VisualScripting;
using UnityEngine;

public class DroneControl : MonoBehaviour
{

    public event Action TipOverEvent;
    public event Action<Collision> CollisionEvent;
    public event Action CollisionTimeoutEvent;
    public event Action ReachedGoalEvent;
    public event Action LeftEnviromentEvent;
    public event Action LowAltitudeEvent;

    public bool isColliding { get; private set; }

    private Vector3 centerOfMass;
    private Rigidbody droneRigidBody;
    private readonly float[] animationSpeeds = new float[4];
    private readonly float[] rotorTurnDirections = { 1, -1, -1, 1 };
    public float tipOverThreshold = -0.5f;
    
    [SerializeField] private Vector3 defaultPosition;
    public Vector3 worldPosition => transform.TransformPoint(centerOfMass);

    // Velocities
    public Vector3 worldVelocity => droneRigidBody.velocity;
    public Vector3 worldAngularVelocity => droneRigidBody.angularVelocity;
    public Vector3 localVelocity => WorldToLocal(droneRigidBody.velocity);
    public Vector3 localAngularVelocity => WorldToLocal(droneRigidBody.velocity);
    public Vector3 inclination => new Vector3(transform.right.y, transform.up.y, transform.forward.y);
    public Vector3 worldForwardXZ => Vector3.ProjectOnPlane(transform.forward, Vector3.up).normalized;

    // Drone's rotation around y-axis
    public Quaternion rotationY => Quaternion.Euler(0, transform.eulerAngles.y, 0);

    // Multipliers.
    // [SerializeField, Tooltip("Action multiplier")]
    public float thrustFactor { get; private set; } = 6.38f; // defaulted to: 6.38f;

    // [SerializeField, Tooltip("Action multiplier")]
    private float torqueFactor = 1.27f;  // defaulted to: 1.27f;

    [SerializeField, Tooltip("Action multiplier")]
    private float animSpeedFactor = 4000;

    [SerializeField, Tooltip("Whether to animate the rotors")]
    private bool animateRotors = true;

    [SerializeField]
    private bool printIsColliding = false;

    [SerializeField] public float[] actions;
    public float[] thrusts { get; private set; } = new float[4];

    private Rotor[] rotors;

    [SerializeField] private float[] velocityArray; // Array to hold velocity for each rotor
    [SerializeField] public float smoothTime = 0.3f; // Example value, adjust based on your needs

    private bool isCoroutineRunning = false; // To ensure the coroutine runs only once per collision

    // for rewards and penalties
    public bool isInGoal = false;
    public bool isInEnviroment = true;
    public bool hasCrashed = true;

    public float collisionTimeout = 3.5f;
    public bool collisionStay = false;
    public int collisionCount = 0;

    private void Awake()
    {
        Initialize();
        Academy.Instance.OnEnvironmentReset += EnvironmentReset;
    }

    public void Initialize()
    {
        droneRigidBody = GetComponent<Rigidbody>();
        defaultPosition = droneRigidBody.position;

        rotors = GetComponentsInChildren<Rotor>();

        foreach (Rotor rotor in rotors)
        {
            rotor.Initialize();
            centerOfMass += transform.InverseTransformPoint(rotor.worldPosition);
        }

        droneRigidBody.centerOfMass = centerOfMass;
        BaseReset();
    }

    public void ApplyActions(float[] actions)
    {
        Array.Copy(actions, this.actions, actions.Length);

        // For now, we'll use a simplified setup, all rotors are aligned with drone's y-axis.
        Vector3 thrustAxis = transform.up; // world
        Vector3 torqueAxis = Vector3.down; // local

        for (int i = 0; i < rotors.Length; i++)
        {
            // 0.5 is precisely needed for drone to hover, since the thrust was calculated the way that it would
            // convert actions range: -1/+1 to 0/+1
            // actions[i] = (actions[i] + 1) * 0.5f; // This is not needed in NEAT since we adjust output by activation
            thrusts[i] = actions[i] * thrustFactor;

            // Thrust per rotor but applied to drone's centre of mass
            Vector3 force = thrusts[i] * thrustAxis;
            droneRigidBody.AddForceAtPosition(force, rotors[i].worldPosition);
            Debug.DrawRay(rotors[i].worldPosition, force.normalized * 5, Color.green);

            // Flip direction for 2 of 4 rotors, torques need to cancel each other out.
            float actionWithDirection = actions[i] * rotorTurnDirections[i];

            droneRigidBody.AddRelativeTorque(actionWithDirection * torqueFactor * torqueAxis);

            // Buffer value for animation.
            animationSpeeds[i] = actionWithDirection;
        }

/*        if (transform.up.y < tipOverThreshold)
        {
            TipOverEvent?.Invoke();
        }*/
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        Debug.DrawRay(worldPosition, Vector3.up * transform.up.y, Color.green);
        Debug.DrawRay(worldPosition, inclination.normalized * 5, Color.red);

        PropellerAnimation();
    }

    public void PropellerAnimation()
    {
        if (animateRotors)
        {
            float maxSpeed = Time.fixedDeltaTime * animSpeedFactor;
            for (int i = 0; i < rotors.Length; i++)
            {
                Rotor rotor = rotors[i];
                float speed = animationSpeeds[i] * maxSpeed;

                rotor.rb.transform.Rotate(Vector3.up, speed);
            }
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        Logger.LogMessage("Drone has collided!", true, false);
        if (collision.gameObject.CompareTag("Ground"))
        {
            Logger.LogMessage("Drone ground collided!", true, false);
            collisionStay = true;
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Goal"))
        {
            Logger.LogMessage("Drone has touched the goal!", true);
            // ReachedGoalEvent?.Invoke();
            isInGoal = true;
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Goal"))
        {
            isInGoal = false;
        }
        if (other.CompareTag("Enviroment"))
        {
            isInEnviroment = false;
        }
    }

    private void OnTriggerStay(Collider other)
    {
    }

    private void OnCollisionExit(Collision collision)
    {
        if (collision.gameObject.CompareTag("Ground"))
        {
            collisionStay = false;
            collisionCount = 0;
        }
        isCoroutineRunning = false;
        StopAllCoroutines();
    }

    private void OnCollisionStay(Collision collision)
    {
        if (collision.gameObject.CompareTag("Ground"))
        {
            collisionStay = true;
        }

/*        if (collision.gameObject.CompareTag("Ground"))
        {
            collisionCount++;
       *//*     if (!isCoroutineRunning)
            {
                // Start the coroutine
                StartCoroutine(CheckCollisionDuration());
            }*//*
        }*/
    }

    IEnumerator CheckCollisionDuration()
    {
        isCoroutineRunning = true; // Mark the coroutine as running

        // Wait for 1 second
        yield return new WaitForSeconds(collisionTimeout);

        // After 1 second, check if still colliding
        if (isCoroutineRunning)
        {
            // Flip the variable here
            hasCrashed = true;
        }

        // Mark the coroutine as not running
        isCoroutineRunning = false;
    }

    // World coordinates to drone's coordinates
    public Vector3 WorldToLocal(Vector3 vector)
    {
        return transform.InverseTransformVector(vector);
    }

    void EnvironmentReset()
    {
        BaseReset();
    }

    void ResetCheckingParameters()
    {
        isInGoal = false;
        isInEnviroment = true;
        isCoroutineRunning = false;
        hasCrashed = false;
        collisionCount = 0;
        collisionStay = false;
    }

    void ResetPhysicalProperties()
    {
        droneRigidBody.velocity = Vector3.zero;
        droneRigidBody.angularVelocity = Vector3.zero;
        droneRigidBody.position = droneRigidBody.transform.parent.TransformPoint(Vector3.zero);
        droneRigidBody.rotation = Quaternion.Euler(0, 0, 0);
    }

    public void BaseReset()
    {
        ResetPhysicalProperties();
        ResetCheckingParameters();
        Array.Clear(actions, 0, actions.Length);
    }
}

