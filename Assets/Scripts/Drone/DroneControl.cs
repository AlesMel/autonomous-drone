using DroneProject;
using System;
using System.Collections;
using System.Threading;
using Unity.MLAgents;
using Unity.Sentis.Layers;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.InputSystem;

public class DroneControl : MonoBehaviour
{
    public event Action TipOverEvent;

    public Rigidbody droneRigidBody;

    public float tipOverThreshold = -0.5f;
    
    public Vector3 worldPosition => transform.TransformPoint(centerOfMass);

    // Velocities
    public Vector3 worldVelocity => droneRigidBody.velocity;
    public Vector3 worldAngularVelocity => droneRigidBody.angularVelocity;
    public Vector3 localVelocity => WorldToLocal(droneRigidBody.velocity);
    public Vector3 localAngularVelocity => WorldToLocal(droneRigidBody.angularVelocity);
    public Vector3 inclination => new Vector3(transform.right.y, transform.up.y, transform.forward.y);
    public Vector3 worldForwardXZ => Vector3.ProjectOnPlane(transform.forward, Vector3.up).normalized;

    // Drone's rotation around y-axis
    public Quaternion rotationY => Quaternion.Euler(0, transform.eulerAngles.y, 0);

    public float thrustFactor { get; private set; } = 25; // 6.38f; // defaulted to: 6.38f;

    // [SerializeField, Tooltip("Action multiplier")]
    private float torqueFactor = 5f; // ;  // defaulted to: 1.27f or 5;

    [SerializeField, Tooltip("Action multiplier")]
    private float animSpeedFactor = 4000;

    [SerializeField, Tooltip("Whether to animate the rotors")]
    private bool animateRotors = true;

    [SerializeField]
    private Rotor[] rotors;

    [SerializeField] public float[] actions;
    public float[] thrusts { get; private set; } = new float[4];
    private readonly float[] animationSpeeds = new float[4];
    private readonly float[] rotorTurnDirections = { 1, 1, -1, -1 }; //{ 1, 1, -1, -1 };//{ 1, -1, -1, 1 };

    // for rewards and penalties
    [HideInInspector] public bool isInGoal = false;
    [HideInInspector] public bool isInEnviroment = true;
    [HideInInspector] public bool isTippedOver = false;
    [HideInInspector] public bool isCrashed = false;

    public bool isColliding { get; private set; }
    public bool m_ResetFlag;
    public bool justEnabled;

    private float m_DefTilt;
    private float m_DefaultRotation;
    private Vector3 centerOfMass;

    [Header("Default spawn position")]
    public Vector3 defaultPosition = Vector3.zero;
    public float defaultYaw = 0.0f;

    private void Awake()
    {
       Initialize();
    }

    public void Initialize()
    {
        droneRigidBody = GetComponent<Rigidbody>();

        m_DefTilt = 0.0f; // transform.localEulerAngles.x;

        foreach (Rotor rotor in rotors)
        {
            rotor.Initialize();
            centerOfMass += transform.InverseTransformPoint(rotor.worldPosition);
        }


        centerOfMass *= 0.25f;
        droneRigidBody.centerOfMass = centerOfMass;
        Academy.Instance.OnEnvironmentReset += EnvironmentReset;
    }

    private void OnDisable()
    {
        BaseReset();
    }

    private void OnEnable()
    {
        m_ResetFlag= false;
    }

    void EnvironmentReset()
    {
    }

    private float[] ClampActions(float[] current_actions)
    {
        float[] act = current_actions;
        act[0] = Mathf.Clamp(current_actions[0], -1, 1);
        act[1] = Mathf.Clamp(current_actions[1], -1, 1);
        act[2] = Mathf.Clamp(current_actions[2], -1, 1);
        act[3] = Mathf.Clamp(current_actions[3], -1, 1);
        return act;
    }

    public void ApplyActions(float[] actions)
    {
        if (m_ResetFlag)
        {
            m_ResetFlag = false;
            return;
        }

        // For now, we'll use a simplified setup, all rotors are aligned with drone's y-axis.
        Vector3 thrustAxis = transform.up; // world
        Vector3 torqueAxis = Vector3.down; // local

        Array.Copy(ClampActions(actions), actions, actions.Length);
        for (int i = 0; i < rotors.Length; i++)
        {
            // 0.5 is precisely needed for drone to hover, since the thrust was calculated the way that it would
            // convert actions range: -1/+1 to 0/+1
            actions[i] = (actions[i] + 1) * 0.5f; // This is not needed in NEAT since we adjust output by activation
            this.actions[i] = actions[i];
            var thrust = actions[i] * thrustFactor;
            // Thrust per rotor but applied to drone's centre of mass
            Vector3 force = thrust * thrustAxis;
            droneRigidBody.AddForceAtPosition(actions[i] * thrustFactor * thrustAxis, rotors[i].worldPosition);

            /*            Debug.DrawRay(rotors[i].worldPosition - Vector3.up * 0.1f, Vector3.up * 0.2f, Color.blue, duration: 0.1f);
                        Debug.DrawRay(rotors[i].worldPosition - Vector3.right * 0.1f, Vector3.right * 0.2f, Color.blue, duration: 0.1f);
                        Debug.Log("Rotors: " + rotors[i].worldPosition);*/

            // Debug.DrawRay(rotors[i].worldPosition, force.normalized * 5, Color.green);

            // Flip direction for 2 of 4 rotors, torques need to cancel each other out.
            actions[i] *= rotorTurnDirections[i];

            droneRigidBody.AddRelativeTorque(actions[i] * torqueFactor * torqueAxis);
            // Buffer value for animation.
            animationSpeeds[i] = actions[i];
        }

        if (transform.up.y < tipOverThreshold)
        {
            // Debug.LogWarning("Drone has been tipped over");
            TipOverEvent?.Invoke();
        }
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        PropellerAnimation();
        //ApplyActions(this.actions);

        //ApplyActions(daco);
    }

    public void PropellerAnimation()
    {
        if (animateRotors)
        {
            float maxSpeed = Time.fixedDeltaTime * animSpeedFactor;
            for (int i = 0; i < rotors.Length; i++)
            {
                //Rotor rotor = rotors[i];
                float speed = animationSpeeds[i] * maxSpeed;
                rotors[i].propeller.Rotate(0, speed, 0);

                //rotors[i].rb.transform.Rotate(Vector3.up, speed);
            }
        }
    }
    
    // World coordinates to drone's coordinates
    public Vector3 WorldToLocal(Vector3 vector)
    {
        return transform.InverseTransformVector(vector);
    }

    // treba lepsie vyriesit toto resetovanie a spustanie pretoze sa to nezhoduje v behoch!
    void ResetCheckingParameters()
    {
        isInGoal = false;
        isInEnviroment = true;
        isTippedOver = false;
        isCrashed = false;
    }

    void SetRigidBodyMaximumVelocities()
    {
        // droneRigidBody.maxLinearVelocity = maxLinearVelocity;
        droneRigidBody.maxAngularVelocity = 3;
    }

    void ResetPhysicalProperties()
    {
        Array.Clear(actions, 0, actions.Length);

        droneRigidBody.velocity = Vector3.zero;
        droneRigidBody.angularVelocity = Vector3.zero;

        droneRigidBody.rotation = Quaternion.Euler(m_DefTilt, defaultYaw, 0);
        droneRigidBody.position = defaultPosition;

        transform.position = defaultPosition;// Vector3.zero;
        transform.rotation = Quaternion.Euler(m_DefTilt, defaultYaw, 0);
    }

    public void BaseReset()
    {
        ResetCheckingParameters();
        ResetPhysicalProperties();
        m_ResetFlag = true;
    }
}

