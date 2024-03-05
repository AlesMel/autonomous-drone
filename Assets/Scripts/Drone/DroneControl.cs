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
    private Vector3 centerOfMass;
    public Rigidbody droneRigidBody;

    private readonly float[] animationSpeeds = new float[4];
    private readonly float[] rotorTurnDirections = { 1, 1, -1, -1 }; //{ 1, 1, -1, -1 };//{ 1, -1, -1, 1 };
    public float tipOverThreshold = -0.5f;
    
    private Vector3 defaultPosition;

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

    public float maxLinearVelocity = 16.0f;
    public float maxAngularVelocity = 2.26f;
    public float thrustFactor { get; private set; } = 6.38f; // defaulted to: 6.38f;
        
    // [SerializeField, Tooltip("Action multiplier")]
    private float torqueFactor = 1.27f;  // defaulted to: 1.27f;

    [SerializeField, Tooltip("Action multiplier")]
    private float animSpeedFactor = 4000;

    [SerializeField, Tooltip("Whether to animate the rotors")]
    private bool animateRotors = true;

    [SerializeField] public float[] actions;
    public float[] thrusts { get; private set; } = new float[4];

    [SerializeField]
    private Rotor[] rotors;

    // for rewards and penalties
    [HideInInspector] public bool isInGoal = false;
    [HideInInspector] public bool isInEnviroment = true;
    [HideInInspector] public bool isTippedOver = false;
    [HideInInspector] public bool isCrashed = false;

    public bool isColliding { get; private set; }
    public bool m_ResetFlag;
    public bool justEnabled;

    private float m_DefTilt;

    private void Awake()
    {
       Initialize();
    }

    public void Initialize()
    {
        droneRigidBody = GetComponent<Rigidbody>();
        defaultPosition = transform.localPosition;
        m_DefTilt = transform.localEulerAngles.x;

        foreach (Rotor rotor in rotors)
        {
            rotor.Initialize();
            centerOfMass += transform.InverseTransformPoint(rotor.worldPosition);
        }

        //droneRigidBody.maxLinearVelocity = 15.0f;
        //droneRigidBody.maxAngularVelocity = 4.36f;
        //centerOfMass *= 0.25f;
        //droneRigidBody.centerOfMass = centerOfMass;
        Academy.Instance.OnEnvironmentReset += EnvironmentReset;
    }

    private void OnDisable()
    {

    }

    private void OnEnable()
    {
        m_ResetFlag= false;
    }

    void EnvironmentReset()
    {
    }

    public void ApplyActions(float[] current_actions)
    {
        //Array.Copy(actions, this.actions, actions.Length);

        // For now, we'll use a simplified setup, all rotors are aligned with drone's y-axis.
        Vector3 thrustAxis = transform.up; // world
        Vector3 torqueAxis = Vector3.down; // local

        for (int i = 0; i < rotors.Length; i++)
        {
            // 0.5 is precisely needed for drone to hover, since the thrust was calculated the way that it would
            // convert actions range: -1/+1 to 0/+1
            actions[i] = (current_actions[i] + 1) * 0.5f; // This is not needed in NEAT since we adjust output by activation
            thrusts[i] = actions[i] * thrustFactor;
            // Thrust per rotor but applied to drone's centre of mass
            Vector3 force = thrusts[i] * thrustAxis;
            droneRigidBody.AddForceAtPosition(force, rotors[i].worldPosition);
            // Debug.DrawRay(rotors[i].worldPosition, force.normalized * 5, Color.green);

            // Flip direction for 2 of 4 rotors, torques need to cancel each other out.
            float actionWithDirection = actions[i] * rotorTurnDirections[i];

            droneRigidBody.AddRelativeTorque(actionWithDirection * torqueFactor * torqueAxis);

            // Buffer value for animation.
            animationSpeeds[i] = actionWithDirection;
        }
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        PropellerAnimation();
        float[] daco = { 0, 0, 0, 0 };
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
                /*
                rotor.rb.transform.Rotate(Vector3.up, speed);*/
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

    void ResetPhysicalProperties()
    {
        droneRigidBody.velocity = Vector3.zero;
        droneRigidBody.angularVelocity = Vector3.zero;

        droneRigidBody.rotation = Quaternion.Euler(0, 0, 0);
        droneRigidBody.position = defaultPosition;

        transform.position = defaultPosition;// Vector3.zero;
        transform.rotation = Quaternion.Euler(0, 0, 0);
        /*droneRigidBody.position = droneRigidBody.transform.parent.TransformPoint(defaultPosition);
        droneRigidBody.rotation = Quaternion.Euler(0, 0, 0);*/
    }

    public void BaseReset()
    {
        ResetCheckingParameters();
        ResetPhysicalProperties();
        m_ResetFlag = true;
    }
}

