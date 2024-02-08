using System;
using Unity.VisualScripting;
using UnityEngine;

public class DroneControl : MonoBehaviour
{

    public event Action TipOverEvent;
    public event Action<Collision> CollisionEvent;
    public event Action CollisionTimeoutEvent;
    public event Action ReachedGoalEvent;
    public event Action LeftEnviromentEvent;

    public bool isColliding { get; private set; }

    private Vector3 centerOfMass;
    private Rigidbody droneRigidBody;
    private readonly float[] animationSpeeds = new float[4];
    private readonly float[] rotorTurnDirections = { 1, -1, -1, 1 };
    private const float tipOverThreshold = -0.5f;
    private int collisionCount = 0;
    // Timeout in secs for continuous collision.
    private const float timeout = 2;
    private float defaultTilt;

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
    [SerializeField, Tooltip("Action multiplier")]
    public float thrustFactor { get; private set; } = 6.38f; // defaulted to: 6.38f;

    // [SerializeField, Tooltip("Action multiplier")]
    private float torqueFactor = 1.27f;

    [SerializeField, Tooltip("Action multiplier")]
    private float animSpeedFactor = 4000;

    [SerializeField, Tooltip("Whether to animate the rotors")]
    private bool animateRotors = true;

    //[SerializeField]
    public float[] actions;
    public float[] thrusts { get; private set; } = new float[4];

    private Rotor[] rotors;

    [SerializeField] private float[] velocityArray; // Array to hold velocity for each rotor
    [SerializeField] public float smoothTime = 0.3f; // Example value, adjust based on your needs

    private bool reachedGoal = false;
    private bool hasCollided = false;
    private void Start()
    {
        Initialize();
    }

    public void Initialize()
    {
        defaultTilt = transform.localEulerAngles.x;
        droneRigidBody = GetComponent<Rigidbody>();

        rotors = GetComponentsInChildren<Rotor>();
/*        Debug.Log("Rotors: " + rotors.Length);
        velocityArray = new float[rotors.Length];

        for (int i = 0; i < rotors.Length; i++)
        {
            Debug.Log("Setting for rotor: " + i);
            velocityArray[i] = 0f; // Start with a velocity of 0
        }*/
        // uncomment for better center of mass
       foreach (Rotor rotor in rotors)
        {
            rotor.Initialize();
            //centerOfMass += transform.InverseTransformPoint(rotor.worldPosition);
        }
        /* 
         Logger.LogMessage("Rotors have been initialized!");
         Logger.LogMessage("Center of mass: " + centerOfMass);
         droneRigidBody.centerOfMass = centerOfMass;*/
    }

    public void ApplyActions(float[] actions)
    {
        Array.Copy(actions, this.actions, actions.Length);

        // Debug.Log("Applying actions: " + actions[0] + ", " + actions[1] + ", " + actions[2] + ", " + actions[3]);

        // For now, we'll use a simplified setup, all rotors are aligned with drone's y-axis.
        Vector3 thrustAxis = transform.up; // world
        Vector3 torqueAxis = Vector3.down; // local
        // Debug.Log("Pre actions: " + string.Join(", ", actions));

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
            //Debug.Log("Actions for rotor " + rotors[i].name + " action: " + actionWithDirection);

            droneRigidBody.AddRelativeTorque(actionWithDirection * torqueFactor * torqueAxis);

            // Buffer value for animation.
            animationSpeeds[i] = actionWithDirection;
        }
        // Debug.Log("Post actions: " + string.Join(", ", actions));

        if (transform.up.y < tipOverThreshold)
        {
            // Debug.LogWarning("Drone has been tipped over");
            TipOverEvent?.Invoke();
        }
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
                /*   Vector3 rotationSpeed = new Vector3(0, speed, 0);

                   Quaternion deltaRotation = Quaternion.Euler(rotationSpeed);
                   rotor.rb.MoveRotation(deltaRotation * rotor.rb.rotation);*/

                //rotor.rb.transform.Rotate(0, speed, 0, Space.Self);
                //rotor.propeller.transform.Rotate(0, speed, 0);
                //Debug.Log("Speed for rotor " + rotor.name + " speed: " + speed);
                //Debug.Log("RigidBody: " + droneRigidBody.transform.position.ToString());
            }
        }   
        /*        if (animateRotors)
                {
                    float maxSpeed = animSpeedFactor; // Assuming animSpeedFactor is your target speed
                    for (int i = 0; i < rotors.Length; i++)
                    {
                        Rotor rotor = rotors[i];

                        // Assume targetAngleArray and velocityArray are defined at class level
                        float targetAngle = rotor.currentAngle + animationSpeeds[i] * maxSpeed * Time.deltaTime;
                        Debug.Log("Velocity array" + velocityArray[i]);
                        float newAngle = Mathf.SmoothDampAngle(rotor.currentAngle, targetAngle, ref velocityArray[i], smoothTime);

                        rotor.rb.MoveRotation(Quaternion.Euler(0, newAngle, 0));

                        // Update current angle
                        rotor.currentAngle = newAngle;

                        // Optional: Logging
                        Debug.Log("RigidBody: " + droneRigidBody.transform.position.ToString());
                    }
                }*/
    }

    private void OnCollisionEnter(Collision collision)
    {
        Logger.LogMessage("Drone has collided!", true);
        collisionCount++;
        UpdatecollisionStatus();
        CollisionEvent?.Invoke(collision);
    }

    private void OnTriggerEnter(Collider other)
    {

        if (other.tag == "Goal" && !reachedGoal)
        {
            // Debug.LogError("Drone has reached goal!");
            ReachedGoalEvent?.Invoke();
            reachedGoal = true;
        }  
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.tag == "Goal" && reachedGoal)
        {
            // Debug.LogError("Drone has moved from the goal!");
            reachedGoal = false;
        }
        if (other.tag == "Enviroment")
        {
            LeftEnviromentEvent?.Invoke();
        }
    }

    private void OnCollisionExit(Collision collision)
    {

        // Debug.LogWarning("Not colliding anymore!");
        collisionCount--;
        UpdatecollisionStatus();
        
    }

    private void OnCollisionStay(Collision collision)
    {
        // Debug.Log("Colliding stay");
        CollisionEvent?.Invoke(collision);
    }


    private void UpdatecollisionStatus()
    {
        // Debug.Log("Collision status update");
        if (collisionCount < 0)
        {
            // Debug.LogWarning("Collision count < 0");
            collisionCount = 0;
        }
        // Debug.LogWarning("Collision count: " + collisionCount);

        bool tmp = isColliding;
        isColliding = collisionCount > 0;
        if (!isColliding)
        {
            hasCollided = false;
            // Debug.Log("Canceling all invokes!");
            CancelInvoke();
        } else if (!tmp)
        {
            hasCollided = true;
            // Debug.Log("Invoking collision stay!");
            Invoke(nameof(NotifyTimeout), timeout);
        }

    }

    private void NotifyTimeout()
    {
        // Debug.Log("Notify Timout invoked!");
        CollisionTimeoutEvent?.Invoke();
    }

    // World coordinates to drone's coordinates
    public Vector3 WorldToLocal(Vector3 vector)
    {
        return transform.InverseTransformVector(vector);
    }

    public void ResetDrone(Vector3 resetPosition)
    {
        BaseReset();

        droneRigidBody.position = resetPosition;
        droneRigidBody.rotation = Quaternion.Euler(0, 0, 0);
    }


    private void BaseReset()
    {

        isColliding = false;
        collisionCount = 0;

        Array.Clear(thrusts, 0, thrusts.Length);
        Array.Clear(actions, 0, thrusts.Length);
        droneRigidBody.angularVelocity = Vector3.zero;
        droneRigidBody.velocity = Vector3.zero;
    }
}

