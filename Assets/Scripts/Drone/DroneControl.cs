using UnityEngine;

public class DroneControl : MonoBehaviour
{
    private Vector3 centerOfMass;
    private Rigidbody droneRigidBody;
    private readonly float[] animationSpeeds = new float[4];
    private readonly float[] rotorTurnDirections = { 1, 1, -1, -1 };

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

    [Space]
    // Multipliers.
    [SerializeField, Tooltip("Action multiplier")]
    private float thrustFactor = 25;

    [SerializeField, Tooltip("Action multiplier")]
    private float torqueFactor = 5;

    [SerializeField, Tooltip("Action multiplier")]
    private float animSpeedFactor = 4000;

    [SerializeField, Tooltip("Whether to animate the rotors")]
    private bool animateRotors = true;

    [SerializeField] public float[] actions;

    private Rotor[] rotors;

    [SerializeField] private float[] velocityArray; // Array to hold velocity for each rotor
    [SerializeField] public float smoothTime = 0.3f; // Example value, adjust based on your needs

    private void Start()
    {
        Initialize();
    }

    public void Initialize()
    {
        droneRigidBody = GetComponent<Rigidbody>();

        rotors = GetComponentsInChildren<Rotor>();
/*        Debug.Log("Rotors: " + rotors.Length);
        velocityArray = new float[rotors.Length];

        for (int i = 0; i < rotors.Length; i++)
        {
            Debug.Log("Setting for rotor: " + i);
            velocityArray[i] = 0f; // Start with a velocity of 0
        }*/

        foreach (Rotor rotor in rotors)
        {
            rotor.Initialize();
            centerOfMass += transform.InverseTransformPoint(rotor.worldPosition);

        }



        Debug.Log("Rotors have been initialized!");
        centerOfMass *= 0.25f; //?
        //centerOfMass = new Vector3(0f, 0f, 0f);
        Debug.Log("Center of mass: " + centerOfMass);
        droneRigidBody.centerOfMass = centerOfMass;
    }

    public void ApplyActions()
    {
        // For now, we'll use a simplified setup, all rotors are aligned with drone's y-axis.
        Vector3 thrustAxis = transform.up; // world
        Vector3 torqueAxis = Vector3.down; // local

        for (int i = 0; i < rotors.Length; i++)
        {
            // -1/+1 => 0/+1
            //actions[i] = (actions[i] + 1) * 0.5f;

            // Thrust per rotor but applied to drone's centre of mass
            Vector3 force = actions[i] * thrustFactor * thrustAxis;
            droneRigidBody.AddForceAtPosition(force, rotors[i].worldPosition);
            Debug.DrawRay(rotors[i].worldPosition, force.normalized * 5, Color.green);

            // Flip direction for 2 of 4 rotors, torques need to cancel each other out.
            float actionWithDirection = actions[i] * rotorTurnDirections[i];
            //Debug.Log("Actions for rotor " + rotors[i].name + " action: " + actionWithDirection);

            droneRigidBody.AddRelativeTorque(actionWithDirection * torqueFactor * torqueAxis);

            // Buffer value for animation.
            animationSpeeds[i] = actionWithDirection;

        }
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        Debug.DrawRay(worldPosition, inclination.normalized * 5, Color.red);

        ApplyActions();
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
                Debug.Log("RigidBody: " + droneRigidBody.transform.position.ToString());
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

    // World coordinates to drone's coordinates
    public Vector3 WorldToLocal(Vector3 vector)
    {
        return transform.InverseTransformVector(vector);
    }
}

