using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Unity.Sentis.Layers;

[RequireComponent(typeof(DroneControl))]
public class ForestAgent : Agent
{

    private string k_Target = "target";
    [Range(0.1f, m_maxFlyingSpeed)]
    [SerializeField]
    private float m_TargetFlyingSpeed = m_maxFlyingSpeed;

    const float m_maxFlyingSpeed = 15.0f;
    //The current target walking speed. Clamped because a value of zero will cause NaNs
    public float TargetWalkingSpeed
    {
        get { return m_TargetFlyingSpeed; }
        set { m_TargetFlyingSpeed = Mathf.Clamp(value, .1f, m_maxFlyingSpeed); }
    }
    //The direction an agent will walk during training.
    [Header("Target To Fly Towards")]
    [SerializeField] public Transform[] m_Targets; //Target the agent will walk towards during training.
    private Transform m_Target; // current target

    [Header("Body")] 
    public Transform body;

    //Because ragdolls can move erratically during training, using a stabilized reference transform improves learning
    OrientationCubeController m_OrientationCube;

    //The indicator graphic gameobject that points towards the target
    DirectionIndicator m_DirectionIndicator;
    DroneControl m_DroneControl;

    protected Bounds bounds;
    protected float maxCheckpointDistance = 50.0f;
    protected bool startCollision = true;
    private bool checkpointActive = true;

    private float reward => 100.0f;
    private float penalty => 1.0f;

    private int chptIndex = 0;
    private RaySensor m_RayDetection;

    private new void Awake()
    {
        m_Target = m_Targets[0];
        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();
        m_DroneControl = GetComponent<DroneControl>();
        bounds = new Bounds(m_DroneControl.transform.position, Vector3.one * maxCheckpointDistance);
        m_RayDetection = GetComponentInChildren<RaySensor>();
    }

    public override void Initialize()
    {
        base.Initialize();
    }

    protected override void OnEnable()
    {
        base.OnEnable();
    }

    protected override void OnDisable()
    {
        base.OnDisable();
    }

    public override void OnEpisodeBegin()
    {
        base.OnEpisodeBegin();
        CancelInvoke();
        m_DroneControl.BaseReset();
        chptIndex = 0;
        UpdateOrientationObjects();
        TargetWalkingSpeed = 2; // Can be also random: Random.Range(0.1f, m_maxFlyingSpeed); 
        m_Target = m_Targets[0];
        m_RayDetection.Clear();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // linear is 1e+16, angular is max 7
        base.CollectObservations(sensor);
        var cubeForward = m_OrientationCube.transform.forward;
        var velocityGoal = cubeForward * TargetWalkingSpeed;
        Quaternion rotation = Quaternion.FromToRotation(body.forward, cubeForward);

        sensor.AddObservation(rotation);
        sensor.AddObservation(HelperFunctions.Sigmoid(velocityGoal, 0.5f));
        // sensor.AddObservation((transform.position - m_Target.position) / maxCheckpointDistance); // NEAT
        sensor.AddObservation(HelperFunctions.Sigmoid(m_DroneControl.localVelocity, 0.5f));
        sensor.AddObservation(HelperFunctions.Sigmoid(m_DroneControl.localAngularVelocity));
        sensor.AddObservation(m_DroneControl.inclination);
        m_RayDetection.AddDistances(sensor);
    }

    private bool IsNotHovering()
    {
        int actionsNotHovering = 0;

        for (int i = 0; i < m_DroneControl.actions.Length; i++)
        {
            // Only add reward if action[i] is not equal to 0.5
            if (m_DroneControl.actions[i] != 0.5f && m_DroneControl.actions[i] != 0.0f && m_DroneControl.actions[i] != 1.0f) // 
            {
                actionsNotHovering++;
            }
        }

        return actionsNotHovering == 4;
    }
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        base.OnActionReceived(actionBuffers);
        var continuousActions = actionBuffers.ContinuousActions;

        if (m_DroneControl.transform.up.y < m_DroneControl.tipOverThreshold)
        {
            AddReward(-penalty);
            EndEpisode();
        }

        if (!bounds.Contains(m_DroneControl.transform.position))
        {
            AddReward(-penalty);
            EndEpisode();
        }

        m_RayDetection.BatchRaycast();
        m_DroneControl.ApplyActions(continuousActions.Array);

    }

    private void FixedUpdate()
    {
        UpdateOrientationObjects();
        var cubeForward = m_OrientationCube.transform.forward;
        var velocityGoal = cubeForward * TargetWalkingSpeed;
        var currentVelocity = m_DroneControl.droneRigidBody.velocity;

        float m_VelocityError = Vector3.Distance(velocityGoal, currentVelocity);
        var matchSpeedReward = GetMatchingVelocityReward(cubeForward * TargetWalkingSpeed, m_DroneControl.droneRigidBody.velocity, TargetWalkingSpeed);
        var matchStabilityReward = HelperFunctions.Reward(m_DroneControl.droneRigidBody.angularVelocity.magnitude, 1f); // default 3f
        var velocityReward = HelperFunctions.Reward(m_VelocityError, 10.0f);
        var lookAtTargetReward = Mathf.Max(0.0f, (Vector3.Dot(cubeForward, body.forward) + 1) * .5F);
        var positionReward = Mathf.Max(0.0f, HelperFunctions.Reward(Vector3.Distance(m_Target.transform.position, m_OrientationCube.transform.position), 1f));


        if (IsNotHovering())
        {
            AddReward(matchSpeedReward * lookAtTargetReward * matchStabilityReward);
        }
    }

    public float GetMatchingVelocityReward(Vector3 velocityGoal, Vector3 actualVelocity, float target)
    {
        //distance between our actual velocity and goal velocity
        var velDeltaMagnitude = Mathf.Clamp(Vector3.Distance(actualVelocity, velocityGoal), 0, target);

        //return the value on a declining sigmoid shaped curve that decays from 1 to 0
        //This reward will approach 1 if it matches perfectly and approach zero as it deviates
        return Mathf.Pow(1 - Mathf.Pow(velDeltaMagnitude / target, 2), 2);
    }

    void UpdateOrientationObjects()
    {
        m_OrientationCube.UpdateOrientation(body, m_Target);
        if (m_DirectionIndicator)
        {
            m_DirectionIndicator.MatchOrientation(m_OrientationCube.transform);
        }
    }

    public void TouchedTarget()
    {
        var matchStabilityReward = HelperFunctions.Reward(m_DroneControl.droneRigidBody.angularVelocity.magnitude, 1f);

        // AddReward(1f * matchStabilityReward);

        chptIndex = (chptIndex+1) % m_Targets.Length;
        
        m_Target = m_Targets[chptIndex];
        if (IsNotHovering())
        {
            AddReward(reward * matchStabilityReward);
        }
        Debug.Log("Next target!" + chptIndex);
    }



    void OnCollisionEnter(Collision col)
    {
        if (col.transform.CompareTag("Environment"))
        {
            AddReward(-penalty);
            EndEpisode();
        }
    }

    private void OnCollisionStay(Collision col)
    {
        if (col.transform.CompareTag("Environment"))
        {
            AddReward(-penalty);
            EndEpisode();
        }
    }

    private void OnTriggerEnter(Collider col)
    {
        if (col.gameObject == m_Target.gameObject)
        {
            TouchedTarget();
        }
    }
    private void OnTriggerStay(Collider col)
    {
        if (col.gameObject == m_Target.gameObject)
        {
        }
    }
    private void OnTriggerExit(Collider col)
    {
        if (col.gameObject == m_Target.gameObject)
        {
            checkpointActive = true;
        }
    }
    private void OnDrawGizmos()
    {
       // Gizmos.DrawLine(transform.position, m_Target.position);
    }
}
