using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Unity.Sentis.Layers;

[RequireComponent(typeof(DroneControl))]
public class CaveAgent : Agent
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

    private float reward => 5.0f;
    private float penalty => 1.0f;
    private Vector3 firstCheckpointPosition = new Vector3(5f, 3f, 5f);
    private Vector3 secondCheckpointPosition = new Vector3(-3f, 4f, 2f);
    private Vector3 thirdCheckpointPosition = new Vector3(-2f, 2f, -2f);
    private Vector3[] checkpoints;
    private int chptIndex = 0;
    private float previousDistance = 0.0f;
    private Vector3 lastPosition = Vector3.zero;

    private new void Awake()
    {
        m_Target = m_Targets[0];
        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();
        m_DroneControl = GetComponent<DroneControl>();
        bounds = new Bounds(m_DroneControl.transform.position, Vector3.one * maxCheckpointDistance);
        checkpoints = new Vector3[] { firstCheckpointPosition, secondCheckpointPosition, thirdCheckpointPosition };
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
        TargetWalkingSpeed = 2; // Random.Range(0.1f, m_maxFlyingSpeed); 
        m_Target = m_Targets[0];
        lastPosition = m_DroneControl.defaultPosition;
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
        sensor.AddObservation(HelperFunctions.Sigmoid(m_DroneControl.localVelocity, 0.5f));
        sensor.AddObservation(HelperFunctions.Sigmoid(m_DroneControl.localAngularVelocity));
        sensor.AddObservation(m_DroneControl.inclination);
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
        var matchStabilityReward = HelperFunctions.Reward(m_DroneControl.droneRigidBody.angularVelocity.magnitude, 3f);
        var velocityReward = HelperFunctions.Reward(m_VelocityError, 10.0f);
        var lookAtTargetReward = (Vector3.Dot(cubeForward, body.forward) + 1) * .5F;
        var positionReward = HelperFunctions.Reward(Vector3.Distance(m_Target.transform.position, m_OrientationCube.transform.position), 1.0f);
        int actionsNotHovering = 0;

        for (int i = 0; i < m_DroneControl.actions.Length; i++)
        {
            // Only add reward if action[i] is not equal to 0.5
            if (m_DroneControl.actions[i] == 0.5f)
            {
                actionsNotHovering++;
            }
        }
        float currentDistance = Vector3.Distance(m_Target.position, lastPosition);
        float distanceGain = currentDistance - previousDistance;
        lastPosition = m_Target.position;
        if (actionsNotHovering < 1)
        {
            // AddReward(matchSpeedReward * lookAtTargetReward * matchStabilityReward);
            // AddReward(matchSpeedReward + lookAtTargetReward + matchStabilityReward);
            AddReward(distanceGain);
            // AddReward for surviving and not hovering
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
        var matchStabilityReward = HelperFunctions.Reward(m_DroneControl.droneRigidBody.angularVelocity.magnitude, 3f);

        // AddReward(1f * matchStabilityReward);
        chptIndex = (chptIndex+1) % m_Targets.Length;
        m_Target = m_Targets[chptIndex];
        AddReward(reward * matchStabilityReward);
        Debug.Log("Next target!");
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        base.Heuristic(actionsOut);
        var actions = actionsOut.ContinuousActions;

        float thrust = m_DroneControl.worldVelocity.y * -0.25f;
        Vector3 incl = m_DroneControl.inclination;
        float pitch = incl.z * -0.025f;
        float roll = incl.x * 0.025f;
        actions[0] = thrust + roll + pitch;
        actions[1] = thrust - roll - pitch;
        actions[2] = thrust - roll + pitch;
        actions[3] = thrust + roll - pitch;
        //actions[i] = (actions[i] + 1) * 0.5f;
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

    private void OnCollisionExit(Collision collision)
    {
        if (startCollision)
        {
            startCollision = false;
        }
        CancelInvoke();
    }

    private void NotifyTimeout()
    {
        SetReward(-1f);
        EndEpisode();
    }
}
