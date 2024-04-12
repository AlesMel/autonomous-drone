using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Unity.Sentis.Layers;

[RequireComponent(typeof(DroneControl))]
public class CrawlerInspo : Agent
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
    public Transform TargetPrefab; //Target prefab to use in Dynamic envs
    private Transform m_Target; //Target the agent will walk towards during training.

    [Header("Body")] 
    public Transform body;

    //Because ragdolls can move erratically during training, using a stabilized reference transform improves learning
    OrientationCubeController m_OrientationCube;

    //The indicator graphic gameobject that points towards the target
    DirectionIndicator m_DirectionIndicator;
    DroneControl m_DroneControl;

    protected Bounds bounds;
    protected float maxCheckpointDistance = 30.0f;
    protected bool startCollision = true;
    private float penalty => 1.0f; //+ (MaxStep-StepCount);

    private new void Awake()
    {
        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();
        m_DroneControl = GetComponent<DroneControl>();
        bounds = new Bounds(m_DroneControl.transform.position, Vector3.one * maxCheckpointDistance);
    }

    public override void Initialize()
    {
        base.Initialize();
        SpawnTarget(TargetPrefab, transform.position);
    }

    void SpawnTarget(Transform prefab, Vector3 pos)
    {
        m_Target = Instantiate(prefab, pos, Quaternion.identity, transform.parent);
    }
    protected override void OnEnable()
    {
        base.OnEnable();
        // SpawnTarget(TargetPrefab, transform.position);
    }
    protected override void OnDisable()
    {
        base.OnDisable();
        if (transform.name == "DummyDrone")
        {
          Destroy(m_Target.gameObject);
        }
    }

    public override void OnEpisodeBegin()
    {
        base.OnEpisodeBegin();
        m_DroneControl.BaseReset();
        UpdateOrientationObjects();
        CancelInvoke();
        startCollision = true;
        TargetWalkingSpeed = Random.Range(0.1f, m_maxFlyingSpeed);
        m_Target.gameObject.GetComponent<TargetController>().MoveTargetToRandomPosition();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // linear is 1e+16, angular is max 7
        base.CollectObservations(sensor);
        /*var cubeForward = m_OrientationCube.transform.forward;
        var velocityGoal = cubeForward * TargetWalkingSpeed;
        var currentVelocity = m_DroneControl.droneRigidBody.velocity;
        var currentAngularVelocity = m_DroneControl.droneRigidBody.angularVelocity;
        Quaternion rotation = Quaternion.FromToRotation(body.forward, cubeForward);
        Vector3 normalizedRotation = rotation.eulerAngles / 180.0f - Vector3.one;  // [-1,1]

        sensor.AddObservation(HelperFunctions.Sigmoid(Vector3.Distance(velocityGoal, currentVelocity), 0.5f));
        sensor.AddObservation(HelperFunctions.Sigmoid(m_OrientationCube.transform.InverseTransformDirection(currentVelocity), 0.5f));
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(currentAngularVelocity) / m_DroneControl.droneRigidBody.maxAngularVelocity);
        sensor.AddObservation(HelperFunctions.Sigmoid(m_OrientationCube.transform.InverseTransformDirection(velocityGoal), 0.5f));
        sensor.AddObservation(normalizedRotation);
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(m_Target.transform.position) / maxCheckpointDistance);*/
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

        int actionsNotHovering = 0;
        float sumActions = 0f;
        float sumSquaredDiffs = 0f;

/*        for (int i = 0; i < m_DroneControl.actions.Length; i++)
        {
            // Only add reward if action[i] is not equal to 0.5
            if (m_DroneControl.actions[i] == 0.5f)
            {
                actionsNotHovering++;
            }
            sumActions += m_DroneControl.actions[i];

        }
        if (actionsNotHovering < 4)
        {
            AddReward(velocityReward * lookAtTargetReward * matchStabilityReward);
        }*/
        AddReward(matchSpeedReward * lookAtTargetReward * matchStabilityReward);

        // for NEAT only
        /* float meanActions = sumActions / m_DroneControl.actions.Length;

         for (int i = 0; i < m_DroneControl.actions.Length; i++)
         {
             sumSquaredDiffs += Mathf.Pow(m_DroneControl.actions[i] - meanActions, 2);
         }
         float variance = sumSquaredDiffs / m_DroneControl.actions.Length;

         // Apply variance penalty
         if (variance > 0.1f) // Adjust the threshold as necessary
         {
             AddReward(-variance);
         }*/
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

        AddReward(1f * matchStabilityReward);
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
        if (col.transform.CompareTag("Ground"))
        {
/*            if (startCollision)
            {
                Invoke(nameof(NotifyTimeout), 1.0f);
            }
            else
            {*/
                AddReward(-penalty);
                EndEpisode();
            //}
        }
    }
    private void OnTriggerEnter(Collider col)
    {
        if (col.gameObject == m_Target.gameObject)
        {
            TouchedTarget();
            m_Target.gameObject.GetComponent<TargetController>().MoveTargetToRandomPosition();
        }
    }
    private void OnTriggerStay(Collider col)
    {
        if (col.gameObject == m_Target.gameObject)
        {
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
