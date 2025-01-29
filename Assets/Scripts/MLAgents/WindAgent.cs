using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Unity.Sentis.Layers;

[RequireComponent(typeof(DroneControl))]
public class WindAgent : Agent
{

    DroneControl m_DroneControl;

    private float penalty = 0.1f;

    private float reward = 1f;
    [SerializeField]
    public Transform m_Target;

    private Bounds bounds;
    private float maxDistance = 10.0f;

    private new void Awake()
    {
        m_DroneControl = GetComponent<DroneControl>();
        bounds = new Bounds(m_DroneControl.transform.position, Vector3.one * maxDistance);
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
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // linear is 1e+16, angular is max 7
        base.CollectObservations(sensor);
        sensor.AddObservation((transform.position - m_Target.position)/maxDistance);
        sensor.AddObservation(m_DroneControl.localAngularVelocity/m_DroneControl.droneRigidBody.maxAngularVelocity); //HelperFunctions.Sigmoid(m_DroneControl.localAngularVelocity)
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

        m_DroneControl.ApplyActions(continuousActions.Array);
    }
    private bool IsNotHovering()
    {
        int actionsNotHovering = 0;

        for (int i = 0; i < m_DroneControl.actions.Length; i++)
        {
            // Only add reward if action[i] is not equal to 0.5
            if (m_DroneControl.actions[i] != 0.5f && m_DroneControl.actions[i] != 0.0f && m_DroneControl.actions[i] != 1.0f)
            {
                actionsNotHovering++;
            }
        }

        return actionsNotHovering == 4;
    }
    private void FixedUpdate()
    {
        if (!bounds.Contains(m_DroneControl.transform.position))
        {
            AddReward(-penalty);
            EndEpisode();
        }
        var distanceToTarget = Vector3.Distance(transform.position, m_Target.position);
        var matchStabilityReward = HelperFunctions.Reward(m_DroneControl.droneRigidBody.angularVelocity.magnitude, 1f);
        var positionReward = HelperFunctions.Reward(distanceToTarget, 1f);

        float angularVelocityPenalty = -m_DroneControl.droneRigidBody.angularVelocity.magnitude;

        if (IsNotHovering())
        {
            AddReward(matchStabilityReward * positionReward);
        }
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
  

    private void OnTriggerStay(Collider other)
    {

        if (other.CompareTag("target") && IsNotHovering())
        {
            var matchStabilityReward = HelperFunctions.Reward(m_DroneControl.droneRigidBody.angularVelocity.magnitude, 5f);
            var bouncingVelocity = HelperFunctions.Reward(Mathf.Abs(m_DroneControl.localVelocity.y), 10f); // bouncing on y axis
            AddReward(reward * matchStabilityReward * bouncingVelocity);
        }
    }

    private void OnDrawGizmos()
    {
        Gizmos.DrawLine(transform.position, m_Target.position);
    }
}
