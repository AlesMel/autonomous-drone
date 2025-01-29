using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System;

namespace DroneProject
{
    /// <summary>
    /// Controls drone physics based on specified target values.
    /// </summary>
    public class RotorControlAg : Agent
    {
        public DroneControl Drone { get; private set; }
        public int DecisionInterval { get; set; }

        // Buffer for action interpolation.
        private float[] m_PrevActions;
        private const int k_NumActions = 4;

        // Local target velocity (model was trained for -10/+10 range).
        private Vector3 m_LocalVelocity;
        // Normalized local target look angle on XZ-plane.
        protected float LocalLookAngle;

        /// <inheritdoc />
        public override void Initialize()
        {
            // TODO Code execution order unclear when
            // Initialize() is invoked via PilotAgent.
            if (Drone != null) return;

            m_PrevActions = new float[k_NumActions];
            Drone = GetComponentInChildren<DroneControl>();
            Drone.Initialize();
        }

        /// <inheritdoc />
        public override void OnEpisodeBegin()
        {
            Array.Clear(m_PrevActions, 0, k_NumActions);
        }

        /// <inheritdoc />
        public override void CollectObservations(VectorSensor sensor)
        {
            // TBD Sigmoid coefficients.
            sensor.AddObservation(m_LocalVelocity);
            sensor.AddObservation(Drone.localVelocity);
            sensor.AddObservation(Drone.localAngularVelocity);
            sensor.AddObservation(Drone.inclination);
        }

        /// <inheritdoc />
        public override void OnActionReceived(ActionBuffers actionBuffers)
        {
            var actions = actionBuffers.ContinuousActions.Array;
            Drone.ApplyActions(actions);
        }

        /// <inheritdoc />
        public override void Heuristic(in ActionBuffers actionsOut)
        {
            var actions = actionsOut.ContinuousActions;
            
            // Hover, TBD.
            float thrust = Drone.worldVelocity.y * -0.25f;
            Vector3 incl = Drone.inclination;
            float pitch = incl.z * -0.025f;
            float roll = incl.x * 0.025f;
            actions[0] = thrust + roll + pitch;
            actions[1] = thrust - roll - pitch;
            actions[2] = thrust - roll + pitch;
            actions[3] = thrust + roll - pitch;
        }
    }
}