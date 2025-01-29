using System;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

namespace DroneProject
{
    /// <summary>
    /// Trains <see cref="RotorControlAgent"/>.
    /// </summary>
    public class RotorControlTrainer : RotorControlAg
    {
        [SerializeField, Tooltip("Log stats to TensorBoard, interval in decision steps")]
        private int m_StatsInterval;
        
        // Allow some lag between expected and measured velocity.
        private Vector3[] m_VelocityBuffer;
        private int m_VelocityBufferIndex;
        private const int k_VelocityBufferSize = 10;
        private float m_VelocityError;
        private Vector3 m_DefPos;

        /// <inheritdoc />
        public override void Initialize()
        {
            base.Initialize();
            m_DefPos = Drone.worldPosition;

            // Standalone agent has its own DecisionRequester.
            DecisionInterval = GetComponent<DecisionRequester>().DecisionPeriod;

            m_VelocityBuffer = new Vector3[k_VelocityBufferSize];
        }

        /// <inheritdoc />
        public override void OnEpisodeBegin()
        {
            base.OnEpisodeBegin();

            m_VelocityBufferIndex = 0;
            Array.Clear(m_VelocityBuffer, 0, k_VelocityBufferSize);
        }
        
        /// <inheritdoc />
        public override void CollectObservations(VectorSensor sensor)
        {
            base.CollectObservations(sensor);

            AddRewards();
        }

        private void AddRewards()
        { 
           
        }

        public override void OnActionReceived(ActionBuffers actionBuffers)
        {
            base.OnActionReceived(actionBuffers);
        }
    }
}