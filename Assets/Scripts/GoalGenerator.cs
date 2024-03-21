using UnityEngine;

public class GoalGenerator : MonoBehaviour
{
    private struct GoalPoint
    {
        public Vector3 Position;
        public Vector3 Velocity;
    }

    private GoalPoint point;

    [SerializeField, Min(0), Tooltip("Max velocity")]
    private float maxVelocity = 5f;

    [SerializeField, Min(1), Tooltip("Direction change interval in steps")]
    private int directionChangeSteps = 10;

    private int currentStep = 0;

    public Vector3 worldVelocity => point.Velocity;
    public Vector3 worldLookDirection => Vector3.ProjectOnPlane(-point.Position, Vector3.up).normalized;

    private void Start()
    {
        ResetGoal();
    }

    // Call this from another script's FixedUpdate to manage goal point updates
    public void ManagedUpdate(float deltaTime)
    {
        currentStep++;
        if (currentStep >= directionChangeSteps)
        {
            ChangeDirection();
            currentStep = 0;
        }

        // Update the goal point's position based on its velocity
        point.Position += point.Velocity * deltaTime; // Assuming ManagedUpdate is called in FixedUpdate
    }

    public void ResetGoal()
    {
        currentStep = 0;
        point.Position = Vector3.zero;
        point.Velocity = Vector3.zero;
        ChangeDirection(); // Initially set a random direction
    }

    private void ChangeDirection()
    {
        // Generate a random direction
        Vector3 randomDirection = Random.insideUnitSphere.normalized;
        //float randomSpeed = Random.Range(0.0f, maxVelocity);

        point.Velocity = randomDirection * maxVelocity;
    }

    public Vector3 GetGoalPosition()
    {
        return transform.position + point.Position;
    }

    public Vector3 GetGoalVelocity()
    {
        return point.Velocity;
    }

    private void OnDrawGizmosSelected()
    {
        Vector3 goalPosition = GetGoalPosition();

        // Draw the goal position
        Gizmos.color = Color.green;
        Gizmos.DrawSphere(goalPosition, 0.05f); // Adjust the size as needed

        // Draw the velocity vector from the goal position
        Gizmos.color = Color.blue;
        Gizmos.DrawRay(goalPosition, GetGoalVelocity());
    }
}
