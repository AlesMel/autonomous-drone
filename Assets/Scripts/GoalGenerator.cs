using UnityEngine;
// TODO: Change velocity, because its not only position
public class GoalGenerator : MonoBehaviour
{
    private struct GoalPoint
    {
        public Vector3 position;
        public Vector3 velocity;
    }

    private struct Attractor
    {
        public Vector3 position;
        public float strength;
    }
    private Attractor[] attractors;

    private GoalPoint point;

    // [SerializeField, Min(1), Tooltip("Number of attractors")]
    private int numAttractors = 5;
    // [SerializeField, Min(0.01f), Tooltip("Attractor min strength")]
    private float minStrength = 0.01f;
    // [SerializeField, Min(0.01f), Tooltip("Attractor max strength")]
    private float maxStrength = 1f;
    private float radius = 20.0f;

    // [SerializeField, Min(0), Tooltip("Max velocity")]
    private float maxVelocity = 1f;
    private float maxVelocitySqr;
    // [SerializeField, Min(1), Tooltip("Direction change interval in steps")]
    private int directionChangeSteps = 500;

    private int currentStep = 0;

    public Vector3 worldVelocity => point.velocity;
    public Vector3 worldLookDirection => Vector3.ProjectOnPlane(-point.position, Vector3.up).normalized;

    [SerializeField, Range(0f, 0.1f), Tooltip("Friction strength")]
    private float friction = 0.04f;
    private float invFriction;

    private float stopProbability = 0.5f;
    private bool stop;
    public bool isMoving => !stop;

    public void Initialize()
    {
        attractors = new Attractor[numAttractors];
        maxVelocitySqr = Mathf.Pow(maxVelocity, 2);
        invFriction = 1 - friction;
    }

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
        if (stop)
        {
            point.velocity *= 0.75f;
        }
        else
        {
            for (int i = 0; i < numAttractors; i++)
            {
                Vector3 delta = attractors[i].position - point.position;
                float sqrMag = delta.sqrMagnitude;
                point.velocity += attractors[i].strength / sqrMag * delta;

                if (sqrMag < 1)
                {
                    ChangeDirection();
                    break;
                }
            }

            if (point.velocity.sqrMagnitude > maxVelocitySqr)
            {
                point.velocity = point.velocity.normalized * maxVelocity;
            }
            else
            {
                point.velocity *= invFriction;
            }
        }
        // Update the goal point's position based on its velocity
        point.position += point.velocity * deltaTime; // Assuming ManagedUpdate is called in FixedUpdate
    }

    public void ResetGoal()
    {
        currentStep = 0;
        point.position = Vector3.zero;
        point.velocity = Vector3.zero;
        ChangeDirection(); // Initially set a random direction
    }

    private void ChangeDirection()
    {
        for (int i = 0; i < numAttractors; i++)
        {
            attractors[i].position = Random.insideUnitSphere * radius;
            attractors[i].strength = Random.Range(minStrength, maxStrength);
        }

        stop = Random.value < stopProbability;
    }

    public Vector3 GetGoalPosition()
    {
        return point.position;
    }

    public Vector3 GetGoalVelocity()
    {
        return point.velocity;
    }

    private void OnDrawGizmosSelected()
    {
        Vector3 goalPosition = GetGoalPosition();

        Gizmos.color = Color.blue;
        for (int i = 0; i < numAttractors; i++)
        {
            Gizmos.DrawSphere(attractors[i].position, 0.1f);
        }

        // Draw the goal position
        Gizmos.color = Color.green;
        Gizmos.DrawSphere(goalPosition, 0.05f); // Adjust the size as needed

        // Draw the velocity vector from the goal position
        Gizmos.color = Color.blue;
        Gizmos.DrawRay(goalPosition, GetGoalVelocity());
    }
}
