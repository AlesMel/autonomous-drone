using UnityEngine;

public class Rotor : MonoBehaviour
{

    public Vector3 worldPosition => transform.position;

    public Vector3 worldThrustAxis => transform.up;

    public Vector3 localTorqueAxis { get; private set; }

    public Rigidbody rb;


    public void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    public void Initialize()
    {
        localTorqueAxis = transform.parent.InverseTransformVector(-transform.up);
    }
}
