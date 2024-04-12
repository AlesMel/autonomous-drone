using UnityEngine;

public class Rotor : MonoBehaviour
{

    public Vector3 worldPosition => transform.position;
    public Vector3 localPosition => transform.localPosition;

    public Vector3 worldThrustAxis => transform.up;

    public Vector3 localTorqueAxis { get; private set; }

    public Rigidbody rb;
    public Transform propeller { get; private set; }

    public void Initialize()
    {
        propeller = transform.Find("Blade");
        rb = GetComponent<Rigidbody>();
        localTorqueAxis = transform.parent.InverseTransformVector(-transform.up);
    }
}
