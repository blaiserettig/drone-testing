using UnityEngine;

public class VirtualGyro : MonoBehaviour
{
    public Rigidbody rb;

    public Vector3 gyro;
    public Vector3 accel;
    public Vector3 euler;

    private Vector3 _lastVel;

    // Update is called once per frame
    void FixedUpdate()
    {
        gyro = rb.angularVelocity * Mathf.Rad2Deg;
        accel = (rb.linearVelocity - _lastVel) / Time.fixedDeltaTime;
        _lastVel = rb.linearVelocity;
        euler = rb.rotation.eulerAngles;
    }
}
