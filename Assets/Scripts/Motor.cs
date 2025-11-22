using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Serialization;

public class Motor : MonoBehaviour
{
    public Rigidbody parentRb;
    public float scale = 1f;
    public bool isClockwise = false;
    
    private float _currentThrust;
    private float _targetThrust;

    public float torqueCoefficient = 0.05f;
    
    //  input latency
    public float timeConstant = 0.05f; 

    public void SetThrust(float t)
    {
        _targetThrust = Mathf.Clamp01(t);
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        float f = dt / (timeConstant + dt);
        
        _currentThrust += (_targetThrust - _currentThrust) * f;

        parentRb.AddForceAtPosition(transform.up * scale * _currentThrust, transform.position);
        
        //float torque = torqueCoefficient * _currentThrust * (isClockwise ? 1f : -1f);
        //parentRb.AddTorque(transform.up * torque, ForceMode.Force);
    }
    
    
}
