using UnityEngine;
using UnityEngine.Serialization;

public class Motor : MonoBehaviour
{
    public Rigidbody parentRb;
    public float scale = 1f;
    
    private float _currentThrust;
    private float _targetThrust;
    
    //  input latency
    public float timeConstant = 0.05f; 

    public void SetThrust(float t)
    {
        _targetThrust = Mathf.Clamp01(t);
    }

    void FixedUpdate()
    {
        // lag filter new = old + (target - old) * (dt / tau)
        float dt = Time.fixedDeltaTime;
        float alpha = dt / (timeConstant + dt);
        
        _currentThrust = _currentThrust + (_targetThrust - _currentThrust) * alpha;

        parentRb.AddForceAtPosition(transform.up * scale * _currentThrust, transform.position);
    }
}
