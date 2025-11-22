using UnityEngine;
using UnityEngine.InputSystem;

public class ControlActions : MonoBehaviour
{
    private InputSystem_Actions _controls;
    
    public float Roll {get; private set;}
    public float Pitch {get; private set;}
    public float Yaw {get; private set;}
    public float Throttle {get; private set;}

    private void Awake()
    {
        _controls = new InputSystem_Actions();
        
        _controls.Drone.Roll.performed += ctx => Roll = ctx.ReadValue<float>();
        _controls.Drone.Roll.canceled += ctx => Roll = 0f;
        
        _controls.Drone.Pitch.performed += ctx => Pitch = ctx.ReadValue<float>();
        _controls.Drone.Pitch.canceled += ctx => Pitch = 0f;
        
        _controls.Drone.Yaw.performed += ctx => Yaw = ctx.ReadValue<float>();
        _controls.Drone.Yaw.canceled += ctx => Yaw = 0f;
    }

    private void Update()
    {
        float rawThrottle = _controls.Drone.Throttle.ReadValue<float>();
        Throttle = Mathf.Max(0f, rawThrottle);
    }


    private void OnEnable()  => _controls.Enable();
    private void OnDisable() => _controls.Disable();
}
