using UnityEngine;
using System.Collections;

[System.Serializable]
public class PIDController
{
    public float Kp;
    public float Ki;
    public float Kd;
    public float MaxOutput;
    public float MaxI;

    private float _integral;
    private float _lastInput;
    
    public bool armed = false;
    public float idleThrottle = 0.05f;

    public PIDController(float kp, float ki, float kd, float maxOutput, float maxI)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        MaxOutput = maxOutput;
        MaxI = maxI;
    }

    public float Update(float error, float input, float dt)
    {
        float p = Kp * error;

        _integral += error * dt;
        _integral = Mathf.Clamp(_integral, -MaxI, MaxI);
        float i = Ki * _integral;

        float d = -Kd * (input - _lastInput) / dt;
        _lastInput = input;

        float output = p + i + d;
        return Mathf.Clamp(output, -MaxOutput, MaxOutput);
    }

    public void Reset()
    {
        _integral = 0f;
        _lastInput = 0f;
    }
}

public class FlightController : MonoBehaviour
{
    [Header("Hardware")]
    public VirtualGyro gyro;
    public Motor[] motors; // 0:LF, 1:RF, 2:LR, 3:RR
    private ControlActions _input;

    // Angle PIDs (Outer Loop) - Output is Target Rate (deg/s)
    private PIDController rollAnglePID = new PIDController(3.0f, 0.0f, 0.5f, 200f, 0f);
    private PIDController pitchAnglePID = new PIDController(3.0f, 0.0f, 0.5f, 200f, 0f);

    // Rate PIDs (Inner Loop) - Output is Motor Mix
    private PIDController rollRatePID = new PIDController(0.8f, 0.1f, 0.15f, 1f, 0.5f);
    private PIDController pitchRatePID = new PIDController(0.8f, 0.1f, 0.15f, 1f, 0.5f);
    private PIDController yawRatePID = new PIDController(0.5f, 0.1f, 0.01f, 1f, 0.5f);

    [Header("Settings")]
    public float maxRollPitchAngle = 45f;
    public float maxYawRate = 150;

    private Vector3 _currentEuler;
    private Vector3 _currentGyro;
    private float _targetRoll, _targetPitch, _targetYawRate, _throttle;
    private float _motorMixLF, _motorMixRF, _motorMixLR, _motorMixRR;

    void Start()
    {
        // 200Hz
        Time.fixedDeltaTime = 0.005f;
        
        _input = GetComponent<ControlActions>();

        StartCoroutine(SensorsTask());
        StartCoroutine(StabilityTask());
        StartCoroutine(MotorsTask());
    }

    IEnumerator SensorsTask()
    {
        while (true)
        {
            _currentEuler = gyro.transform.localRotation.eulerAngles;
            _currentEuler.x = NormalizeAngle(_currentEuler.x);
            _currentEuler.y = NormalizeAngle(_currentEuler.y);
            _currentEuler.z = NormalizeAngle(_currentEuler.z);
            
            _currentGyro = gyro.gyro; // deg/s
            
            _targetRoll = -_input.Roll * maxRollPitchAngle;
            _targetPitch = _input.Pitch * maxRollPitchAngle;
            _targetYawRate = _input.Yaw * maxYawRate;
            _throttle = _input.Throttle;

            yield return new WaitForFixedUpdate(); 
        }
    }

    IEnumerator StabilityTask()
    {
        while (true)
        {
            float dt = Time.fixedDeltaTime;

            // error
            float rollError = _targetRoll - _currentEuler.z;
            float pitchError = _targetPitch - _currentEuler.x;

            // target rates
            float rollRateSetPoint = rollAnglePID.Update(rollError, _currentEuler.z, dt);
            float pitchRateSetPoint = pitchAnglePID.Update(pitchError, _currentEuler.x, dt);

            // rate error
            float rollRateError = rollRateSetPoint - _currentGyro.z;
            float pitchRateError = pitchRateSetPoint - _currentGyro.x;
            float yawRateError = _targetYawRate - _currentGyro.y;

            // PID output
            float rollOut = rollRatePID.Update(rollRateError, _currentGyro.z, dt);
            float pitchOut = pitchRatePID.Update(pitchRateError, _currentGyro.x, dt);
            float yawOut = yawRatePID.Update(yawRateError, _currentGyro.y, dt);

            // mixing
            // LF (CW)  RF (CCW)
            // LR (CCW) RR (CW)
            // Corrected for Unity Frame:
            // pitch forward (+X) -> rear boost (+), front cut (-)
            // roll left (+Z) -> left cut (-), right boost (+)
            
            // Right after calculating yawOut
            Debug.Log($"Yaw Input: {_input.Yaw} | Target Rate: {_targetYawRate} | Current Gyro.Y: {_currentGyro.y} | Yaw Error: {yawRateError} | Yaw Out: {yawOut}");
            
            _motorMixLF = _throttle - pitchOut - rollOut - yawOut;
            _motorMixRF = _throttle - pitchOut + rollOut + yawOut;
            _motorMixLR = _throttle + pitchOut - rollOut + yawOut;
            _motorMixRR = _throttle + pitchOut + rollOut - yawOut;

            Debug.Log($"Throttle: {_throttle} | Motors: LF={_motorMixLF:F3} RF={_motorMixRF:F3} LR={_motorMixLR:F3} RR={_motorMixRR:F3}");
            
            yield return new WaitForFixedUpdate();
        }
    }

    IEnumerator MotorsTask()
    {
        while (true) 
        {
            if (_throttle <= 0.01f && transform.position.y < 1f)
            {
                _motorMixLF = 0f;
                _motorMixRF = 0f;
                _motorMixLR = 0f;
                _motorMixRR = 0f;
            }
            
            motors[0].SetThrust(_motorMixLF);
            motors[1].SetThrust(_motorMixRF);
            motors[2].SetThrust(_motorMixLR);
            motors[3].SetThrust(_motorMixRR);

            yield return new WaitForFixedUpdate();
        }
    }

    private float NormalizeAngle(float angle)
    {
        if (angle > 180f) angle -= 360f;
        return angle;
    }
}
