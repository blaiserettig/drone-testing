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
    private float _lastError;
    
    public PIDController(float kp, float ki, float kd, float maxOutput, float maxI)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        MaxOutput = maxOutput;
        MaxI = maxI;
    }

    public float Update(float error, float dt)
    {
        // Proportional
        float p = Kp * error;

        // Integral with anti-windup
        _integral += error * dt;
        _integral = Mathf.Clamp(_integral, -MaxI, MaxI);
        float i = Ki * _integral;

        // Derivative on error
        float derivative = (error - _lastError) / dt;
        float d = Kd * derivative;
        _lastError = error;

        float output = p + i + d;
        return Mathf.Clamp(output, -MaxOutput, MaxOutput);
    }

    public void Reset()
    {
        _integral = 0f;
        _lastError = 0f;
    }
}

public class FlightController : MonoBehaviour
{
    [Header("Hardware")]
    public VirtualGyro gyro;
    public Motor[] motors; // 0:LF, 1:RF, 2:LR, 3:RR
    private ControlActions _input;

    // Angle PIDs (Outer Loop) - Output is Target Rate (deg/s)
    private PIDController rollAnglePID = new PIDController(6.0f, 0.0f, 1.2f, 120f, 50f);
    private PIDController pitchAnglePID = new PIDController(6.0f, 0.0f, 1.2f, 120f, 50f);

    // Rate PIDs (Inner Loop) - Output is Motor Mix
    private PIDController rollRatePID = new PIDController(2.5f, 1.5f, 0.025f, 1.0f, 0.3f);
    private PIDController pitchRatePID = new PIDController(2.5f, 1.5f, 0.025f, 1.0f, 0.3f);
    private PIDController yawRatePID = new PIDController(3.5f, 1.5f, 0.01f, 1.0f, 0.3f);

    [Header("Settings")]
    public float maxRollPitchAngle = 45f;
    public float maxYawRate = 150;

    private Vector3 _currentEuler;
    private Vector3 _currentGyro;
    private float _targetRoll, _targetPitch, _targetYaw, _throttle;
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
            _targetYaw = _input.Yaw * maxYawRate;
            _throttle = _input.Throttle;

            yield return new WaitForFixedUpdate(); 
        }
    }

    IEnumerator StabilityTask()
    {
        while (true)
        {
            float dt = Time.fixedDeltaTime;
            
            // Calculate angle errors with proper wrapping
            float rollError = WrapAngleError(_targetRoll - _currentEuler.z);
            float pitchError = WrapAngleError(_targetPitch - _currentEuler.x);

            // Outer loop: angle error -> desired rotation rate
            float rollRateSetPoint = rollAnglePID.Update(rollError, dt);
            float pitchRateSetPoint = pitchAnglePID.Update(pitchError, dt);

            // Calculate rate errors
            float rollRateError = rollRateSetPoint - _currentGyro.z;
            float pitchRateError = pitchRateSetPoint - _currentGyro.x;
            float yawRateError = _targetYaw - _currentGyro.y;

            // Inner loop: rate error -> motor commands
            float rollOut = rollRatePID.Update(rollRateError, dt);
            float pitchOut = pitchRatePID.Update(pitchRateError, dt);
            float yawOut = yawRatePID.Update(yawRateError, dt);

            // Motor mixing
            // LF (CW)  RF (CCW)
            // LR (CCW) RR (CW)
            
            Debug.Log($"Angles: Roll={_currentEuler.z:F1} Pitch={_currentEuler.x:F1} Yaw={_currentEuler.y:F1}");
            Debug.Log($"Errors: RollE={rollError:F1} PitchE={pitchError:F1} | Rates: RollR={rollRateSetPoint:F1} PitchR={pitchRateSetPoint:F1}");
            Debug.Log($"PID Outputs: Roll={rollOut:F3} Pitch={pitchOut:F3} Yaw={yawOut:F3}");
            
            _motorMixLF = _throttle - pitchOut - rollOut - yawOut;
            _motorMixRF = _throttle - pitchOut + rollOut + yawOut;
            _motorMixLR = _throttle + pitchOut - rollOut + yawOut;
            _motorMixRR = _throttle + pitchOut + rollOut - yawOut;

            _motorMixLF = Mathf.Clamp01(_motorMixLF);
            _motorMixRF = Mathf.Clamp01(_motorMixRF);
            _motorMixLR = Mathf.Clamp01(_motorMixLR);
            _motorMixRR = Mathf.Clamp01(_motorMixRR);
            
            Debug.Log($"Throttle: {_throttle} | Motors: LF={_motorMixLF:F3} RF={_motorMixRF:F3} LR={_motorMixLR:F3} RR={_motorMixRR:F3}");
            
            yield return new WaitForFixedUpdate();
        }
    }

    IEnumerator MotorsTask()
    {
        while (true) 
        {
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

    private float WrapAngleError(float error)
    {
        while (error > 180f) error -= 360f;
        while (error < -180f) error += 360f;
        return error;
    }
}