using UnityEngine;
using UnityEngine.InputSystem;

namespace Utility
{
    [RequireComponent(typeof(Camera))]
    public class CameraController : MonoBehaviour
    {
        [Header("Target")]
        public Transform target;

        [Header("Orbit Settings")]
        public float distance = 200f;
        public float minDistance = 30f;
        public float maxDistance = 200f;
        public float zoomSpeed = 10f;

        public float orbitSpeedX = 10f;
        public float orbitSpeedY = 10f;
        public float minPitch = 0f;
        public float maxPitch = 80f;

        [Header("Additive Target Rotation")]
        public float pitchInfluence = 1f;
        public float rollInfluence = 1f;

        [Header("Smoothing")]
        public float smoothTime = 0.1f;

        private float _yaw = 0f;
        private float _pitch = 20f;

        private Vector3 _currentVelocity;
        private Vector3 _targetPosition;

        private bool _isOrbiting = false;

        private void Awake()
        {
            if (!target)
            {
                Debug.LogError("CameraController: No target set!");
            }
        }

        private void Update()
        {
            if (target == null) return;

            var mouse = Mouse.current;
            var scroll = mouse.scroll.ReadValue().y;
            
            distance -= scroll * zoomSpeed;
            distance = Mathf.Clamp(distance, minDistance, maxDistance);

            _isOrbiting = mouse.rightButton.isPressed;

            if (_isOrbiting)
            {
                var delta = mouse.delta.ReadValue();
                _yaw += delta.x * orbitSpeedX * Time.deltaTime;
                _pitch -= delta.y * orbitSpeedY * Time.deltaTime;
                _pitch = Mathf.Clamp(_pitch, minPitch, maxPitch);
            }
            
            Vector3 targetEuler = target.rotation.eulerAngles;
            float additivePitch = 0;//NormalizeAngle(targetEuler.x) * pitchInfluence;
            float additiveRoll = 0; //NormalizeAngle(targetEuler.z) * rollInfluence;

            Quaternion orbitRotation = Quaternion.Euler(_pitch + additivePitch, _yaw, additiveRoll);
            _targetPosition = target.position - (orbitRotation * Vector3.forward * distance);

            transform.position = Vector3.SmoothDamp(transform.position, _targetPosition, ref _currentVelocity, smoothTime);
            transform.rotation = orbitRotation;
        }
        
        private float NormalizeAngle(float angle)
        {
            if (angle > 180f) angle -= 360f;
            return angle;
        }
    }
}
