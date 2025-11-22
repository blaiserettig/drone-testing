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

        [Header("Smoothing")]
        public float positionSmooth = 0.1f;
        public float rotationSmooth = 0.08f;

        private float _yaw = 0f;
        private float _pitch = 20f;

        // smoothed values
        private float _smoothYaw;
        private float _smoothPitch;
        private float _yawVel;
        private float _pitchVel;
        private Vector3 _positionVel;

        private Vector3 _targetPosition;

        private bool _isOrbiting = false;

        private void Update()
        {
            if (!target) return;

            var mouse = Mouse.current;
            var scroll = mouse.scroll.ReadValue().y;

            // zoom
            distance -= scroll * zoomSpeed;
            distance = Mathf.Clamp(distance, minDistance, maxDistance);

            // orbit
            _isOrbiting = mouse.rightButton.isPressed;
            if (_isOrbiting)
            {
                var delta = mouse.delta.ReadValue();
                _yaw += delta.x * orbitSpeedX * Time.deltaTime;
                _pitch -= delta.y * orbitSpeedY * Time.deltaTime;
                _pitch = Mathf.Clamp(_pitch, minPitch, maxPitch);
            }

            // Smooth rotation
            _smoothYaw = Mathf.SmoothDampAngle(_smoothYaw, _yaw, ref _yawVel, rotationSmooth);
            _smoothPitch = Mathf.SmoothDampAngle(_smoothPitch, _pitch, ref _pitchVel, rotationSmooth);

            Quaternion rotation = Quaternion.Euler(_smoothPitch, _smoothYaw, 0);
            _targetPosition = target.position - rotation * Vector3.forward * distance;
        }

        private void LateUpdate()
        {
            if (!target) return;

            // Smooth position
            transform.position = Vector3.SmoothDamp(
                transform.position,
                _targetPosition,
                ref _positionVel,
                positionSmooth
            );

            // Smooth rotation (already smoothed via damp)
            transform.rotation = Quaternion.Euler(_smoothPitch, _smoothYaw, 0);
        }
    }
}
