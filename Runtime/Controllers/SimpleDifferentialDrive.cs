using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;

namespace RoverCompatibility
{
    /// <summary>
    /// Differential drive velocity controller for ArticulationBody-based rovers.
    /// Operates on the detected drive axis of each wheel every FixedUpdate.
    /// Reads W/A/S/D and Arrow keys with support for both legacy and new input systems.
    /// </summary>
    [DisallowMultipleComponent]
    public class SimpleDifferentialDrive : MonoBehaviour
    {
        [Header("Wheel Configuration")]
        [Tooltip("List of detected rover wheels and their drive axes (supports 2, 4, 6, 8 or N wheels).")]
        public List<WheelDriveInfo> wheels = new List<WheelDriveInfo>();

        [Header("Steering Configuration (Optional)")]
        [Tooltip("Active steering knuckle joints (e.g. for Ackermann or 4/6-corner steered rovers like Curiosity/Perseverance).")]
        public List<ArticulationBody> steeringJoints = new List<ArticulationBody>();

        [Tooltip("Maximum steering angle for steering knuckle joints in degrees.")]
        public float maxSteerAngle = 35f;

        [Header("Drive Parameters")]
        [Tooltip("Maximum wheel rotational speed in degrees per second (e.g. 400 deg/s ≈ 6.98 rad/s).")]
        public float maxSpeed = 400f;

        [Tooltip("Acceleration in deg/s^2 to smooth torque delivery and prevent vehicle tipping.")]
        public float acceleration = 800f;

        [Tooltip("Maximum motor torque per wheel in Nm. For heavy rovers like Clearpath Husky (~50kg), 3000-5000 Nm is recommended.")]
        public float maxTorque = 5000f;

        [Tooltip("Drive damping (velocity gain in PhysX). Keep > 0 to maintain torque under velocity control.")]
        public float wheelDamping = 500f;

        [Tooltip("Steering agility multiplier for differential skid-steering.")]
        [Range(0.1f, 2f)]
        public float turnFactor = 0.75f;

        [Header("Direction Inversion")]
        [Tooltip("Invert forward/backward drive direction.")]
        public bool invertForward = false;

        [Tooltip("Invert left/right turning direction.")]
        public bool invertTurn = false;

        [Tooltip("Invert left-side wheels rotation direction.")]
        public bool invertLeftSide = false;

        [Tooltip("Invert right-side wheels rotation direction.")]
        public bool invertRightSide = false;

        [Header("External / UI Drive Input")]
        public bool useExternalInput = false;
        public float externalForwardInput = 0f;
        public float externalTurnInput = 0f;

        [Header("Live Diagnostics")]
        [SerializeField] private float currentForwardInput = 0f;
        [SerializeField] private float currentTurnInput = 0f;
        [SerializeField] private float smoothedForwardVelocity = 0f;
        [SerializeField] private float smoothedTurnVelocity = 0f;
        [SerializeField] private float smoothedSteerAngle = 0f;

        public float CurrentForwardInput => currentForwardInput;
        public float CurrentTurnInput => currentTurnInput;
        public float SmoothedForwardVelocity => smoothedForwardVelocity;

        public void SetWheels(List<WheelDriveInfo> detectedWheels)
        {
            wheels = new List<WheelDriveInfo>(detectedWheels);
        }

        public void SetSteeringJoints(List<ArticulationBody> detectedSteeringJoints)
        {
            steeringJoints = new List<ArticulationBody>(detectedSteeringJoints);
        }

        private void FixedUpdate()
        {
            if (wheels == null || wheels.Count == 0) return;

            ReadInput(out float forward, out float turn);

            if (useExternalInput)
            {
                if (Mathf.Abs(externalForwardInput) > Mathf.Abs(forward)) forward = externalForwardInput;
                if (Mathf.Abs(externalTurnInput) > Mathf.Abs(turn)) turn = externalTurnInput;
            }

            currentForwardInput = forward;
            currentTurnInput = turn;

            if (invertForward) forward *= -1f;
            if (invertTurn) turn *= -1f;

            // Target velocities
            float targetLongitudinal = forward * maxSpeed;
            float targetTurn = turn * maxSpeed * turnFactor;

            // Smooth acceleration towards target velocities to prevent physical tipping and lateral wheel chattering
            float dt = Time.fixedDeltaTime;
            smoothedForwardVelocity = Mathf.MoveTowards(smoothedForwardVelocity, targetLongitudinal, acceleration * dt);
            smoothedTurnVelocity = Mathf.MoveTowards(smoothedTurnVelocity, targetTurn, acceleration * dt);

            // Update active steering knuckle angles with smooth slew-rate limiting
            // Automatically adapts to Front/Rear and Left/Right joint coordinate conventions so wheels never fight each other
            if (steeringJoints != null && steeringJoints.Count > 0)
            {
                float targetAngle = currentTurnInput * maxSteerAngle;
                smoothedSteerAngle = Mathf.MoveTowards(smoothedSteerAngle, targetAngle, 90f * dt);
                for (int s = 0; s < steeringJoints.Count; s++)
                {
                    var steer = steeringJoints[s];
                    if (steer == null) continue;
                    var drive = steer.xDrive;
                    drive.target = GetSteerJointAngle(steer, smoothedSteerAngle);
                    steer.xDrive = drive;
                }
            }

            for (int i = 0; i < wheels.Count; i++)
            {
                var info = wheels[i];
                if (info == null || info.body == null) continue;

                // Base longitudinal speed
                float targetVel = smoothedForwardVelocity;

                // Differential skid-steer turning:
                // To turn right: left wheels spin forward faster, right wheels spin slower/backward
                if (info.isRightSide)
                {
                    targetVel -= smoothedTurnVelocity;
                    if (invertRightSide) targetVel *= -1f;
                }
                else
                {
                    targetVel += smoothedTurnVelocity;
                    if (invertLeftSide) targetVel *= -1f;
                }

                if (info.invertDirection)
                {
                    targetVel *= -1f;
                }

                // Clamp wheel velocity to maxSpeed to prevent high-speed differential spikes
                targetVel = Mathf.Clamp(targetVel, -maxSpeed, maxSpeed);

                ApplyWheelVelocity(info, targetVel);
            }
        }

        private float GetSteerJointAngle(ArticulationBody steer, float steerAngleDeg)
        {
            if (steer == null) return 0f;

            Transform rootTransform = transform;
            Vector3 localPos = rootTransform.InverseTransformPoint(steer.transform.position);
            string name = steer.name.ToLowerInvariant();

            bool isFront = localPos.z >= 0f;
            if (name.Contains("front") || name.Contains("_f") || name.StartsWith("f")) isFront = true;
            else if (name.Contains("rear") || name.Contains("back") || name.Contains("_r") || name.Contains("_h")) isFront = false;

            // Check the knuckle joint's twist/drive axis in world coordinates relative to chassis Up
            Vector3 worldAxis = steer.transform.rotation * steer.anchorRotation * Vector3.right;
            float upDot = Vector3.Dot(worldAxis, rootTransform.up);
            float axisSign = upDot >= 0f ? 1f : -1f;

            // Front steered wheels turn with steer angle; rear steered wheels counter-steer to follow turning radius
            float longitudinalSign = isFront ? 1f : -1f;

            return steerAngleDeg * longitudinalSign * axisSign;
        }

        private void ApplyWheelVelocity(WheelDriveInfo info, float targetVel)
        {
            if (info == null || info.body == null) return;

            // Ensure wheel is revolute (some URDFs import them as FixedJoint)
            if (info.body.jointType == ArticulationJointType.FixedJoint)
            {
                info.body.jointType = ArticulationJointType.RevoluteJoint;
            }

            // Ensure drive axis is unlocked in PhysX
            if (info.axis == WheelAxis.X && info.body.twistLock != ArticulationDofLock.FreeMotion)
                info.body.twistLock = ArticulationDofLock.FreeMotion;
            else if (info.axis == WheelAxis.Y && info.body.swingYLock != ArticulationDofLock.FreeMotion)
                info.body.swingYLock = ArticulationDofLock.FreeMotion;
            else if (info.axis == WheelAxis.Z && info.body.swingZLock != ArticulationDofLock.FreeMotion)
                info.body.swingZLock = ArticulationDofLock.FreeMotion;

            ArticulationDrive drive = RoverAutoConfigurator.GetDrive(info.body, info.axis);
            drive.targetVelocity = targetVel;
            drive.forceLimit = maxTorque;
            drive.stiffness = 0f;           // Pure velocity mode (stiffness = 0)
            drive.damping = wheelDamping;
            drive.driveType = ArticulationDriveType.Velocity;
            RoverAutoConfigurator.SetDrive(info.body, info.axis, drive);
        }

        private static bool _inputSystemChecked = false;
        private static Func<Vector2> _readNewInputFunc = null;

        private static void InitNewInputReader()
        {
            if (_inputSystemChecked) return;
            _inputSystemChecked = true;

            try
            {
                var keyboardType = Type.GetType("UnityEngine.InputSystem.Keyboard, Unity.InputSystem");
                if (keyboardType == null) return;

                var currentProp = keyboardType.GetProperty("current", System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Static);
                var wProp = keyboardType.GetProperty("wKey");
                var sProp = keyboardType.GetProperty("sKey");
                var aProp = keyboardType.GetProperty("aKey");
                var dProp = keyboardType.GetProperty("dKey");
                var upProp = keyboardType.GetProperty("upArrowKey");
                var downProp = keyboardType.GetProperty("downArrowKey");
                var leftProp = keyboardType.GetProperty("leftArrowKey");
                var rightProp = keyboardType.GetProperty("rightArrowKey");

                var keyControl = wProp != null ? wProp.PropertyType : null;
                var isPressedProp = keyControl != null ? keyControl.GetProperty("isPressed") : null;

                if (currentProp != null && isPressedProp != null)
                {
                    _readNewInputFunc = () =>
                    {
                        var kb = currentProp.GetValue(null);
                        if (kb == null) return Vector2.zero;

                        bool IsPressed(System.Reflection.PropertyInfo p) => p != null && (bool)isPressedProp.GetValue(p.GetValue(kb));

                        float fwd = 0f;
                        float trn = 0f;
                        if (IsPressed(wProp) || IsPressed(upProp)) fwd += 1f;
                        if (IsPressed(sProp) || IsPressed(downProp)) fwd -= 1f;
                        if (IsPressed(dProp) || IsPressed(rightProp)) trn += 1f;
                        if (IsPressed(aProp) || IsPressed(leftProp)) trn -= 1f;
                        return new Vector2(fwd, trn);
                    };
                }
            }
            catch { }
        }

        private void ReadInput(out float forward, out float turn)
        {
            forward = 0f;
            turn = 0f;

            // 1. Safe New Input System check (zero hard dependency on Unity.InputSystem assembly)
            if (!_inputSystemChecked) InitNewInputReader();
            if (_readNewInputFunc != null)
            {
                try
                {
                    var v = _readNewInputFunc();
                    forward += v.x;
                    turn += v.y;
                }
                catch { }
            }

            // 2. Legacy Input Manager fallback
            try
            {
                if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow)) forward += 1f;
                if (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow)) forward -= 1f;
                if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow)) turn += 1f;
                if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow)) turn -= 1f;

                float v = Input.GetAxisRaw("Vertical");
                float h = Input.GetAxisRaw("Horizontal");
                if (Mathf.Abs(v) > Mathf.Abs(forward)) forward = v;
                if (Mathf.Abs(h) > Mathf.Abs(turn)) turn = h;
            }
            catch (InvalidOperationException) { }

            forward = Mathf.Clamp(forward, -1f, 1f);
            turn = Mathf.Clamp(turn, -1f, 1f);
        }
    }
}

