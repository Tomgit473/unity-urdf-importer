using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using UnityEngine;

namespace RoverCompatibility
{
    [System.Serializable]
    public class ArticulatedJointTarget
    {
        public string jointName;
        public ArticulationBody body;
        [Range(-180f, 180f)] public float targetAngleDeg;
        public float defaultAngleDeg;
        public float minLimitDeg;
        public float maxLimitDeg;
        public bool isThigh;
        public bool isKnee;
        public bool isAbduction;
        public bool isRightSide;
        public bool isFront;
    }

    /// <summary>
    /// Interactive controller and posture stabilizer for multi-joint articulated robots
    /// (Quadrupeds like Unitree Go1/Go2, Boston Dynamics Spot, Google Barkour, arms, and hexapods).
    /// Provides critically damped stance-holding, interactive trotting gait with W/A/S/D,
    /// and hotkey presets [1] Stand, [2] Crouch, [3] Sit, [0] Zero.
    /// </summary>
    [DisallowMultipleComponent]
    public class ArticulatedRobotController : MonoBehaviour
    {
        public enum StancePreset
        {
            Custom,
            Stand,
            Crouch,
            Sit,
            Zero
        }

        [Header("Stance Management")]
        public StancePreset currentPreset = StancePreset.Stand;

        [Header("Drive Stiffness & Damping (No Collapsing / No Jitter)")]
        [Tooltip("Position spring stiffness (Nm/rad). Calibrated for stable posture without jitter.")]
        public float jointStiffness = 2000f;

        [Tooltip("Critical damping (Nm/(rad/s)). Eliminates harmonic shaking and violent vibrations.")]
        public float jointDamping = 140f;

        [Tooltip("Maximum joint motor torque/effort in Nm.")]
        public float maxTorque = 500f;

        [Header("Interactive Locomotion (W/A/S/D)")]
        [Tooltip("Enable keyboard trot / stepping when pressing W/A/S/D or Arrow keys (quadrupeds only).")]
        public bool enableInteractiveDrive = true;

        [Range(0.5f, 5.0f)] public float gaitCycleSpeed = 2.0f;
        [Range(2f, 15f)] public float stepAmplitudeDeg = 6.0f;

        [Header("Active Upright Posture & Locomotion Stabilization")]
        [Tooltip("Actively levels roll and pitch to prevent quadrupeds from tipping over or falling off.")]
        public bool enableUprightStabilizer = true;
        [Tooltip("Restorative upright alignment strength.")]
        public float uprightStiffness = 18f;
        [Tooltip("Forward walking speed in m/s during interactive trotting.")]
        public float walkSpeed = 1.2f;
        [Tooltip("Turning yaw rate in deg/s during interactive trotting.")]
        public float turnSpeed = 65f;

        [Header("Interactive Stance Hotkeys")]
        [Tooltip("Enable hotkeys: [1] Stand, [2] Crouch, [3] Sit/Rest, [0] Zero angles")]
        public bool enableKeyboardHotkeys = true;

        [Header("External / UI Input (Optional)")]
        public bool useExternalInput = false;
        public float externalForward = 0f;
        public float externalTurn = 0f;

        [Header("Joint Breakdown")]
        public List<ArticulatedJointTarget> joints = new List<ArticulatedJointTarget>();

        // Internal gait state
        private float _gaitPhase = 0f;
        private float _currentForward = 0f;
        private float _currentTurn = 0f;
        private float _smoothedTurnAv = 0f;
        private float _smoothedForwardVel = 0f;
        private ArticulationBody _rootBody;

        public float CurrentForward => _currentForward;
        public float CurrentTurn => _currentTurn;
        public bool IsMoving => enableInteractiveDrive && currentPreset == StancePreset.Stand && (Mathf.Abs(_currentForward) > 0.05f || Mathf.Abs(_currentTurn) > 0.05f);

        private void Awake()
        {
            CacheRootBody();

            if (GetComponent<SimpleDifferentialDrive>() != null || GetComponentInChildren<SimpleDifferentialDrive>() != null)
            {
                enableInteractiveDrive = false;
            }

            if (joints == null || joints.Count == 0)
            {
                InitializeJoints();
            }
        }

        private void Start()
        {
            CacheRootBody();

            if (GetComponent<SimpleDifferentialDrive>() != null || GetComponentInChildren<SimpleDifferentialDrive>() != null)
            {
                enableInteractiveDrive = false;
            }

            if (joints == null || joints.Count == 0)
            {
                InitializeJoints();
            }

            // Default to Stand stance on startup to prevent collapsing
            ApplyStandStance();
        }

        private void CacheRootBody()
        {
            if (_rootBody != null) return;
            var bodies = GetComponentsInChildren<ArticulationBody>(true);
            _rootBody = bodies?.FirstOrDefault(b => b.isRoot);
            if (_rootBody != null)
            {
                _rootBody.angularDamping = Mathf.Max(_rootBody.angularDamping, 4.0f);
            }
        }

        private void Update()
        {
            ReadInput(out float forward, out float turn, out bool k1, out bool k2, out bool k3, out bool k0);

            if (useExternalInput)
            {
                if (Mathf.Abs(externalForward) > Mathf.Abs(forward)) forward = externalForward;
                if (Mathf.Abs(externalTurn) > Mathf.Abs(turn)) turn = externalTurn;
            }

            if (enableKeyboardHotkeys)
            {
                if (k1) ApplyStandStance();
                if (k2) ApplyCrouchStance();
                if (k3) ApplySitStance();
                if (k0) ApplyZeroStance();
            }

            _currentForward = forward;
            _currentTurn = turn;
        }

        private void FixedUpdate()
        {
            if (joints == null || joints.Count == 0) return;

            CacheRootBody();

            // CRITICAL: If robot possesses wheels (driven by SimpleDifferentialDrive)
            // or has fewer than 8 leg joints (e.g. bipeds, arms), legs NEVER trot!
            bool hasWheelDrive = GetComponent<SimpleDifferentialDrive>() != null || GetComponentInChildren<SimpleDifferentialDrive>() != null;
            if (hasWheelDrive || joints.Count < 8)
            {
                enableInteractiveDrive = false;
            }

            // 1. Heading-Invariant Upright Posture Stabilization (prevents tipping or rolling at ANY yaw angle)
            Vector3 uprightAngularVel = Vector3.zero;

            if (enableUprightStabilizer && _rootBody != null && !_rootBody.immovable)
            {
                // Calculate tilt axis and angle directly in 3D space:
                // Cross product gives the exact horizontal rotation axis needed to right the body.
                // This completely eliminates Euler gimbal/heading cross-talk where turning east turns pitch into roll!
                Vector3 currentUp = _rootBody.transform.up;
                Vector3 tiltAxis = Vector3.Cross(currentUp, Vector3.up);
                float tiltAngleDeg = Vector3.Angle(currentUp, Vector3.up);

                if (tiltAngleDeg > 0.05f && tiltAxis.sqrMagnitude > 0.00001f)
                {
                    float correctionGain = Mathf.Clamp(tiltAngleDeg * (uprightStiffness * 0.05f), 0f, 4.0f);
                    uprightAngularVel = tiltAxis.normalized * (correctionGain * Mathf.Deg2Rad);
                }

                // Emergency righting only if severely inverted (> 65 degrees upside down)
                if (tiltAngleDeg > 65f)
                {
                    Vector3 euler = _rootBody.transform.rotation.eulerAngles;
                    Quaternion targetRot = Quaternion.Euler(0f, euler.y, 0f);
                    _rootBody.TeleportRoot(_rootBody.transform.position + Vector3.up * 0.1f, targetRot);
                    _rootBody.angularVelocity = Vector3.zero;
                    _rootBody.linearVelocity = Vector3.zero;
                    _smoothedTurnAv = 0f;
                    _smoothedForwardVel = 0f;
                }
            }

            // 2. Interactive Locomotion (True quadruped walking & turning)
            bool isMoving = enableInteractiveDrive && currentPreset == StancePreset.Stand && (Mathf.Abs(_currentForward) > 0.05f || Mathf.Abs(_currentTurn) > 0.05f);
            if (isMoving)
            {
                _gaitPhase += Time.fixedDeltaTime * gaitCycleSpeed * Mathf.PI * 2f;
                if (_gaitPhase > Mathf.PI * 200f) _gaitPhase -= Mathf.PI * 200f;

                // Propel root body forward and turn smoothly without physical snapping
                if (_rootBody != null && !_rootBody.immovable)
                {
                    Vector3 fwd = _rootBody.transform.forward;
                    fwd.y = 0f;
                    fwd.Normalize();

                    Vector3 curVel = _rootBody.linearVelocity;
                    float targetForwardVel = _currentForward * walkSpeed;
                    _smoothedForwardVel = Mathf.MoveTowards(_smoothedForwardVel, targetForwardVel, 4.0f * Time.fixedDeltaTime);
                    Vector3 targetLinearVel = fwd * _smoothedForwardVel;
                    _rootBody.linearVelocity = new Vector3(targetLinearVel.x, curVel.y, targetLinearVel.z);

                    // Smooth yaw angular velocity ramp (prevents instant impulse tear on ground contacts)
                    float targetYawRate = _currentTurn * (turnSpeed * Mathf.Deg2Rad);
                    _smoothedTurnAv = Mathf.MoveTowards(_smoothedTurnAv, targetYawRate, (turnSpeed * Mathf.Deg2Rad * 4.0f) * Time.fixedDeltaTime);

                    // Upright torque is purely horizontal; yaw torque is purely vertical (zero cross-talk!)
                    Vector3 yawAngularVel = Vector3.up * _smoothedTurnAv;
                    _rootBody.angularVelocity = uprightAngularVel + yawAngularVel;
                }
            }
            else
            {
                // When stopped, gently damp residual velocities while keeping upright balance
                _smoothedForwardVel = 0f;
                _smoothedTurnAv = Mathf.MoveTowards(_smoothedTurnAv, 0f, (turnSpeed * Mathf.Deg2Rad * 6.0f) * Time.fixedDeltaTime);

                if (_rootBody != null && !_rootBody.immovable)
                {
                    Vector3 yawAngularVel = Vector3.up * _smoothedTurnAv;
                    _rootBody.angularVelocity = uprightAngularVel + yawAngularVel;
                }
            }

            // 3. Apply Target Drives across Articulation joints every FixedUpdate
            bool isTurningInPlace = isMoving && Mathf.Abs(_currentForward) < 0.15f && Mathf.Abs(_currentTurn) > 0.05f;

            for (int i = 0; i < joints.Count; i++)
            {
                var j = joints[i];
                if (j.body == null) continue;

                // STRICT GUARD: Never overwrite a velocity-driven wheel!
                if (j.body.xDrive.driveType == ArticulationDriveType.Velocity) continue;

                float targetAngle = j.targetAngleDeg;

                // Apply dynamic trot gait offset when walking or turning
                if (isMoving)
                {
                    // Diagonal pairs trot: (FL + RR) vs (FR + RL)
                    bool isDiagonalPairA = (j.isFront && !j.isRightSide) || (!j.isFront && j.isRightSide);
                    float phaseOffset = isDiagonalPairA ? 0f : Mathf.PI;
                    float legOscillation = Mathf.Sin(_gaitPhase + phaseOffset);

                    if (j.isThigh)
                    {
                        float pitchOffset;
                        if (isTurningInPlace)
                        {
                            // In-place turn trot: left and right sides swing in opposite directions to step naturally in a circle
                            float sideDir = j.isRightSide ? -1f : 1f;
                            pitchOffset = legOscillation * (stepAmplitudeDeg * 0.75f) * _currentTurn * sideDir;
                        }
                        else
                        {
                            // Moving forward while turning: apply differential stride length (outside leg steps wider)
                            float turnStrideMultiplier = j.isRightSide ? (1f - _currentTurn * 0.4f) : (1f + _currentTurn * 0.4f);
                            pitchOffset = legOscillation * stepAmplitudeDeg * _currentForward * turnStrideMultiplier;
                        }

                        // Invert pitch swing for rear legs with negative stand angle (e.g. ANYmal, M20 rear HFE)
                        if (j.targetAngleDeg < 0f && !j.isFront) pitchOffset = -pitchOffset;

                        targetAngle = SafeClamp(j.targetAngleDeg + pitchOffset, j.minLimitDeg, j.maxLimitDeg);
                    }
                    else if (j.isKnee)
                    {
                        // Lift knee during swing phase (vital during in-place turning to avoid dragging feet across floor!)
                        float activity = isTurningInPlace ? Mathf.Abs(_currentTurn) * 0.8f : Mathf.Abs(_currentForward);
                        float lift = Mathf.Max(0f, legOscillation) * (stepAmplitudeDeg * 0.9f) * activity;
                        float liftOffset = j.targetAngleDeg >= 0f ? lift : -lift;
                        targetAngle = SafeClamp(j.targetAngleDeg + liftOffset, j.minLimitDeg, j.maxLimitDeg);
                    }
                }

                var drive = j.body.xDrive;
                drive.driveType = ArticulationDriveType.Target;
                drive.stiffness = jointStiffness;
                drive.damping = jointDamping;
                drive.forceLimit = maxTorque;
                drive.target = targetAngle;

                j.body.xDrive = drive;
            }
        }

        [ContextMenu("Initialize Joint List")]
        public void InitializeJoints()
        {
            joints.Clear();
            var bodies = GetComponentsInChildren<ArticulationBody>(true);
            Transform rootTransform = transform;

            // Retrieve any wheel drive bodies from SimpleDifferentialDrive if present
            var diffDrive = GetComponent<SimpleDifferentialDrive>() ?? GetComponentInChildren<SimpleDifferentialDrive>();
            var wheelBodies = new HashSet<ArticulationBody>();
            if (diffDrive != null && diffDrive.wheels != null)
            {
                foreach (var w in diffDrive.wheels)
                {
                    if (w != null && w.body != null) wheelBodies.Add(w.body);
                }
            }

            foreach (var b in bodies)
            {
                if (b == null || b.isRoot) continue;
                if (b.jointType != ArticulationJointType.RevoluteJoint) continue;

                // STRICTLY SKIP WHEELS:
                // 1. If SimpleDifferentialDrive registered this body as a wheel
                if (wheelBodies.Contains(b)) continue;

                string lower = b.name.ToLowerInvariant();

                // 2. Skip active continuous wheels, tires, rotors
                if (lower.Contains("wheel") || lower.Contains("tire") || lower.Contains("rotor"))
                {
                    continue;
                }

                // 3. If robot has differential drive, also skip foot links that are wheels (e.g. Go2W, B2W)
                if (diffDrive != null && (lower.Contains("foot") || lower.EndsWith("_foot") || lower.Contains("ankle_mj")))
                {
                    continue;
                }

                // Analyze 3D spatial position relative to robot root
                Vector3 localPos = rootTransform.InverseTransformPoint(b.transform.position);

                bool isExplicitRear = lower.Contains("rear") || lower.Contains("back") || lower.Contains("hind") ||
                                      lower.StartsWith("lh") || lower.StartsWith("rh") || lower.Contains("_lh") || lower.Contains("_rh") ||
                                      lower.StartsWith("hl") || lower.StartsWith("hr") || lower.Contains("_hl") || lower.Contains("_hr") ||
                                      lower.Contains("rl_") || lower.Contains("rr_") || lower.StartsWith("rl") || lower.StartsWith("rr");

                bool isExplicitFront = lower.Contains("front") ||
                                       lower.StartsWith("lf") || lower.StartsWith("rf") || lower.Contains("_lf") || lower.Contains("_rf") ||
                                       lower.StartsWith("fl") || lower.StartsWith("fr") || lower.Contains("_fl") || lower.Contains("_fr");

                bool isFront = isExplicitFront || (!isExplicitRear && localPos.z > -0.001f);

                bool isExplicitRight = lower.Contains("right") || lower.StartsWith("rf") || lower.StartsWith("rh") ||
                                       lower.Contains("_rf") || lower.Contains("_rh") || lower.StartsWith("fr") || lower.StartsWith("hr") ||
                                       lower.Contains("_fr") || lower.Contains("_hr") || lower.Contains("rr_");

                bool isExplicitLeft = lower.Contains("left") || lower.StartsWith("lf") || lower.StartsWith("lh") ||
                                      lower.Contains("_lf") || lower.Contains("_lh") || lower.StartsWith("fl") || lower.StartsWith("hl") ||
                                      lower.Contains("_fl") || lower.Contains("_hl") || lower.Contains("rl_");

                bool isRight = isExplicitRight || (!isExplicitLeft && localPos.x > 0.001f);

                // Identify joint anatomical role
                bool isAbduction = lower.Contains("abduction") || lower.Contains("_hx") || lower.Contains(".hx") || lower.Contains("haa") ||
                                   lower.Contains("hipx") || (lower.Contains("hip") && (lower.Contains("roll") || lower.Contains("abd") || lower.EndsWith("_hip_joint") || lower.EndsWith("_hip")));

                bool isThigh = !isAbduction && (lower.Contains("thigh") || lower.Contains("upper") || lower.Contains("femur") ||
                               lower.Contains("hfe") || lower.Contains("hipy") || lower.Contains("_hy") || lower.Contains(".hy") ||
                               (lower.Contains("hip") && lower.Contains("pitch")));

                bool isKnee = lower.Contains("calf") || lower.Contains("lower") || lower.Contains("knee") ||
                              lower.Contains("tibia") || lower.Contains("kfe") || lower.Contains("_kn") || lower.Contains(".kn") ||
                              lower.Contains("shank");

                var x = b.xDrive;
                var jt = new ArticulatedJointTarget
                {
                    jointName = b.name,
                    body = b,
                    targetAngleDeg = x.target,
                    defaultAngleDeg = x.target,
                    minLimitDeg = x.lowerLimit,
                    maxLimitDeg = x.upperLimit,
                    isAbduction = isAbduction,
                    isThigh = isThigh,
                    isKnee = isKnee,
                    isRightSide = isRight,
                    isFront = isFront
                };
                joints.Add(jt);
            }

            Debug.Log($"[Articulated Robot Controller] Initialized {joints.Count} controllable joints on '{name}'.");
        }

        [ContextMenu("Apply Stand Stance")]
        public void ApplyStandStance()
        {
            currentPreset = StancePreset.Stand;
            if (joints.Count == 0) InitializeJoints();

            string rootName = name.ToLowerInvariant();
            bool isAnymal = rootName.Contains("anymal") || joints.Any(j => j.jointName.ToLowerInvariant().Contains("kfe"));
            bool isM20 = (rootName.Contains("m20") && !rootName.Contains("m2020")) || joints.Any(j => j.jointName.ToLowerInvariant().Contains("hipx") || j.jointName.ToLowerInvariant().Contains("hipy"));
            bool isGo = rootName.Contains("go2") || rootName.Contains("go1") || rootName.Contains("magicdog") || joints.Any(j => j.jointName.ToLowerInvariant().Contains("calf"));
            bool isSpot = rootName.Contains("spot") || joints.Any(j => j.jointName.ToLowerInvariant().Contains(".hx") || j.jointName.ToLowerInvariant().Contains(".kn"));
            bool isBarkour = rootName.Contains("barkour");
            bool isUpkie = rootName.Contains("upkie");

            if (isM20)
            {
                jointStiffness = 3500f;
                jointDamping = 250f;
                maxTorque = 2500f;
                if (GetComponent<SimpleDifferentialDrive>() != null) enableInteractiveDrive = false;
            }
            else if (isAnymal)
            {
                jointStiffness = 2200f;
                jointDamping = 150f;
                maxTorque = 600f;
            }
            else if (isSpot)
            {
                jointStiffness = 2200f;
                jointDamping = 150f;
                maxTorque = 650f;
            }
            else if (isGo)
            {
                jointStiffness = 1800f;
                jointDamping = 120f;
                maxTorque = 500f;
            }
            else if (isBarkour)
            {
                jointStiffness = 1800f;
                jointDamping = 120f;
                maxTorque = 500f;
            }
            else if (isUpkie)
            {
                jointStiffness = 2500f;
                jointDamping = 180f;
                maxTorque = 800f;
                if (GetComponent<SimpleDifferentialDrive>() != null) enableInteractiveDrive = false;
            }

            // Detect whether rear legs use X-configuration (rear knee bends forward, rear thigh pitches backward)
            bool hasXChassis = isAnymal || isM20 ||
                joints.Any(j => j.isKnee && !j.isFront && j.minLimitDeg >= -5f && j.maxLimitDeg > 20f);

            foreach (var j in joints)
            {
                if (isAnymal)
                {
                    if (j.isAbduction) j.targetAngleDeg = 0f;
                    else if (j.isThigh) j.targetAngleDeg = j.isFront ? 35f : -35f;
                    else if (j.isKnee) j.targetAngleDeg = j.isFront ? -65f : 65f;
                    else j.targetAngleDeg = j.defaultAngleDeg;
                }
                else if (isM20)
                {
                    if (j.isAbduction) j.targetAngleDeg = 0f;
                    else if (j.isThigh) j.targetAngleDeg = j.isFront ? 35f : -35f;
                    else if (j.isKnee) j.targetAngleDeg = j.isFront ? -75f : 75f;
                    else j.targetAngleDeg = j.defaultAngleDeg;
                }
                else if (isSpot)
                {
                    if (j.isAbduction) j.targetAngleDeg = 0f;
                    else if (j.isThigh) j.targetAngleDeg = 40f;
                    else if (j.isKnee) j.targetAngleDeg = -70f;
                    else j.targetAngleDeg = j.defaultAngleDeg;
                }
                else if (isGo)
                {
                    if (j.isAbduction) j.targetAngleDeg = 0f;
                    else if (j.isThigh) j.targetAngleDeg = 36f;
                    else if (j.isKnee) j.targetAngleDeg = -72f;
                    else j.targetAngleDeg = j.defaultAngleDeg;
                }
                else if (isBarkour)
                {
                    // Barkour knee limits are strictly positive [0, 144 deg]. 57.3 deg = 1.0 rad (official standing pose)
                    if (j.isAbduction) j.targetAngleDeg = 0f;
                    else if (j.isThigh) j.targetAngleDeg = 30f;
                    else if (j.isKnee) j.targetAngleDeg = 57.3f;
                    else j.targetAngleDeg = j.defaultAngleDeg;
                }
                else if (isUpkie)
                {
                    // Upkie wheeled biped: slight flex to keep wheels squarely under torso with minimum strain
                    if (j.isAbduction) j.targetAngleDeg = 0f;
                    else if (j.isThigh) j.targetAngleDeg = 10f;
                    else if (j.isKnee) j.targetAngleDeg = -15f;
                    else j.targetAngleDeg = j.defaultAngleDeg;
                }
                else
                {
                    // Universal kinematic adaptation for arbitrary custom user models
                    if (j.isAbduction)
                    {
                        j.targetAngleDeg = 0f;
                    }
                    else if (j.isThigh)
                    {
                        float naturalThigh = (!j.isFront && hasXChassis) ? -32f : 32f;
                        j.targetAngleDeg = SafeClamp(naturalThigh, j.minLimitDeg, j.maxLimitDeg);
                    }
                    else if (j.isKnee)
                    {
                        if (j.minLimitDeg >= -5f && j.maxLimitDeg > 20f)
                        {
                            // Strictly positive limits (e.g. Barkour-style flexion)
                            float naturalBend = Mathf.Clamp(55f, Mathf.Max(20f, j.minLimitDeg + 5f), j.maxLimitDeg - 5f);
                            j.targetAngleDeg = naturalBend;
                        }
                        else if (!j.isFront && hasXChassis)
                        {
                            // Rear knee flexes forward in X-quadruped chassis
                            j.targetAngleDeg = SafeClamp(65f, j.minLimitDeg, j.maxLimitDeg);
                        }
                        else if (j.maxLimitDeg <= 5f && j.minLimitDeg < -20f)
                        {
                            // Strictly negative limits (e.g. Spot, Go2, B2W style flexion)
                            float naturalBend = Mathf.Clamp(-68f, j.minLimitDeg + 5f, Mathf.Min(-20f, j.maxLimitDeg - 5f));
                            j.targetAngleDeg = naturalBend;
                        }
                        else
                        {
                            // Symmetric or wide limits
                            float naturalBend = j.isFront ? -65f : (hasXChassis ? 65f : -65f);
                            j.targetAngleDeg = SafeClamp(naturalBend, j.minLimitDeg, j.maxLimitDeg);
                        }
                    }
                    else
                    {
                        j.targetAngleDeg = j.defaultAngleDeg;
                    }
                }

                j.targetAngleDeg = SafeClamp(j.targetAngleDeg, j.minLimitDeg, j.maxLimitDeg);

                if (j.body != null && j.body.xDrive.driveType != ArticulationDriveType.Velocity)
                {
                    var drive = j.body.xDrive;
                    drive.driveType = ArticulationDriveType.Target;
                    drive.stiffness = jointStiffness;
                    drive.damping = jointDamping;
                    drive.forceLimit = maxTorque;
                    drive.target = j.targetAngleDeg;
                    j.body.xDrive = drive;

                    // Immediately configure PhysX reduced space coordinate to prevent explosive snap
                    try
                    {
                        j.body.jointPosition = new ArticulationReducedSpace(j.targetAngleDeg * Mathf.Deg2Rad);
                    }
                    catch { }
                }
            }
        }

        [ContextMenu("Apply Crouch Stance")]
        public void ApplyCrouchStance()
        {
            currentPreset = StancePreset.Crouch;
            if (joints.Count == 0) InitializeJoints();

            bool hasXChassis = joints.Any(j => j.isKnee && !j.isFront && j.minLimitDeg >= -5f && j.maxLimitDeg > 20f) ||
                name.ToLowerInvariant().Contains("anymal") || name.ToLowerInvariant().Contains("m20");

            foreach (var j in joints)
            {
                if (j.isThigh)
                {
                    float crouchThigh = (!j.isFront && hasXChassis) ? -55f : 55f;
                    j.targetAngleDeg = SafeClamp(crouchThigh, j.minLimitDeg, j.maxLimitDeg);
                }
                else if (j.isKnee)
                {
                    bool bendsPositive = j.minLimitDeg >= -5f || (!j.isFront && hasXChassis);
                    float crouchKnee = bendsPositive ? 100f : -100f;
                    j.targetAngleDeg = SafeClamp(crouchKnee, j.minLimitDeg, j.maxLimitDeg);
                }
                else
                {
                    j.targetAngleDeg = j.defaultAngleDeg;
                }

                if (j.body != null && j.body.xDrive.driveType != ArticulationDriveType.Velocity)
                {
                    var drive = j.body.xDrive;
                    drive.driveType = ArticulationDriveType.Target;
                    drive.stiffness = jointStiffness;
                    drive.damping = jointDamping;
                    drive.forceLimit = maxTorque;
                    drive.target = j.targetAngleDeg;
                    j.body.xDrive = drive;

                    try
                    {
                        j.body.jointPosition = new ArticulationReducedSpace(j.targetAngleDeg * Mathf.Deg2Rad);
                    }
                    catch { }
                }
            }
        }

        [ContextMenu("Apply Sit Stance")]
        public void ApplySitStance()
        {
            currentPreset = StancePreset.Sit;
            if (joints.Count == 0) InitializeJoints();

            bool hasXChassis = joints.Any(j => j.isKnee && !j.isFront && j.minLimitDeg >= -5f && j.maxLimitDeg > 20f) ||
                name.ToLowerInvariant().Contains("anymal") || name.ToLowerInvariant().Contains("m20");

            foreach (var j in joints)
            {
                if (j.isThigh)
                {
                    // Front legs stay slightly standing, rear legs fold onto the floor
                    float sitThigh = j.isFront ? 25f : ((!j.isFront && hasXChassis) ? -75f : 75f);
                    j.targetAngleDeg = SafeClamp(sitThigh, j.minLimitDeg, j.maxLimitDeg);
                }
                else if (j.isKnee)
                {
                    if (j.isFront)
                    {
                        bool bendsPositive = j.minLimitDeg >= -5f;
                        j.targetAngleDeg = SafeClamp(bendsPositive ? 50f : -50f, j.minLimitDeg, j.maxLimitDeg);
                    }
                    else
                    {
                        bool bendsPositive = j.minLimitDeg >= -5f || hasXChassis;
                        j.targetAngleDeg = SafeClamp(bendsPositive ? 120f : -120f, j.minLimitDeg, j.maxLimitDeg);
                    }
                }
                else
                {
                    j.targetAngleDeg = 0f;
                }

                if (j.body != null && j.body.xDrive.driveType != ArticulationDriveType.Velocity)
                {
                    var drive = j.body.xDrive;
                    drive.driveType = ArticulationDriveType.Target;
                    drive.stiffness = jointStiffness;
                    drive.damping = jointDamping;
                    drive.forceLimit = maxTorque;
                    drive.target = j.targetAngleDeg;
                    j.body.xDrive = drive;

                    try
                    {
                        j.body.jointPosition = new ArticulationReducedSpace(j.targetAngleDeg * Mathf.Deg2Rad);
                    }
                    catch { }
                }
            }
        }

        [ContextMenu("Apply Zero Stance")]
        public void ApplyZeroStance()
        {
            currentPreset = StancePreset.Zero;
            if (joints.Count == 0) InitializeJoints();

            foreach (var j in joints)
            {
                j.targetAngleDeg = 0f;
                if (j.body != null && j.body.xDrive.driveType != ArticulationDriveType.Velocity)
                {
                    var drive = j.body.xDrive;
                    drive.driveType = ArticulationDriveType.Target;
                    drive.stiffness = jointStiffness;
                    drive.damping = jointDamping;
                    drive.forceLimit = maxTorque;
                    drive.target = 0f;
                    j.body.xDrive = drive;

                    try
                    {
                        j.body.jointPosition = new ArticulationReducedSpace(0f);
                    }
                    catch { }
                }
            }
        }

        private static float SafeClamp(float target, float min, float max)
        {
            if (Mathf.Approximately(min, 0f) && Mathf.Approximately(max, 0f)) return target;
            if (min >= max) return target;
            return Mathf.Clamp(target, min, max);
        }

        private static bool _inputSystemChecked = false;
                private static Func<Tuple<float, float, bool, bool, bool, bool>> _readNewInputFunc = null;

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

                var d1Prop = keyboardType.GetProperty("digit1Key");
                var d2Prop = keyboardType.GetProperty("digit2Key");
                var d3Prop = keyboardType.GetProperty("digit3Key");
                var d0Prop = keyboardType.GetProperty("digit0Key");
                var n1Prop = keyboardType.GetProperty("numpad1Key");
                var n2Prop = keyboardType.GetProperty("numpad2Key");
                var n3Prop = keyboardType.GetProperty("numpad3Key");
                var n0Prop = keyboardType.GetProperty("numpad0Key");

                var keyControl = wProp != null ? wProp.PropertyType : null;
                var isPressedProp = keyControl != null ? keyControl.GetProperty("isPressed") : null;
                var wasPressedProp = keyControl != null ? keyControl.GetProperty("wasPressedThisFrame") : null;

                if (currentProp != null && isPressedProp != null)
                {
                    _readNewInputFunc = () =>
                    {
                        var kb = currentProp.GetValue(null);
                        if (kb == null) return Tuple.Create(0f, 0f, false, false, false, false);

                        bool IsPressed(System.Reflection.PropertyInfo p) => p != null && (bool)isPressedProp.GetValue(p.GetValue(kb));
                        bool WasPressed(System.Reflection.PropertyInfo p) => p != null && wasPressedProp != null && (bool)wasPressedProp.GetValue(p.GetValue(kb));

                        float fwd = 0f;
                        float trn = 0f;
                        if (IsPressed(wProp) || IsPressed(upProp)) fwd += 1f;
                        if (IsPressed(sProp) || IsPressed(downProp)) fwd -= 1f;
                        if (IsPressed(dProp) || IsPressed(rightProp)) trn += 1f;
                        if (IsPressed(aProp) || IsPressed(leftProp)) trn -= 1f;

                        bool b1 = WasPressed(d1Prop) || WasPressed(n1Prop);
                        bool b2 = WasPressed(d2Prop) || WasPressed(n2Prop);
                        bool b3 = WasPressed(d3Prop) || WasPressed(n3Prop);
                        bool b0 = WasPressed(d0Prop) || WasPressed(n0Prop);

                        return Tuple.Create(fwd, trn, b1, b2, b3, b0);
                    };
                }
            }
            catch { }
        }

        private void ReadInput(out float forward, out float turn, out bool k1, out bool k2, out bool k3, out bool k0)
        {
            forward = 0f;
            turn = 0f;
            k1 = false; k2 = false; k3 = false; k0 = false;

            // 1. Safe New Input System check (zero hard dependency on Unity.InputSystem assembly)
            if (!_inputSystemChecked) InitNewInputReader();
            if (_readNewInputFunc != null)
            {
                try
                {
                    var t = _readNewInputFunc();
                    forward += t.Item1;
                    turn += t.Item2;
                    k1 |= t.Item3;
                    k2 |= t.Item4;
                    k3 |= t.Item5;
                    k0 |= t.Item6;
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

                if (!k1) k1 = Input.GetKeyDown(KeyCode.Alpha1) || Input.GetKeyDown(KeyCode.Keypad1);
                if (!k2) k2 = Input.GetKeyDown(KeyCode.Alpha2) || Input.GetKeyDown(KeyCode.Keypad2);
                if (!k3) k3 = Input.GetKeyDown(KeyCode.Alpha3) || Input.GetKeyDown(KeyCode.Keypad3);
                if (!k0) k0 = Input.GetKeyDown(KeyCode.Alpha0) || Input.GetKeyDown(KeyCode.Keypad0);
            }
            catch (InvalidOperationException) { }

            forward = Mathf.Clamp(forward, -1f, 1f);
            turn = Mathf.Clamp(turn, -1f, 1f);
        }
    }
}
