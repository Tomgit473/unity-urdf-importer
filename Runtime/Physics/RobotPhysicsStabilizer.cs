using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace RoverCompatibility
{
    /// <summary>
    /// Runtime and edit-mode stabilizer for multi-joint articulated robots (quadrupeds, arms, rovers).
    /// Eliminates PhysX numerical explosions, limb sagging, jitter, and parts falling off:
    /// 
    /// 1. Intra-Robot Self-Collision Elimination:
    ///    Pairs all internal colliders with Physics.IgnoreCollision(c1, c2, true).
    ///    This prevents violent depenetration forces when CAD links overlap at hinge pivots.
    /// 
    /// 2. Mass Ratio Conditioning (12:1 Rule):
    ///    PhysX ArticulationBody solvers experience numerical divergence when connected links have
    ///    extreme mass ratios (e.g. 50kg chassis to 0.001kg foot). Clamps micro-masses and limits
    ///    parent-child ratios to 12:1, stopping limbs from tearing away or falling off.
    /// 
    /// 3. High-Precision Solver Iterations & Depenetration Capping:
    ///    Configures solverIterations = 32 and solverVelocityIterations = 16, and caps
    ///    maxDepenetrationVelocity = 1.0 m/s to eliminate explosive catapult launches.
    /// 
    /// 4. Fixed & Revolute Joint Integrity:
    ///    Locks all 6 DOFs on Fixed joints with matchAnchors = true to prevent feet/sensors from detaching.
    ///    Locks linear motion on Revolute joints so hinges never stretch under gravity.
    /// </summary>
    [DisallowMultipleComponent]
    [DefaultExecutionOrder(-500)]
    public class RobotPhysicsStabilizer : MonoBehaviour
    {
        [Header("Collision Protection")]
        [Tooltip("Dynamically ignore collisions between all internal colliders within this robot's hierarchy.")]
        public bool ignoreInternalCollisions = true;

        [Header("PhysX Solver Precision")]
        [Range(16, 64)] public int solverIterations = 32;
        [Range(4, 32)] public int solverVelocityIterations = 16;
        [Range(0.2f, 3.0f)] public float maxDepenetrationVelocity = 1.0f;

        [Header("Joint Stability & Constraints")]
        [Tooltip("Lock swingY, swingZ, and linear axes on Revolute joints to prevent hinge dislocation.")]
        public bool enforceSingleAxisRevolute = true;

        [Tooltip("Lock all 6 DOFs on Fixed joints to prevent child parts (feet, sensors) from detaching.")]
        public bool lockAllFixedJointAxes = true;

        [Tooltip("Condition body masses and parent-child mass ratios to eliminate joint tearing.")]
        public bool conditionMassRatios = true;

        [Tooltip("Legacy alias for conditionMassRatios.")]
        public bool clampMicroMasses { get => conditionMassRatios; set => conditionMassRatios = value; }

        [Header("Diagnostics")]
        [SerializeField] private int ignoredColliderPairs = 0;
        [SerializeField] private int stabilizedBodiesCount = 0;

        public int IgnoredColliderPairs => ignoredColliderPairs;
        public int StabilizedBodiesCount => stabilizedBodiesCount;

        private void Awake()
        {
            ApplyStabilization();
        }

        private void OnEnable()
        {
            ApplyStabilization();
        }

        /// <summary>
        /// Applies full physical stabilization across all colliders and ArticulationBodies under this robot.
        /// Can be safely invoked in Editor or at Runtime.
        /// </summary>
        [ContextMenu("Apply Stabilization Now")]
        public void ApplyStabilization()
        {
            // 1. Eliminate internal self-collisions
            if (ignoreInternalCollisions)
            {
                var colliders = GetComponentsInChildren<Collider>(true);
                ignoredColliderPairs = 0;
                for (int i = 0; i < colliders.Length; i++)
                {
                    if (colliders[i] == null) continue;
                    for (int j = i + 1; j < colliders.Length; j++)
                    {
                        if (colliders[j] == null) continue;
                        Physics.IgnoreCollision(colliders[i], colliders[j], true);
                        ignoredColliderPairs++;
                    }
                }
            }

            // 2. Tune ArticulationBodies & Condition Mass Hierarchy
            var bodies = GetComponentsInChildren<ArticulationBody>(true);
            stabilizedBodiesCount = bodies.Length;
            if (bodies == null || bodies.Length == 0) return;

            // Calculate total robot mass to establish a healthy minimum link mass
            float totalMass = 0f;
            foreach (var b in bodies) if (b != null) totalMass += b.mass;
            float minAllowedMass = Mathf.Max(0.25f, totalMass * 0.015f);

            // Create calibrated physics materials:
            // - Wheels use Minimum combine mode and 0.40 friction so lateral skid-steering slips smoothly without chattering
            // - Body / Feet have firm, predictable traction without sticking
            PhysicsMaterial wheelMat = new PhysicsMaterial("RobotWheelTraction")
            {
                dynamicFriction = 0.40f,
                staticFriction = 0.45f,
                frictionCombine = PhysicsMaterialCombine.Minimum,
                bounciness = 0.0f,
                bounceCombine = PhysicsMaterialCombine.Minimum
            };

            PhysicsMaterial bodyMat = new PhysicsMaterial("RobotStabilizerTraction")
            {
                dynamicFriction = 0.80f,
                staticFriction = 0.80f,
                frictionCombine = PhysicsMaterialCombine.Average,
                bounciness = 0.0f,
                bounceCombine = PhysicsMaterialCombine.Minimum
            };

            // Identify all registered wheel bodies from SimpleDifferentialDrive if present
            var diffDrive = GetComponent<SimpleDifferentialDrive>() ?? GetComponentInChildren<SimpleDifferentialDrive>();
            var wheelBodies = new HashSet<ArticulationBody>();
            if (diffDrive != null && diffDrive.wheels != null)
            {
                foreach (var w in diffDrive.wheels)
                {
                    if (w != null && w.body != null) wheelBodies.Add(w.body);
                }
            }

            var allColliders = GetComponentsInChildren<Collider>(true);
            foreach (var c in allColliders)
            {
                if (c == null) continue;
                var ab = c.GetComponentInParent<ArticulationBody>();
                string cName = c.name.ToLowerInvariant();
                string pName = c.transform.parent != null ? c.transform.parent.name.ToLowerInvariant() : "";

                bool isWheelCollider = (ab != null && wheelBodies.Contains(ab)) ||
                    cName.Contains("wheel") || cName.Contains("tire") || cName.Contains("tyre") || cName.Contains("rotor") ||
                    pName.Contains("wheel") || pName.Contains("tire") || pName.Contains("tyre") || pName.Contains("rotor");

                c.sharedMaterial = isWheelCollider ? wheelMat : bodyMat;
            }

            foreach (var body in bodies)
            {
                if (body == null) continue;

                body.solverIterations = Mathf.Max(body.solverIterations, solverIterations);
                body.solverVelocityIterations = Mathf.Max(body.solverVelocityIterations, solverVelocityIterations);
                body.maxDepenetrationVelocity = maxDepenetrationVelocity;

                if (body.isRoot)
                {
                    // Gentle root damping: prevents high-speed oscillation without fighting turning motors
                    body.angularDamping = Mathf.Max(body.angularDamping, 0.8f);
                    body.linearDamping = Mathf.Max(body.linearDamping, 0.5f);
                }
                body.jointFriction = Mathf.Max(body.jointFriction, 0.05f);

                // Mass conditioning: prevent 1000:1 micro-mass divergence
                if (conditionMassRatios)
                {
                    if (body.mass < minAllowedMass)
                    {
                        body.mass = minAllowedMass;
                    }

                    // Enforce max 12:1 ratio against direct parent
                    var parentBody = body.transform.parent != null ? body.transform.parent.GetComponentInParent<ArticulationBody>() : null;
                    if (parentBody != null && parentBody.mass > 0f)
                    {
                        float minByParent = parentBody.mass / 12f;
                        float maxByParent = parentBody.mass * 12f;
                        body.mass = Mathf.Clamp(body.mass, minByParent, maxByParent);
                    }
                }

                // Ensure diagonal inertia tensor has solid positive values
                float minInertia = Mathf.Max(0.005f, body.mass * 0.015f);
                body.inertiaTensor = new Vector3(
                    Mathf.Max(body.inertiaTensor.x, minInertia),
                    Mathf.Max(body.inertiaTensor.y, minInertia),
                    Mathf.Max(body.inertiaTensor.z, minInertia)
                );

                if (lockAllFixedJointAxes && body.jointType == ArticulationJointType.FixedJoint)
                {
                    body.linearLockX = ArticulationDofLock.LockedMotion;
                    body.linearLockY = ArticulationDofLock.LockedMotion;
                    body.linearLockZ = ArticulationDofLock.LockedMotion;
                    body.twistLock = ArticulationDofLock.LockedMotion;
                    body.swingYLock = ArticulationDofLock.LockedMotion;
                    body.swingZLock = ArticulationDofLock.LockedMotion;
                    body.matchAnchors = true;
                }
                else if (enforceSingleAxisRevolute && body.jointType == ArticulationJointType.RevoluteJoint)
                {
                    // Linear axes must always be locked to prevent joint stretching / falling off
                    body.linearLockX = ArticulationDofLock.LockedMotion;
                    body.linearLockY = ArticulationDofLock.LockedMotion;
                    body.linearLockZ = ArticulationDofLock.LockedMotion;

                    // Non-driving rotation axes must be locked so the hinge cannot wobble
                    body.swingYLock = ArticulationDofLock.LockedMotion;
                    body.swingZLock = ArticulationDofLock.LockedMotion;
                }
            }
        }
    }
}
