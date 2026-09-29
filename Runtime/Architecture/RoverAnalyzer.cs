using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace RoverCompatibility
{
    /// <summary>
    /// Analyzes an imported URDF GameObject hierarchy to universally identify the robot's
    /// architecture (Wheeled Mobile Rover, Articulated Legged Quadruped/Biped, Robotic Arm,
    /// or Generic Multi-Joint System), classifying the chassis/base and all functional joints.
    /// </summary>
    public static class RoverAnalyzer
    {
        private static readonly string[] WheelKeywords = {
            "wheel", "tire", "tyre", "track", "caster",
            "left_front_link", "right_front_link", "left_back_link", "right_back_link",
            "left_rear_link", "right_rear_link", "left_front", "right_front", "left_back", "right_back"
        };

        private static readonly string[] LegKeywords = {
            "hip", "thigh", "calf", "knee", "foot", "shin", "ankle", "leg",
            "abduction", "coxa", "femur", "tibia"
        };

        private static readonly string[] ArmKeywords = {
            "shoulder", "elbow", "wrist", "forearm", "arm", "gripper", "finger",
            "flange", "link1", "link2", "link3", "link4", "link5", "link6"
        };

        private static readonly string[] SteeringKeywords = {
            "steer", "steering", "knuckle", "swivel", "pivot", "yaw", "base_wheel"
        };

        private static readonly string[] SuspensionKeywords = {
            "rocker", "bogie", "bogey", "suspension", "strut", "diff_bar", "diff_box", "rocker_bogie", "trailing_arm", "wishbone"
        };

        private static readonly string[] NonWheelKeywords = {
            "arm", "shoulder", "elbow", "wrist", "finger", "gripper",
            "pan", "tilt", "gimbal", "lidar", "laser", "camera", "sensor", "bumper",
            "steer", "steering", "base_wheel", "rocker", "bogie", "diff", "differential", "suspension",
            "hip", "knee", "thigh", "calf", "leg", "foot", "abduction", "femur", "tibia",
            "mast", "rsm", "hga", "antenna", "turret", "drill", "corer", "bit",
            "sherloc", "watson", "pixl", "head", "neck"
        };

        public static RoverDescriptor Analyze(GameObject root)
        {
            var desc = new RoverDescriptor { Root = root };

            if (root == null)
            {
                desc.Warnings.Add("Provided root GameObject is null.");
                return desc;
            }

            var bodies = root.GetComponentsInChildren<ArticulationBody>(true);
            if (bodies == null || bodies.Length == 0)
            {
                desc.Warnings.Add("No ArticulationBody components found under root hierarchy.");
                return desc;
            }

            desc.AllBodies = bodies.ToList();

            // Identify root ArticulationBody
            desc.RootArticulationBody = bodies.FirstOrDefault(b => b.isRoot);

            // Identify Chassis / Base:
            // Standard rovers have either base_link as root (e.g. Husky), or base_footprint (mass ~0.001) as root
            // with base_link having >90% of the mass. Chassis should be the primary heavy structural body.
            desc.Chassis = bodies
                .OrderByDescending(b => b.mass)
                .ThenByDescending(b => b.isRoot)
                .FirstOrDefault();

            // If highest mass body is negligible or not found, fallback to root articulation body
            if (desc.Chassis == null || (desc.RootArticulationBody != null && desc.RootArticulationBody.mass >= desc.Chassis.mass * 0.5f))
            {
                desc.Chassis = desc.RootArticulationBody ?? desc.Chassis;
            }

            if (desc.Chassis == null)
            {
                desc.Warnings.Add("No chassis or root ArticulationBody found.");
                return desc;
            }

            // Identify Wheels vs Steering vs Suspension vs Articulated Joints
            var namedWheels = new List<ArticulationBody>();
            var candidateFreeDrives = new List<ArticulationBody>();
            var nonWheelJoints = new List<ArticulationBody>();

            int legKeywordHits = 0;
            int armKeywordHits = 0;

            foreach (var body in bodies)
            {
                if (body == desc.Chassis || (desc.RootArticulationBody != null && body == desc.RootArticulationBody))
                    continue;

                string lowerName = body.name.ToLowerInvariant();

                if (LegKeywords.Any(kw => lowerName.Contains(kw))) legKeywordHits++;
                if (ArmKeywords.Any(kw => lowerName.Contains(kw))) armKeywordHits++;

                bool hasWheelName = WheelKeywords.Any(kw => lowerName == kw || lowerName.EndsWith("_" + kw) || lowerName.StartsWith(kw + "_") || lowerName.Contains(kw));

                // 1. FixedJoint Handling
                if (body.jointType == ArticulationJointType.FixedJoint)
                {
                    // Check if this fixed body is a wheel directly attached to chassis or base
                    // (like Pickerbot Mini where wheels were authored as fixed joints in ROS URDF)
                    bool isDirectFixedWheel = hasWheelName && body.transform.parent != null &&
                        (body.transform.parent.GetComponent<ArticulationBody>() == desc.Chassis ||
                         body.transform.parent.GetComponent<ArticulationBody>() == desc.RootArticulationBody ||
                         body.transform.parent.name.ToLowerInvariant().Contains("base"));

                    if (isDirectFixedWheel)
                    {
                        namedWheels.Add(body);
                    }
                    continue;
                }

                // 2. Steering knuckles and pivots
                bool isSteering = SteeringKeywords.Any(kw => lowerName.Contains(kw));
                if (isSteering)
                {
                    desc.SteeringJoints.Add(body);
                    continue;
                }

                // 3. Suspension (rocker, bogie, strut, etc.)
                bool isSuspension = SuspensionKeywords.Any(kw => lowerName.Contains(kw));
                if (isSuspension)
                {
                    desc.SuspensionJoints.Add(body);
                    continue;
                }

                // 4. Wheeled quadruped foot wheels (Go2W, B2W: FL_foot, FR_foot with RevoluteJoint)
                bool isFootWheel = lowerName.Contains("foot") && body.jointType == ArticulationJointType.RevoluteJoint;

                // 5. Upkie ankle rotor wheel
                bool isAnkleWheel = lowerName.Contains("ankle") && lowerName.Contains("rotor") &&
                    body.GetComponentsInChildren<Transform>(true).Any(t => t.name.ToLowerInvariant().Contains("wheel_hub"));

                // 6. Reject upper limb joints from being wheels
                // (e.g. hip, thigh, calf, knee, shoulder, elbow, arm must NEVER be wheels)
                bool isUpperLimb = NonWheelKeywords.Any(kw => lowerName.Contains(kw));
                if (isUpperLimb && !lowerName.Contains("wheel") && !lowerName.Contains("tire") && !isFootWheel && !isAnkleWheel)
                {
                    nonWheelJoints.Add(body);
                    continue;
                }

                if (hasWheelName || isFootWheel || isAnkleWheel)
                {
                    namedWheels.Add(body);
                    continue;
                }

                if (IsDriveFree(body))
                {
                    candidateFreeDrives.Add(body);
                }
                else
                {
                    nonWheelJoints.Add(body);
                }
            }

            // Architecture Classification
            // Supports 2, 4, 6, 8 or N-wheeled rovers (e.g. Turtlebot3 [2], Husky [4], Perseverance [6], 8x8 rovers [8])
            if (legKeywordHits >= 2 && namedWheels.Count >= 2)
            {
                // Wheeled quadruped / legged rover (e.g. Unitree Go2-W, ANYmal with wheels)
                desc.RobotType = RobotArchitectureType.ArticulatedLegged;
                desc.Wheels = namedWheels;
                nonWheelJoints.AddRange(candidateFreeDrives);
            }
            else if (namedWheels.Count >= 2)
            {
                // Pure wheeled rover OR mobile rover with manipulator arm (e.g. Pickerbot Mini, Perseverance)
                desc.RobotType = RobotArchitectureType.WheeledMobile;
                desc.Wheels = namedWheels;
                nonWheelJoints.AddRange(candidateFreeDrives);
            }
            else if (candidateFreeDrives.Count >= 2 && legKeywordHits < 2 && armKeywordHits < 2)
            {
                desc.RobotType = RobotArchitectureType.WheeledMobile;
                desc.Wheels = candidateFreeDrives;
            }
            else if (legKeywordHits >= 2)
            {
                desc.RobotType = RobotArchitectureType.ArticulatedLegged;
                desc.Wheels = namedWheels;
                nonWheelJoints.AddRange(candidateFreeDrives);
            }
            else if (armKeywordHits >= 2)
            {
                desc.RobotType = RobotArchitectureType.RoboticArm;
                desc.Wheels = new List<ArticulationBody>();
                nonWheelJoints.AddRange(candidateFreeDrives);
                nonWheelJoints.AddRange(namedWheels);
            }
            else if (nonWheelJoints.Count >= 1 || candidateFreeDrives.Count >= 1)
            {
                desc.RobotType = RobotArchitectureType.GeneralArticulated;
                desc.Wheels = new List<ArticulationBody>();
                nonWheelJoints.AddRange(candidateFreeDrives);
                nonWheelJoints.AddRange(namedWheels);
            }
            else
            {
                desc.RobotType = RobotArchitectureType.GeneralArticulated;
            }

            desc.NonWheelJoints = nonWheelJoints.Distinct().ToList();

            // Sort wheels front-to-back, left-to-right relative to chassis
            if (desc.Chassis != null && desc.Wheels.Count > 0)
            {
                desc.Wheels = desc.Wheels
                    .OrderByDescending(w => desc.Chassis.transform.InverseTransformPoint(w.transform.position).z)
                    .ThenBy(w => desc.Chassis.transform.InverseTransformPoint(w.transform.position).x)
                    .ToList();
            }

            if (desc.RobotType == RobotArchitectureType.WheeledMobile)
            {
                if (desc.Wheels.Count < 2)
                {
                    desc.Warnings.Add($"Only {desc.Wheels.Count} potential wheel(s) detected. Expected ≥ 2 for a mobile rover.");
                }
                else if (desc.Wheels.Count > 12)
                {
                    desc.Warnings.Add($"Unusually high number ({desc.Wheels.Count}) of wheel candidates detected – review hierarchy.");
                }
            }

            desc.ModelType = DetectModel(root, desc);

            return desc;
        }

        public static KnownRobotModel DetectModel(GameObject root, RoverDescriptor desc)
        {
            if (root == null) return KnownRobotModel.Generic;

            string rootName = root.name.ToLowerInvariant();
            var allNames = desc != null && desc.AllBodies != null
                ? desc.AllBodies.Select(b => b.name.ToLowerInvariant()).ToList()
                : new List<string>();

            // 1. Clearpath Husky
            if (rootName.Contains("husky") || allNames.Any(n => n.Contains("husky") || n.Contains("front_left_wheel_link")))
            {
                return KnownRobotModel.ClearpathHusky;
            }

            // 2. NASA Perseverance / Curiosity / M2020
            if (rootName.Contains("m2020") || rootName.Contains("perseverance") || rootName.Contains("curiosity") ||
                allNames.Any(n => n.Contains("body_rocker") || n.Contains("body_wheel_fl") || n.Contains("body_steer_fl")))
            {
                return KnownRobotModel.NasaPerseverance;
            }

            // 3. Deep Robotics M20 (Wheeled Quadruped)
            if ((rootName.Contains("m20") && !rootName.Contains("m2020")) ||
                allNames.Any(n => n == "fl_knee" || n == "fl_hipx" || n == "hl_hipx" || n == "fl_wheel"))
            {
                return KnownRobotModel.DeepRoboticsM20;
            }

            // 4. Unitree Go2-W (Wheeled Quadruped) / B2W / magicdog-w
            if (rootName.Contains("go2w") || rootName.Contains("go2_w") || rootName.Contains("b2w") || rootName.Contains("b2_w") || rootName.Contains("magicdog") ||
                ((rootName.Contains("go2") || rootName.Contains("b2") || allNames.Any(n => n.Contains("fl_calf") || n.Contains("fl_thigh"))) && desc != null && desc.Wheels.Count >= 2))
            {
                return KnownRobotModel.UnitreeGo2W;
            }

            // 5. Unitree Go2 (Pure Quadruped)
            if (rootName.Contains("go2") || allNames.Any(n => n.StartsWith("fl_hip") || n.StartsWith("fl_thigh") || n.StartsWith("fl_calf")))
            {
                if (desc != null && desc.Wheels.Count >= 2) return KnownRobotModel.UnitreeGo2W;
                return KnownRobotModel.UnitreeGo2;
            }

            // 6. Unitree Go1 (Pure Quadruped)
            if (rootName.Contains("go1"))
            {
                return KnownRobotModel.UnitreeGo1;
            }

            // 7. ANYmal D (Pure Quadruped)
            if (rootName.Contains("anymal") || allNames.Any(n => n.Contains("lf_kfe") || n.Contains("lh_kfe") || n.Contains("lf_haa")))
            {
                return KnownRobotModel.AnymalD;
            }

            // 8. Boston Dynamics Spot (Pure Quadruped)
            if (rootName.Contains("spot") || allNames.Any(n => n.Contains("fl.hx") || n.Contains("fl.hy") || n.Contains("fl.kn")))
            {
                return KnownRobotModel.BostonDynamicsSpot;
            }

            // 9. Google Barkour (Pure Quadruped)
            if (rootName.Contains("barkour"))
            {
                return KnownRobotModel.GoogleBarkour;
            }

            // 10. Pickerbot Mini (Mobile Rover with Arm)
            if (rootName.Contains("pickerbot") || rootName.Contains("mec_arm") ||
                allNames.Any(n => n.Contains("arm_a_link") || n.Contains("left_front_link")))
            {
                return KnownRobotModel.PickerbotMini;
            }

            // 11. RB-Vogui (4-wheel steerable mobile rover)
            if (rootName.Contains("rbvogui") || allNames.Any(n => n.Contains("rbvogui") || n.Contains("base_wheel")))
            {
                return KnownRobotModel.RbvoguiMobileRover;
            }

            // 12. Upkie (Wheeled Biped)
            if (rootName.Contains("upkie") || allNames.Any(n => n.Contains("upkie") || n.Contains("mj5208") || n.Contains("qdd100")))
            {
                return KnownRobotModel.UpkieWheeledBiped;
            }

            // 13. Robotic Arm
            if (desc != null && desc.RobotType == RobotArchitectureType.RoboticArm)
            {
                return KnownRobotModel.RoboticArm;
            }

            return KnownRobotModel.Generic;
        }

        public static bool IsRightSideWheel(ArticulationBody wheel, ArticulationBody chassis)
        {
            if (wheel == null) return false;

            string name = wheel.name.ToLowerInvariant();

            // 1. Explicit name check
            if (name.Contains("right") || name.Contains("_r_") || name.Contains(".r.") || name.StartsWith("r_") || name.EndsWith("_r") || name.Contains("_right"))
                return true;
            if (name.Contains("left") || name.Contains("_l_") || name.Contains(".l.") || name.StartsWith("l_") || name.EndsWith("_l") || name.Contains("_left"))
                return false;

            // 2. Spatial check relative to chassis frame (Unity standard: +X is Right)
            if (chassis != null)
            {
                Vector3 localPos = chassis.transform.InverseTransformPoint(wheel.transform.position);
                return localPos.x > 0.001f;
            }

            return wheel.transform.localPosition.x > 0.001f;
        }

        /// <summary>
        /// Analyzes the wheel's world rotation axis relative to the chassis's right vector.
        /// If the spin axis is flipped (pointing Left rather than Right), positive rotational velocity
        /// drives the wheel backward, so invertDirection is automatically set to true.
        /// </summary>
        public static bool DetectWheelInversion(ArticulationBody wheel, ArticulationBody chassis)
        {
            if (wheel == null || chassis == null) return false;

            // In Unity PhysX, a RevoluteJoint rotates around its anchor's X axis (twist axis)
            Vector3 wheelSpinAxisWorld = wheel.transform.rotation * wheel.anchorRotation * Vector3.right;
            Vector3 chassisRight = chassis.transform.right;

            // If wheel spin axis opposes chassis right (dot < -0.3), direction is inverted
            return Vector3.Dot(wheelSpinAxisWorld, chassisRight) < -0.3f;
        }

        private static bool IsDriveFree(ArticulationBody body)
        {
            if (body.twistLock == ArticulationDofLock.FreeMotion ||
                body.swingYLock == ArticulationDofLock.FreeMotion ||
                body.swingZLock == ArticulationDofLock.FreeMotion)
            {
                return true;
            }

            bool xFree = body.xDrive.lowerLimit < body.xDrive.upperLimit - 0.01f || (Mathf.Approximately(body.xDrive.lowerLimit, 0f) && Mathf.Approximately(body.xDrive.upperLimit, 0f));
            bool yFree = body.yDrive.lowerLimit < body.yDrive.upperLimit - 0.01f;
            bool zFree = body.zDrive.lowerLimit < body.zDrive.upperLimit - 0.01f;

            return xFree || yFree || zFree;
        }
    }
}
