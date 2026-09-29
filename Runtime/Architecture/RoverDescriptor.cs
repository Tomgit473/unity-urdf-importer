using System.Collections.Generic;
using UnityEngine;

namespace RoverCompatibility
{
    public enum RobotArchitectureType
    {
        WheeledMobile,      // Skid-steer / differential / rocker-bogie / steered rovers (>= 2 wheels)
        ArticulatedLegged,  // Quadruped, Biped, Hexapod (hip, thigh, calf, knee, foot)
        RoboticArm,         // Manipulator chains (shoulder, elbow, wrist, gripper)
        GeneralArticulated  // Any other multi-joint ArticulationBody tree
    }

    public enum KnownRobotModel
    {
        Generic,
        ClearpathHusky,
        DeepRoboticsM20,
        NasaPerseverance,
        UnitreeGo2W,
        UnitreeGo2,
        UnitreeGo1,
        AnymalD,
        BostonDynamicsSpot,
        GoogleBarkour,
        PickerbotMini,
        RbvoguiMobileRover,
        UpkieWheeledBiped,
        RoboticArm
    }

    /// <summary>
    /// Holds the structural analysis of an imported robot: the identified chassis/base,
    /// detected continuous/revolute wheels, steering knuckles, suspension/rocker-bogie joints,
    /// non-wheel articulated joints, root link, architecture classification, and diagnostic warnings.
    /// </summary>
    public class RoverDescriptor
    {
        public GameObject Root;
        public ArticulationBody RootArticulationBody;
        public ArticulationBody Chassis;
        public RobotArchitectureType RobotType = RobotArchitectureType.GeneralArticulated;
        public KnownRobotModel ModelType = KnownRobotModel.Generic;
        public List<ArticulationBody> Wheels = new List<ArticulationBody>();
        public List<ArticulationBody> SteeringJoints = new List<ArticulationBody>();
        public List<ArticulationBody> SuspensionJoints = new List<ArticulationBody>();
        public List<ArticulationBody> NonWheelJoints = new List<ArticulationBody>();
        public List<ArticulationBody> AllBodies = new List<ArticulationBody>();
        public List<string> Warnings = new List<string>();

        /// <summary>
        /// Universal compatibility: Compatible if it has >= 2 wheels (mobile rover)
        /// OR if it has a valid ArticulationBody hierarchy of at least 1 link (legged, arm, or generic).
        /// </summary>
        public bool IsCompatible => (Wheels != null && Wheels.Count >= 2 && Chassis != null) || (AllBodies != null && AllBodies.Count >= 1);

        public float TotalMass
        {
            get
            {
                if (Root == null) return 0f;
                float mass = 0f;
                var bodies = Root.GetComponentsInChildren<ArticulationBody>(true);
                foreach (var b in bodies)
                {
                    mass += b.mass;
                }
                return mass;
            }
        }
    }
}
