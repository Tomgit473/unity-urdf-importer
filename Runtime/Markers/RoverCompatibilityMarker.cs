using System;
using UnityEngine;

namespace RoverCompatibility
{
    /// <summary>
    /// Marks a robot that has been processed by the Rover & Robot Compatibility System.
    /// Prevents duplicate auto-configuration passes while storing configuration metadata.
    /// </summary>
    [DisallowMultipleComponent]
    public class RoverCompatibilityMarker : MonoBehaviour
    {
        [SerializeField] public string configuredAt = "";
        [SerializeField] public string architectureType = "WheeledMobile";
        [SerializeField] public int wheelCount = 0;
        [SerializeField] public int jointCount = 0;
        [SerializeField] public string chassisName = "";
        [SerializeField] public float calculatedTotalMass = 0f;

        public void Stamp(int wheels, string chassis, float mass)
        {
            configuredAt = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss");
            architectureType = wheels >= 3 ? "WheeledMobile" : "Articulated";
            wheelCount = wheels;
            jointCount = 0;
            chassisName = chassis;
            calculatedTotalMass = mass;
        }

        public void Stamp(string archType, int joints, int wheels, string chassis, float mass)
        {
            configuredAt = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss");
            architectureType = archType;
            jointCount = joints;
            wheelCount = wheels;
            chassisName = chassis;
            calculatedTotalMass = mass;
        }
    }
}
