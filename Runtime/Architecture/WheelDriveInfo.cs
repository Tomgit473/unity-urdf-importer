using System;
using UnityEngine;

namespace RoverCompatibility
{
    public enum WheelAxis { X, Y, Z }

    /// <summary>
    /// Represents a detected rover wheel paired with its active ArticulationDrive axis.
    /// Being a serializable class allows it to be inspected, adjusted, or fine-tuned
    /// directly in the Unity Inspector on the SimpleDifferentialDrive component.
    /// </summary>
    [Serializable]
    public class WheelDriveInfo
    {
        public string wheelName;
        public ArticulationBody body;
        public WheelAxis axis = WheelAxis.X;
        public bool isRightSide;
        public bool invertDirection;

        public WheelDriveInfo() { }

        public WheelDriveInfo(ArticulationBody body, WheelAxis axis, bool isRightSide, bool invertDirection = false)
        {
            this.body = body;
            this.wheelName = body != null ? body.name : "Unknown Wheel";
            this.axis = axis;
            this.isRightSide = isRightSide;
            this.invertDirection = invertDirection;
        }

        public override string ToString()
        {
            return $"Wheel '{wheelName}' [Axis={axis}, Side={(isRightSide ? "Right" : "Left")}, Invert={invertDirection}]";
        }
    }
}

