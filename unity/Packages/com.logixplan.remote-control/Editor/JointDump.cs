// Menu: IVI Dynamic → Remote Control → Debug: Dump Joints of Selection
// Writes the physics settings of every ArticulationBody under the selected object (mass, damping, friction,
// velocity caps, drive gains and limits) plus the project physics settings to Logs/joint_dump.txt — for
// diagnosing joints that lag, overshoot or don't move.

using System.Globalization;
using System.IO;
using System.Text;
using UnityEditor;
using UnityEngine;

namespace RobotMarket.RemoteControl.Editor
{
    public static class JointDump
    {
        [MenuItem("IVI Dynamic/Remote Control/Debug: Dump Joints of Selection")]
        static void Dump()
        {
            var root = Selection.activeGameObject;
            if (root == null)
            {
                EditorUtility.DisplayDialog("Dump Joints", "Select a robot in the Hierarchy first.", "OK");
                return;
            }
            var ci = CultureInfo.InvariantCulture;
            var sb = new StringBuilder();
            sb.AppendLine(string.Format(ci, "project physics: fixedDeltaTime={0} gravity={1} solverIterations={2} solverVelocityIterations={3} " +
                                            "defaultMaxAngularSpeed={4} defaultMaxDepenetrationVelocity={5}",
                Time.fixedDeltaTime, Physics.gravity, Physics.defaultSolverIterations, Physics.defaultSolverVelocityIterations,
                Physics.defaultMaxAngularSpeed, Physics.defaultMaxDepenetrationVelocity));
            foreach (var b in root.GetComponentsInChildren<ArticulationBody>(true))
            {
                var d = b.xDrive;
                sb.AppendLine(string.Format(ci,
                    "{0}  type={1} root={2} immovable={3} enabled={4} active={5}\n" +
                    "   mass={6:0.###} useGravity={7} linearDamping={8} angularDamping={9} jointFriction={10}\n" +
                    "   maxJointVelocity={11} maxAngularVelocity={12} maxLinearVelocity={13} maxDepenetrationVelocity={14}\n" +
                    "   twistLock={15} linearLockX={16}  xDrive: stiffness={17} damping={18} forceLimit={19} driveType={20} " +
                    "lower={21} upper={22} target={23} targetVelocity={24}\n" +
                    "   solverIterations={25} solverVelocityIterations={26} collisionDetection={27}",
                    b.name, b.jointType, b.isRoot, b.immovable, b.enabled, b.gameObject.activeInHierarchy,
                    b.mass, b.useGravity, b.linearDamping, b.angularDamping, b.jointFriction,
                    b.maxJointVelocity, b.maxAngularVelocity, b.maxLinearVelocity, b.maxDepenetrationVelocity,
                    b.twistLock, b.linearLockX, d.stiffness, d.damping, d.forceLimit, d.driveType,
                    d.lowerLimit, d.upperLimit, d.target, d.targetVelocity,
                    b.solverIterations, b.solverVelocityIterations, b.collisionDetectionMode));
            }
            Directory.CreateDirectory("Logs");
            File.WriteAllText("Logs/joint_dump.txt", sb.ToString());
            Debug.Log($"[RemoteControl] joint dump of '{root.name}' written to Logs/joint_dump.txt");
        }
    }
}
