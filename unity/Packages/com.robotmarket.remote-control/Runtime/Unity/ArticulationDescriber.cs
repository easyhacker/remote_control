// Builds the geometry part of a `description` reply from an ArticulationBody chain: where the robot is
// (base_pose), every link's pose, and the kinematic tree (joint origins, axes, limits, positions).
//
// Everything is converted to the ROS / URDF convention (x forward, y left, z up, right-handed), so the
// result can be compared with the robot's URDF and used by ROS tools. Joint origins come from the
// articulation anchors (the joint frame at zero position), not from the current pose, so they match the URDF.

using System;
using System.Linq;
using System.Reflection;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace RobotMarket.RemoteControl.Unity
{
    public static class ArticulationDescriber
    {
        /// <summary>Unity (x right, y up, z forward; left-handed) → ROS (x forward, y left, z up).</summary>
        public static Vector3 ToRos(Vector3 v) => new Vector3(v.z, -v.x, v.y);

        public static Quaternion ToRos(Quaternion q) => new Quaternion(-q.z, q.x, -q.y, q.w);

        static JArray Arr(Vector3 v) => new JArray(R(v.x), R(v.y), R(v.z));
        static JArray Arr(Quaternion q) => new JArray(R(q.x), R(q.y), R(q.z), R(q.w));
        static double R(double x) => Math.Round(x, 6);

        public static JObject Pose(Vector3 position, Quaternion rotation) => new JObject
        {
            ["position"] = Arr(ToRos(position)),
            ["orientation"] = Arr(ToRos(rotation)),
        };

        /// <summary>URDF rpy (fixed-axis roll x, pitch y, yaw z) of a ROS-frame rotation.</summary>
        public static Vector3 Rpy(Quaternion q)
        {
            double x = q.x, y = q.y, z = q.z, w = q.w;
            double n = Math.Sqrt(x * x + y * y + z * z + w * w);
            x /= n; y /= n; z /= n; w /= n;
            // rotation matrix of R = Rz(yaw) · Ry(pitch) · Rx(roll)
            double r00 = 1 - 2 * (y * y + z * z), r10 = 2 * (x * y + w * z), r20 = 2 * (x * z - w * y);
            double r21 = 2 * (y * z + w * x), r22 = 1 - 2 * (x * x + y * y);
            double r11 = 1 - 2 * (x * x + z * z), r12 = 2 * (y * z - w * x);
            double cp = Math.Sqrt(r00 * r00 + r10 * r10);
            double pitch = Math.Atan2(-r20, cp);
            double roll, yaw;
            if (cp > 1e-5)
            {
                roll = Math.Atan2(r21, r22);
                yaw = Math.Atan2(r10, r00);
            }
            else   // pitch = ±90°: only roll ∓ yaw is defined; put it all in roll (URDF exporters do the same)
            {
                yaw = 0;
                roll = Math.Atan2(-r12, r11);
            }
            return new Vector3((float)roll, (float)pitch, (float)yaw);
        }

        public static JObject Describe(ArticulationBody root, ArticulationJointDriver driver, bool tree,
                                       GameObject robotObject)
        {
            var d = new JObject
            {
                ["model"] = robotObject != null ? robotObject.name : root.name,
                ["base_pose"] = Pose(root.transform.position, root.transform.rotation),
            };
            if (!tree) return d;

            var pos = driver.ReadPositions();
            var bodies = root.GetComponentsInChildren<ArticulationBody>(true);
            d["root"] = root.name;
            d["links"] = new JArray(bodies.Select(b => new JObject
            {
                ["name"] = b.name,
                ["pose"] = Pose(b.transform.position, b.transform.rotation),
            }));

            var joints = new JArray();
            foreach (var b in bodies)
            {
                if (b == root) continue;
                var parent = b.transform.parent != null ? b.transform.parent.GetComponentInParent<ArticulationBody>() : null;
                var joint = driver.JointOf(b);
                string type;
                switch (b.jointType)
                {
                    case ArticulationJointType.RevoluteJoint:
                        type = b.twistLock == ArticulationDofLock.FreeMotion ? "continuous" : "revolute"; break;
                    case ArticulationJointType.PrismaticJoint: type = "prismatic"; break;
                    case ArticulationJointType.SphericalJoint: type = "spherical"; break;
                    default: type = "fixed"; break;
                }

                // Child pose relative to the parent, in ROS coordinates (current joint position included)
                var pRot = ToRos(parent != null ? parent.transform.rotation : Quaternion.identity);
                var pPos = ToRos(parent != null ? parent.transform.position : Vector3.zero);
                var pInv = Quaternion.Inverse(pRot);
                var relRot = pInv * ToRos(b.transform.rotation);
                var relPos = pInv * (ToRos(b.transform.position) - pPos);

                // The joint moves along / about the anchor's x axis, given in the child link's frame.
                // URDF Importer revolute joints point the anchor at -axis (Unity is left-handed), so a positive
                // joint position turns the URDF way round the negated vector; prismatic joints use +axis.
                bool moving = type == "revolute" || type == "continuous" || type == "prismatic";
                var axis = ToRos(b.anchorRotation * Vector3.right).normalized;
                if (type != "prismatic") axis = -axis;
                double q = moving && b.jointPosition.dofCount > 0 ? b.jointPosition[0] : 0.0;

                // URDF origin = the relative pose at joint position 0: undo the current motion
                // (child = origin · motion, with motion = rotation q about the axis or translation q along it)
                Quaternion originRot = relRot;
                Vector3 originPos = relPos;
                if (type == "revolute" || type == "continuous")
                {
                    float s = (float)Math.Sin(-q / 2), c = (float)Math.Cos(-q / 2);
                    originRot = relRot * new Quaternion(axis.x * s, axis.y * s, axis.z * s, c);
                }
                else if (type == "prismatic")
                {
                    originPos = relPos - relRot * (axis * (float)q);
                }
                var j = new JObject
                {
                    ["name"] = UrdfJointName(b) ?? b.name,
                    ["type"] = type,
                    ["parent"] = parent != null ? parent.name : null,
                    ["child"] = b.name,
                    ["origin"] = new JObject { ["xyz"] = Arr(originPos), ["rpy"] = Arr(Rpy(originRot)) },
                };
                if (moving)
                {
                    j["axis"] = Arr(axis);
                    j["lower"] = joint?.Lower;
                    j["upper"] = joint?.Upper;
                    j["max_velocity"] = joint?.MaxVelocity;
                    j["command_name"] = joint?.Name;
                    j["position"] = joint != null && pos.TryGetValue(joint.Name, out var x) ? R(x) : (double?)null;
                }
                else
                {
                    j["command_name"] = null;
                }
                joints.Add(j);
            }
            d["joints"] = joints;
            return d;
        }

        /// <summary>The URDF joint name kept by the URDF Importer (UrdfJoint.jointName), found by reflection
        /// so this package doesn't depend on the importer.</summary>
        static string UrdfJointName(ArticulationBody body)
        {
            foreach (var c in body.GetComponents<Component>())
            {
                if (c == null) continue;
                for (var t = c.GetType(); t != null && t != typeof(MonoBehaviour); t = t.BaseType)
                {
                    if (t.Name != "UrdfJoint") continue;
                    var f = t.GetField("jointName", BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
                    if (f?.GetValue(c) is string s && !string.IsNullOrEmpty(s)) return s;
                }
            }
            return null;
        }
    }
}
