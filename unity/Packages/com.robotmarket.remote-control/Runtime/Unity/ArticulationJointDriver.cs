// IJointDriver for Unity physics articulations (ArticulationBody), e.g. robots from the URDF Importer.
// Every single-DOF revolute or prismatic ArticulationBody under the root becomes a joint, named after its
// GameObject (override names, limits and max velocity per joint in RemoteControlRobot's inspector).
// Units on the wire: radians / meters. Unity drives use degrees for revolute targets.

using System;
using System.Collections.Generic;
using UnityEngine;

namespace RobotMarket.RemoteControl.Unity
{
    [Serializable]
    public class JointOverride
    {
        [Tooltip("GameObject name of the ArticulationBody")]
        public string bodyName;
        [Tooltip("Joint name used on the wire (empty = GameObject name)")]
        public string jointName;
        public bool overrideLimits;
        [Tooltip("rad or m")] public float lower;
        [Tooltip("rad or m")] public float upper;
        [Tooltip("rad/s or m/s; 0 = use the default")] public float maxVelocity;
    }

    public sealed class ArticulationJointDriver : IJointDriver
    {
        sealed class Entry
        {
            public ArticulationBody Body;
            public Joint Joint;
            public bool Revolute;
        }

        readonly List<Entry> _entries = new List<Entry>();
        readonly Dictionary<string, Entry> _byName = new Dictionary<string, Entry>();
        readonly List<Joint> _joints = new List<Joint>();

        public ArticulationJointDriver(ArticulationBody root, IEnumerable<JointOverride> overrides,
                                       float defaultMaxVelocity)
        {
            var map = new Dictionary<string, JointOverride>();
            if (overrides != null)
                foreach (var o in overrides)
                    if (o != null && !string.IsNullOrEmpty(o.bodyName)) map[o.bodyName] = o;

            foreach (var body in root.GetComponentsInChildren<ArticulationBody>(true))
            {
                bool revolute = body.jointType == ArticulationJointType.RevoluteJoint;
                bool prismatic = body.jointType == ArticulationJointType.PrismaticJoint;
                if (!revolute && !prismatic) continue;

                map.TryGetValue(body.gameObject.name, out var o);
                var name = !string.IsNullOrEmpty(o?.jointName) ? o.jointName : body.gameObject.name;
                if (_byName.ContainsKey(name))
                {
                    Debug.LogWarning($"[RemoteControl] duplicate joint name '{name}' — skipping {body.name}; set a jointName override");
                    continue;
                }

                var joint = new Joint { Name = name, Type = revolute ? "revolute" : "prismatic" };
                if (o != null && o.overrideLimits)
                {
                    joint.Lower = o.lower;
                    joint.Upper = o.upper;
                }
                else
                {
                    var drive = body.xDrive;
                    bool limited = revolute ? body.twistLock == ArticulationDofLock.LimitedMotion
                                            : body.linearLockX == ArticulationDofLock.LimitedMotion;
                    if (limited)
                    {
                        double scale = revolute ? Math.PI / 180.0 : 1.0;
                        joint.Lower = drive.lowerLimit * scale;
                        joint.Upper = drive.upperLimit * scale;
                    }
                }
                float vmax = o != null && o.maxVelocity > 0 ? o.maxVelocity : defaultMaxVelocity;
                joint.MaxVelocity = vmax > 0 ? vmax : (double?)null;

                var e = new Entry { Body = body, Joint = joint, Revolute = revolute };
                _entries.Add(e);
                _byName[name] = e;
                _joints.Add(joint);
            }
        }

        public IReadOnlyList<Joint> Joints => _joints;

        public IReadOnlyDictionary<string, double> ReadPositions()
        {
            var d = new Dictionary<string, double>(_entries.Count);
            foreach (var e in _entries)
                d[e.Joint.Name] = e.Body.jointPosition.dofCount > 0 ? e.Body.jointPosition[0] : 0.0;  // rad / m
            return d;
        }

        public void WriteTargets(IReadOnlyDictionary<string, double> targets)
        {
            foreach (var kv in targets)
            {
                if (!_byName.TryGetValue(kv.Key, out var e)) continue;
                var drive = e.Body.xDrive;
                drive.target = (float)(e.Revolute ? kv.Value * 180.0 / Math.PI : kv.Value);
                e.Body.xDrive = drive;
            }
        }
    }
}
