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
            public double? LastTarget;     // drive units (deg or m), for velocity feed-forward
            public bool WrittenThisStep;
        }

        /// <summary>
        /// Velocity feed-forward: besides the position target, give each drive the target's speed (from the change
        /// since the previous physics step). With a damped drive this removes most of the lag behind a moving
        /// target. Joints that get no new target in a step have their target speed reset to 0 (see EndStep).
        /// </summary>
        public bool VelocityFeedForward = true;

        /// <summary>When set, EndStep appends one CSV row per physics step: per joint the commanded target,
        /// the drive's target velocity, and the measured position / velocity (URDF units: rad, m).</summary>
        public System.IO.TextWriter Trace;
        double _traceTime;

        readonly List<Entry> _entries = new List<Entry>();
        readonly Dictionary<string, Entry> _byName = new Dictionary<string, Entry>();
        readonly Dictionary<ArticulationBody, Entry> _byBody = new Dictionary<ArticulationBody, Entry>();
        readonly List<Joint> _joints = new List<Joint>();

        // Used when a joint's drive has no stiffness (e.g. robots whose importer sets gains only at runtime):
        // without it the joint would just go limp.
        public const float DefaultStiffness = 10000f, DefaultDamping = 1000f, DefaultForceLimit = 10000f;

        public ArticulationJointDriver(ArticulationBody root, IEnumerable<JointOverride> overrides,
                                       float defaultMaxVelocity)
        {
            int gainsApplied = 0;
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

                var xd = body.xDrive;
                if (xd.stiffness <= 0f)
                {
                    xd.stiffness = DefaultStiffness;
                    xd.damping = DefaultDamping;
                    xd.forceLimit = DefaultForceLimit;
                    body.xDrive = xd;
                    gainsApplied++;
                }

                var e = new Entry { Body = body, Joint = joint, Revolute = revolute };
                _entries.Add(e);
                _byName[name] = e;
                _byBody[body] = e;
                _joints.Add(joint);
            }
            if (gainsApplied > 0)
                Debug.Log($"[RemoteControl] {gainsApplied} joint(s) had no drive stiffness — applied stiffness " +
                          $"{DefaultStiffness}, damping {DefaultDamping}");
        }

        public IReadOnlyList<Joint> Joints => _joints;

        /// <summary>The commandable joint driven by this body, or null (fixed / unsupported joints).</summary>
        public Joint JointOf(ArticulationBody body) => body != null && _byBody.TryGetValue(body, out var e) ? e.Joint : null;

        void WriteTrace()
        {
            var ci = System.Globalization.CultureInfo.InvariantCulture;
            if (_traceTime == 0)
            {
                var header = new System.Text.StringBuilder("t");
                foreach (var e in _entries)
                    header.Append($",{e.Joint.Name}.cmd,{e.Joint.Name}.ffvel,{e.Joint.Name}.pos,{e.Joint.Name}.vel,{e.Joint.Name}.written");
                Trace.WriteLine(header.ToString());
            }
            _traceTime += Time.fixedDeltaTime;
            var row = new System.Text.StringBuilder(_traceTime.ToString("0.000", ci));
            foreach (var e in _entries)
            {
                double scale = e.Revolute ? Math.PI / 180.0 : 1.0;
                var d = e.Body.xDrive;
                double pos = e.Body.jointPosition.dofCount > 0 ? e.Body.jointPosition[0] : 0;
                double vel = e.Body.jointVelocity.dofCount > 0 ? e.Body.jointVelocity[0] : 0;
                row.Append(string.Format(ci, ",{0:0.#####},{1:0.#####},{2:0.#####},{3:0.#####},{4}",
                    d.target * scale, d.targetVelocity * scale, pos, vel, e.WrittenThisStep ? 1 : 0));
            }
            Trace.WriteLine(row.ToString());
        }

        public IReadOnlyDictionary<string, double> ReadPositions()
        {
            var d = new Dictionary<string, double>(_entries.Count);
            foreach (var e in _entries)
                d[e.Joint.Name] = e.Body.jointPosition.dofCount > 0 ? e.Body.jointPosition[0] : 0.0;  // rad / m
            return d;
        }

        public void WriteTargets(IReadOnlyDictionary<string, double> targets)
        {
            float dt = Time.fixedDeltaTime;
            foreach (var kv in targets)
            {
                if (!_byName.TryGetValue(kv.Key, out var e)) continue;
                double target = e.Revolute ? kv.Value * 180.0 / Math.PI : kv.Value;
                var drive = e.Body.xDrive;
                drive.target = (float)target;
                drive.targetVelocity = VelocityFeedForward && e.LastTarget.HasValue && dt > 0
                    ? (float)((target - e.LastTarget.Value) / dt) : 0f;
                e.Body.xDrive = drive;
                e.LastTarget = target;
                e.WrittenThisStep = true;
            }
        }

        /// <summary>Call once per physics step after the executor ran: joints that were not commanded this step
        /// stop being fed a target speed (otherwise the last one would keep pulling them along).</summary>
        public void EndStep()
        {
            if (Trace != null) WriteTrace();
            foreach (var e in _entries)
            {
                if (!e.WrittenThisStep)
                {
                    e.LastTarget = null;
                    var drive = e.Body.xDrive;
                    if (drive.targetVelocity != 0f)
                    {
                        drive.targetVelocity = 0f;
                        e.Body.xDrive = drive;
                    }
                }
                e.WrittenThisStep = false;
            }
        }
    }
}
