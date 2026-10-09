// Kinematic differential drive: turning the drive wheels moves the whole robot over the floor.
//
// The wheels are ordinary revolute joints (driven by Remote Control Robot, Joint Target Controller, or anything else).
// Every physics step this component measures how far each drive wheel actually turned, rolls it without slip, and
// moves the articulation root by the resulting rigid motion in the floor plane (forward from the mean wheel travel,
// turning from the difference). No wheel-floor physics: the base goes exactly where the wheels say, which keeps
// positions repeatable. Swivel casters are steered to follow the motion.
//
// Geometry comes from the model: each wheel's axle direction and position are read from the scene, so the track width
// and the sign of each wheel need no setup; the wheel radius is measured from the wheel's renderers (or set by hand).
// The articulation root stays Immovable - it is teleported, not pushed.

using System.Collections.Generic;
using UnityEngine;

namespace RobotMarket.RemoteControl.Unity
{
    [AddComponentMenu("IVI Dynamic/Mobile Base Drive")]
    [DefaultExecutionOrder(100)]    // after the joint targets of this step were written
    public class MobileBaseDrive : MonoBehaviour
    {
        [Tooltip("Left drive wheel: a revolute ArticulationBody (found automatically by name when empty)")]
        public ArticulationBody leftWheel;
        [Tooltip("Right drive wheel: a revolute ArticulationBody (found automatically by name when empty)")]
        public ArticulationBody rightWheel;
        [Tooltip("Wheel radius in metres; 0 = measure it from the wheel's renderers")]
        public float wheelRadius = 0f;
        [Tooltip("Swivel casters (revolute joints about the vertical) to steer along the motion; found by name when empty")]
        public List<ArticulationBody> casters = new List<ArticulationBody>();
        [Tooltip("Steer the casters to follow the motion")]
        public bool steerCasters = true;

        /// <summary>The rigid motion the base has made since Play started (rotation about the vertical, then
        /// translation): a point p of the robot at the start is now at TravelRotation * p + TravelOffset.</summary>
        public Quaternion TravelRotation { get; private set; } = Quaternion.identity;
        public Vector3 TravelOffset { get; private set; } = Vector3.zero;

        ArticulationBody _root;
        float _radius;
        Quaternion _lastLeft, _lastRight;     // wheel rotation relative to its parent body, last step
        bool _ready;

        void Reset() => FindParts();

        void FindParts()
        {
            foreach (var body in GetComponentsInChildren<ArticulationBody>(true))
            {
                if (body.jointType != ArticulationJointType.RevoluteJoint) continue;
                var n = body.name.ToLowerInvariant();
                if (n.Contains("caster"))
                {
                    if (!casters.Contains(body)) casters.Add(body);
                }
                else if (n.Contains("wheel"))
                {
                    if (leftWheel == null && (n.EndsWith("_l") || n.Contains("left"))) leftWheel = body;
                    else if (rightWheel == null && (n.EndsWith("_r") || n.Contains("right"))) rightWheel = body;
                }
            }
        }

        void Start()
        {
            if (leftWheel == null || rightWheel == null) FindParts();
            foreach (var body in GetComponentsInChildren<ArticulationBody>(true))
                if (body.isRoot) { _root = body; break; }
            if (leftWheel == null || rightWheel == null || _root == null)
            {
                Debug.LogError("[RemoteControl] Mobile Base Drive needs a left and a right wheel (revolute joints) under " +
                               "an articulation root - set them in the inspector", this);
                enabled = false;
                return;
            }
            _radius = wheelRadius > 0f ? wheelRadius : MeasureRadius(leftWheel);
            Debug.Log($"[RemoteControl] Mobile Base Drive: wheels {leftWheel.name} / {rightWheel.name}, radius " +
                      $"{_radius:0.###} m, track {Vector3.Distance(leftWheel.transform.position, rightWheel.transform.position):0.###} m, " +
                      $"{casters.Count} caster(s)", this);
        }

        /// <summary>For the robot's description ("mobile_base", see PROTOCOL.md): which joints are the wheels and how
        /// they move the robot, in the ROS convention of the rest of the description. Null before Start.</summary>
        public Newtonsoft.Json.Linq.JObject Describe(ArticulationJointDriver driver)
        {
            if (!enabled || _root == null || leftWheel == null || rightWheel == null) return null;
            string Name(ArticulationBody b) => driver?.JointOf(b)?.Name ?? b.name;
            Vector3 pL = leftWheel.transform.position, pR = rightWheel.transform.position;
            // forward: perpendicular to the axle line, with the left wheel on the left (Unity is left-handed)
            var forward = Vector3.Cross(Vector3.ProjectOnPlane(pR - pL, Vector3.up), Vector3.up).normalized;
            int Sign(ArticulationBody wheel)
            {
                // a positive joint angle turns the wheel about its anchor X axis; where does its centre roll?
                var axis = wheel.transform.rotation * (wheel.anchorRotation * Vector3.right);
                var roll = Quaternion.AngleAxis(1f, axis) * Vector3.up - Vector3.up;
                return Vector3.Dot(roll, forward) >= 0f ? 1 : -1;
            }
            var forwardInBase = ArticulationDescriber.ToRos(Quaternion.Inverse(_root.transform.rotation) * forward);
            // the point it turns about in place: midway between the wheels, on the floor (base frame)
            var mid = Vector3.ProjectOnPlane((pL + pR) * 0.5f - _root.transform.position, Vector3.up) + _root.transform.position;
            var centerInBase = ArticulationDescriber.ToRos(Quaternion.Inverse(_root.transform.rotation) * (mid - _root.transform.position));
            var casterNames = new Newtonsoft.Json.Linq.JArray();
            foreach (var c in casters) if (c != null) casterNames.Add(Name(c));
            return new Newtonsoft.Json.Linq.JObject
            {
                ["type"] = "differential",
                ["left_wheel"] = Name(leftWheel),
                ["right_wheel"] = Name(rightWheel),
                ["wheel_radius"] = _radius,
                ["track"] = Vector3.Distance(Vector3.ProjectOnPlane(pL, Vector3.up), Vector3.ProjectOnPlane(pR, Vector3.up)),
                ["left_sign"] = Sign(leftWheel),       // +1: a positive wheel angle drives the robot forward
                ["right_sign"] = Sign(rightWheel),
                ["forward"] = new Newtonsoft.Json.Linq.JArray(forwardInBase.x, forwardInBase.y, forwardInBase.z),
                ["center"] = new Newtonsoft.Json.Linq.JArray(centerInBase.x, centerInBase.y, centerInBase.z),
                ["casters"] = casterNames,
            };
        }

        /// <summary>Radius from the wheel's renderers: an upright wheel's height is its diameter.</summary>
        static float MeasureRadius(ArticulationBody wheel)
        {
            var renderers = wheel.GetComponentsInChildren<Renderer>();
            if (renderers.Length == 0) return 0.05f;
            var b = renderers[0].bounds;
            foreach (var r in renderers) b.Encapsulate(r.bounds);
            return Mathf.Max(0.001f, b.size.y * 0.5f);
        }

        static Quaternion Relative(ArticulationBody wheel)
        {
            var parent = wheel.transform.parent;
            return parent != null ? Quaternion.Inverse(parent.rotation) * wheel.transform.rotation : wheel.transform.rotation;
        }

        /// <summary>Where the wheel's centre moved this step when it rolls without slipping: its rotation since the
        /// last step (in world space, relative to the base) turns the vector from the floor contact to the centre.</summary>
        Vector3 Roll(ArticulationBody wheel, ref Quaternion last)
        {
            var now = Relative(wheel);
            var delta = now * Quaternion.Inverse(last);            // in the parent body's frame
            last = now;
            var parent = wheel.transform.parent;
            var q = parent != null ? parent.rotation * delta * Quaternion.Inverse(parent.rotation) : delta;
            var up = Vector3.up * _radius;
            var d = q * up - up;
            return Vector3.ProjectOnPlane(d, Vector3.up);
        }

        void FixedUpdate()
        {
            if (!_ready)
            {
                _lastLeft = Relative(leftWheel);
                _lastRight = Relative(rightWheel);
                _ready = true;
                return;
            }
            var dL = Roll(leftWheel, ref _lastLeft);
            var dR = Roll(rightWheel, ref _lastRight);
            if (dL.sqrMagnitude < 1e-14f && dR.sqrMagnitude < 1e-14f) return;

            // rigid planar motion that carries both wheel centres: turn about the vertical, then translate
            Vector3 pL = Vector3.ProjectOnPlane(leftWheel.transform.position, Vector3.up);
            Vector3 pR = Vector3.ProjectOnPlane(rightWheel.transform.position, Vector3.up);
            float angle = Vector3.SignedAngle(pR - pL, (pR + dR) - (pL + dL), Vector3.up);
            var turn = Quaternion.AngleAxis(angle, Vector3.up);
            Vector3 mid = (pL + pR) * 0.5f;
            Vector3 move = (dL + dR) * 0.5f;

            var rootPos = _root.transform.position;
            var pivot = new Vector3(mid.x, rootPos.y, mid.z);
            var newPos = pivot + move + turn * (rootPos - pivot);
            _root.TeleportRoot(newPos, turn * _root.transform.rotation);

            // the same motion for TravelRotation / TravelOffset: p' = turn * (p - pivot) + pivot + move
            TravelRotation = turn * TravelRotation;
            TravelOffset = turn * (TravelOffset - pivot) + pivot + move;

            if (steerCasters) SteerCasters(turn, pivot, move);
        }

        /// <summary>Turn each caster so its wheel rolls along the direction its pivot moves (either way round).</summary>
        void SteerCasters(Quaternion turn, Vector3 pivot, Vector3 move)
        {
            foreach (var caster in casters)
            {
                if (caster == null || caster.jointPosition.dofCount == 0) continue;
                var p = caster.transform.position;
                var v = Vector3.ProjectOnPlane(pivot + move + turn * (p - pivot) - p, Vector3.up);
                if (v.sqrMagnitude < 1e-12f) continue;
                // the caster wheel's axle is the caster link's X axis (URDF cylinders along X); it rolls across it
                var axle = caster.transform.right;
                var roll = Vector3.ProjectOnPlane(Vector3.Cross(Vector3.up, axle), Vector3.up);
                if (roll.sqrMagnitude < 1e-12f) continue;
                float delta = Vector3.SignedAngle(roll, v, Vector3.up);
                if (delta > 90f) delta -= 180f;                     // rolling backwards is as good as forwards
                else if (delta < -90f) delta += 180f;
                var swivelAxis = caster.transform.TransformDirection(caster.anchorRotation * Vector3.right);
                float sign = Vector3.Dot(swivelAxis, Vector3.up) >= 0f ? 1f : -1f;
                var drive = caster.xDrive;
                drive.target = caster.jointPosition[0] * Mathf.Rad2Deg + sign * delta;
                drive.targetVelocity = 0f;
                caster.xDrive = drive;
            }
        }
    }
}
