// A target the controller can move a robot's TCP to. Targets are ordinary scene objects:
//  - everything under the scene-root object "Targets" (leaves, or anything with this component), and
//  - any object anywhere with this component (e.g. attached to a part, so the target moves with it).
// They are saved with the scene like any other object. Robots report them to the controller in `describe`
// replies (pose in the scene, in the robot and in its root link), so the controller can list them and reach them.
//
// Ctrl+click on an object (Game view in Play mode, Scene view any time) creates a target on its surface.

using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace RobotMarket.RemoteControl.Unity
{
    /// <summary>How a target made by Ctrl+click is oriented.</summary>
    public enum ClickOrientation
    {
        [Tooltip("z into the surface (the tool approaches the part), x towards the robot")]
        Approach,
        [Tooltip("z out of the surface (surface normal), x along the view direction")]
        Surface,
        [Tooltip("The TCP's current orientation; only the position comes from the click")]
        KeepTcp,
    }

    [DisallowMultipleComponent]
    [SelectionBase]
    [AddComponentMenu("IVI Dynamic/Remote Control Target")]
    public sealed class RemoteControlTarget : MonoBehaviour
    {
        public const string RootName = "Targets";

        [Tooltip("Axis length of the drawn frame (m)")]
        [Min(0.01f)] public float size = 0.1f;

        /// <summary>New targets made by Ctrl+click become children of the clicked object (true) or of the
        /// "Targets" object (false). Set from the robot's Inspector or the controller.</summary>
        public static bool AttachNewTargets;

        /// <summary>Orientation of targets made by Ctrl+click (robot Inspector or controller setting).</summary>
        public static ClickOrientation NewTargetOrientation = ClickOrientation.Approach;

        static readonly List<RemoteControlTarget> _all = new List<RemoteControlTarget>();
        public static IReadOnlyList<RemoteControlTarget> All => _all;

        LineRenderer[] _axes;
        bool _highlighted;

        /// <summary>Stable id for the protocol: "target:" + path from the scene root.</summary>
        public string Id => "target:" + PathOf(transform);

        public static string PathOf(Transform t) => t.parent == null ? t.name : PathOf(t.parent) + "/" + t.name;

        void OnEnable()
        {
            if (!_all.Contains(this)) _all.Add(this);
            if (Application.isPlaying) BuildAxes();
        }

        void OnDisable()
        {
            _all.Remove(this);
            if (_axes != null)
                foreach (var a in _axes) if (a != null) Destroy(a.gameObject);
            _axes = null;
        }

        // ── finding ──────────────────────────────────────────────────────────

        /// <summary>The scene-root "Targets" object of a scene; created when `create` and missing.</summary>
        public static Transform Root(UnityEngine.SceneManagement.Scene scene, bool create)
        {
            foreach (var go in scene.GetRootGameObjects())
                if (go.name == RootName) return go.transform;
            if (!create) return null;
            var root = new GameObject(RootName);
            UnityEngine.SceneManagement.SceneManager.MoveGameObjectToScene(root, scene);
            return root.transform;
        }

        /// <summary>Give every leaf object under "Targets" this component, so it counts as a target.</summary>
        public static void AdoptChildrenOfRoot(UnityEngine.SceneManagement.Scene scene)
        {
            var root = Root(scene, false);
            if (root == null) return;
            foreach (var t in root.GetComponentsInChildren<Transform>(true))
                if (t != root && t.childCount == 0 && t.GetComponent<RemoteControlTarget>() == null
                    && t.GetComponent<Renderer>() == null)
                    t.gameObject.AddComponent<RemoteControlTarget>();
        }

        public static RemoteControlTarget Find(string id) => _all.FirstOrDefault(t => t != null && t.Id == id);

        /// <summary>"&lt;prefix&gt;_&lt;n&gt;" with the lowest n not used by another target: "Fixture_1", "Part3_2".</summary>
        public static string UniqueName(string prefix)
        {
            var sb = new System.Text.StringBuilder();
            foreach (var c in (prefix ?? "").Trim())
                sb.Append(char.IsLetterOrDigit(c) || c == '-' || c == '_' ? c : '_');
            var p = System.Text.RegularExpressions.Regex.Replace(sb.ToString(), "_{2,}", "_").Trim('_');
            if (p.Length == 0) p = "target";
            var names = new HashSet<string>(_all.Where(t => t != null).Select(t => t.name));
            for (int n = 1; ; n++)
                if (!names.Contains($"{p}_{n}")) return $"{p}_{n}";
        }

        // ── creating from a click ────────────────────────────────────────────

        /// <summary>What a click ray hits: point, surface normal and object. Uses colliders, then renderer bounds
        /// (imported meshes often have no collider). Robots' markers and targets are skipped.</summary>
        public static bool Pick(Ray ray, out Vector3 point, out Vector3 normal, out Transform hitObject)
        {
            point = normal = Vector3.zero;
            hitObject = null;
            float best = float.MaxValue;
            foreach (var hit in Physics.RaycastAll(ray, 10000f, ~0, QueryTriggerInteraction.Ignore))
            {
                if (Skip(hit.transform) || hit.distance >= best) continue;
                best = hit.distance;
                point = hit.point;
                normal = hit.normal;
                hitObject = hit.transform;
            }
            if (hitObject != null) return true;
            foreach (var r in FindObjectsByType<Renderer>(FindObjectsInactive.Exclude, FindObjectsSortMode.None))
            {
                if (!r.enabled || Skip(r.transform) || !r.bounds.IntersectRay(ray, out float d) || d >= best) continue;
                best = d;
                point = ray.GetPoint(d);
                normal = -ray.direction;
                hitObject = r.transform;
            }
            return hitObject != null;
        }

        static bool Skip(Transform t) =>
            t.GetComponentInParent<RemoteControlTarget>() != null || t.GetComponentInParent<RemoteControlMarker>() != null;

        /// <summary>Rotation of a new target at a surface point (Unity axes: forward = ROS x, up = ROS z).
        /// tcpRotation / robotPosition may be null (no TCP shown / no robot): Approach is used, x along the view.</summary>
        public static Quaternion Orientation(ClickOrientation mode, Vector3 point, Vector3 normal, Vector3 viewForward,
                                             Quaternion? tcpRotation, Vector3? robotPosition)
        {
            if (mode == ClickOrientation.KeepTcp && tcpRotation.HasValue) return tcpRotation.Value;
            var n = normal.sqrMagnitude > 1e-6f ? normal.normalized : Vector3.up;
            var up = mode == ClickOrientation.Surface ? n : -n;                       // ROS z
            var toward = mode != ClickOrientation.Surface && robotPosition.HasValue
                ? robotPosition.Value - point : viewForward;                          // ROS x
            var forward = Vector3.ProjectOnPlane(toward, up);
            if (forward.sqrMagnitude < 1e-6f) forward = Vector3.ProjectOnPlane(viewForward, up);
            if (forward.sqrMagnitude < 1e-6f) forward = Vector3.ProjectOnPlane(Vector3.forward, up);
            if (forward.sqrMagnitude < 1e-6f) forward = Vector3.ProjectOnPlane(Vector3.right, up);
            return Quaternion.LookRotation(forward.normalized, up);
        }

        /// <summary>Create a target at a surface point, named after the clicked object ("Fixture_1").</summary>
        public static RemoteControlTarget CreateAt(Vector3 point, Quaternion rotation, Transform hitObject, bool attach,
                                                   string name = null)
        {
            var scene = hitObject != null ? hitObject.gameObject.scene : UnityEngine.SceneManagement.SceneManager.GetActiveScene();
            var parent = attach && hitObject != null ? hitObject : Root(scene, true);
            var go = new GameObject(name ?? UniqueName(hitObject != null ? hitObject.name : "target"));
            go.transform.SetParent(parent, false);
            go.transform.SetPositionAndRotation(point, rotation);
            return go.AddComponent<RemoteControlTarget>();
        }

        // ── drawing ──────────────────────────────────────────────────────────

        static readonly Color AxisX = new Color(0.9f, 0.2f, 0.2f), AxisY = new Color(0.2f, 0.85f, 0.2f),
                              AxisZ = new Color(0.25f, 0.45f, 1f), Highlight = new Color(1f, 0.55f, 0f);
        static readonly Dictionary<Color, Material> _materials = new Dictionary<Color, Material>();

        static Material Mat(Color c)
        {
            if (_materials.TryGetValue(c, out var m) && m != null) return m;
            var shader = Shader.Find("Hidden/Internal-Colored") ?? Shader.Find("Universal Render Pipeline/Unlit");
            m = new Material(shader) { color = c, hideFlags = HideFlags.DontSave };
            if (m.HasProperty("_ZTest")) m.SetInt("_ZTest", (int)UnityEngine.Rendering.CompareFunction.Always);
            if (m.HasProperty("_ZWrite")) m.SetInt("_ZWrite", 0);
            m.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Overlay;
            _materials[c] = m;
            return m;
        }

        void BuildAxes()
        {
            var dirs = new[] { Vector3.forward, Vector3.left, Vector3.up };   // ROS x, y, z in Unity
            var colors = new[] { AxisX, AxisY, AxisZ };
            _axes = new LineRenderer[3];
            for (int i = 0; i < 3; i++)
            {
                var go = new GameObject("axis") { hideFlags = HideFlags.DontSave | HideFlags.HideInHierarchy };
                go.transform.SetParent(transform, false);
                var lr = go.AddComponent<LineRenderer>();
                lr.useWorldSpace = false;
                lr.sharedMaterial = Mat(colors[i]);
                lr.positionCount = 2;
                lr.SetPosition(0, Vector3.zero);
                lr.SetPosition(1, dirs[i] * size);
                lr.widthMultiplier = 0.004f;
                lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                lr.receiveShadows = false;
                _axes[i] = lr;
            }
            SetHighlighted(_highlighted);
        }

        public void SetHighlighted(bool on)
        {
            _highlighted = on;
            if (_axes == null) return;
            foreach (var a in _axes) if (a != null) a.widthMultiplier = on ? 0.008f : 0.004f;
        }

        void OnDrawGizmos()
        {
            if (Application.isPlaying) return;   // drawn with lines in Play mode
            var t = transform;
            Gizmos.color = AxisX; Gizmos.DrawLine(t.position, t.position + t.forward * size);
            Gizmos.color = AxisY; Gizmos.DrawLine(t.position, t.position - t.right * size);
            Gizmos.color = AxisZ; Gizmos.DrawLine(t.position, t.position + t.up * size);
            Gizmos.color = Color.white; Gizmos.DrawSphere(t.position, size * 0.06f);
        }

        void OnDrawGizmosSelected()
        {
            Gizmos.color = Highlight;
            Gizmos.DrawWireSphere(transform.position, size * 0.15f);
        }
    }
}
