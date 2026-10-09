// Draws what the controller sends in `visualize` messages: coordinate frames (TCP, targets, user frames)
// and kinematic chains, attached to the robot's links so they move with it. Each marker is a named GameObject in
// the scene ("TCP_right_arm" under the end link, "Frame_pick", …).
//  - selectable frames can be clicked in the Game view (or selected in the Hierarchy / Scene view): the robot
//    reports `selected` to the controller;
//  - editable frames (TCP, user frames) can be moved / rotated with Unity's tools in the Scene view: when the user
//    stops dragging, the robot reports `edited` with the new pose in the parent link.
//
// Item format (ROS convention: x forward, y left, z up, metres, quaternion x y z w), see PROTOCOL.md:
//   { "id": "frame:pick", "kind": "frame", "parent": "base", "pose": {"position": [..], "orientation": [..]},
//     "name": "Frame_pick", "label": "pick", "size": 0.1, "style": "target", "selectable": true, "editable": true }
//   { "id": "chain:R", "kind": "chain", "links": ["base", "R_shoulder", ...], "end": "tcp:R", "style": "chain" }
//   { "id": "frame:pick", "remove": true }

using System;
using System.Collections.Generic;
using System.Linq;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace RobotMarket.RemoteControl.Unity
{
    /// <summary>On every marker GameObject: lets the Editor map a Hierarchy / Scene selection back to the item.
    /// SelectionBase: clicking one of its axis lines in the Scene view selects the whole frame, so the Move / Rotate
    /// tools move the frame.</summary>
    [AddComponentMenu("")]
    [SelectionBase]
    public sealed class RemoteControlMarker : MonoBehaviour
    {
        public string itemId;
        public string label;
        public bool selectable;
        public bool editable;
        [NonSerialized] public RemoteControlRobot owner;
    }

    public sealed class MarkerLayer
    {
        sealed class Item
        {
            public string Id, Kind, Label, Style, End, Parent;
            public bool Selectable, Editable;
            public Vector3 SetPosition;       // local pose last set by the controller or reported to it
            public Quaternion SetRotation;
            public double ChangedAt = -1;     // realtime of the last unreported user change (-1: none)
            public GameObject Go;
            public LineRenderer[] Axes;       // frame: x, y, z
            public Transform Handle;          // frame: small sphere at the origin (easy to see and grab)
            public LineRenderer Line;         // chain
            public string[] Links;
            public float Size;
        }

        readonly RemoteControlRobot _owner;
        readonly Transform _robotRoot;
        readonly Dictionary<string, Item> _items = new Dictionary<string, Item>();
        readonly Dictionary<string, Transform> _links = new Dictionary<string, Transform>();
        readonly Dictionary<Color, Material> _materials = new Dictionary<Color, Material>();
        GameObject _container;
        GUIStyle _labelStyle;
        string _focusRequest;
        public string SelectedId { get; private set; }

        /// <summary>The user moved an editable frame: (item id, parent link, pose in the parent, ROS convention).</summary>
        public event Action<string, string, JObject> Edited;

        /// <summary>Seconds without further change before an edit is reported (the user stopped dragging).</summary>
        public double EditSettleTime = 0.3;

        static readonly Color AxisX = new Color(0.9f, 0.2f, 0.2f), AxisY = new Color(0.2f, 0.85f, 0.2f),
                              AxisZ = new Color(0.25f, 0.45f, 1f), ChainColor = new Color(1f, 0.8f, 0.15f),
                              SelectedColor = new Color(1f, 0.55f, 0f);

        public MarkerLayer(RemoteControlRobot owner)
        {
            _owner = owner;
            _robotRoot = owner.transform;
        }

        // ── ROS → Unity ──────────────────────────────────────────────────────

        static Vector3 FromRos(Vector3 v) => new Vector3(-v.y, v.z, v.x);
        static Quaternion FromRos(Quaternion r) => new Quaternion(r.y, -r.z, -r.x, r.w);

        static JObject RosPose(Vector3 localPosition, Quaternion localRotation)
        {
            var p = ArticulationDescriber.ToRos(localPosition);
            var q = ArticulationDescriber.ToRos(localRotation);
            return new JObject
            {
                ["position"] = new JArray(Math.Round(p.x, 6), Math.Round(p.y, 6), Math.Round(p.z, 6)),
                ["orientation"] = new JArray(Math.Round(q.x, 6), Math.Round(q.y, 6), Math.Round(q.z, 6), Math.Round(q.w, 6)),
            };
        }

        static string ObjectName(JObject it, string fallbackKind, string id)
        {
            var name = it["name"]?.Value<string>();
            return !string.IsNullOrWhiteSpace(name) ? name : $"{fallbackKind}_{id}";
        }

        static Vector3 Vec(JToken t, Vector3 fallback)
        {
            var a = t as JArray;
            return a != null && a.Count >= 3 ? new Vector3(a[0].Value<float>(), a[1].Value<float>(), a[2].Value<float>()) : fallback;
        }

        static Quaternion Quat(JToken t)
        {
            var a = t as JArray;
            return a != null && a.Count >= 4
                ? new Quaternion(a[0].Value<float>(), a[1].Value<float>(), a[2].Value<float>(), a[3].Value<float>())
                : Quaternion.identity;
        }

        Transform Link(string name)
        {
            if (string.IsNullOrEmpty(name) || name == "@scene") return null;
            if (_links.TryGetValue(name, out var t) && t != null) return t;
            t = _robotRoot.GetComponentsInChildren<Transform>(true).FirstOrDefault(x => x.name == name);
            if (t == null) throw new ArgumentException($"no link named '{name}' on {_robotRoot.name}");
            _links[name] = t;
            return t;
        }

        Material Mat(Color c)
        {
            if (_materials.TryGetValue(c, out var m)) return m;
            // Drawn on top of the robot (markers sit inside links / tools): Unity's internal unlit line shader with
            // depth testing off. It has no LightMode tag, so URP and HDRP render it too.
            var overlay = Shader.Find("Hidden/Internal-Colored");
            var shader = overlay ?? Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Unlit/Color");
            m = new Material(shader) { color = c, hideFlags = HideFlags.DontSave };
            if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", c);
            if (overlay != null)
            {
                m.SetInt("_ZTest", (int)UnityEngine.Rendering.CompareFunction.Always);
                m.SetInt("_ZWrite", 0);
                m.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
                m.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Overlay;
            }
            _materials[c] = m;
            return m;
        }

        LineRenderer NewLine(Transform parent, string name, Color c, float width, bool world)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            var lr = go.AddComponent<LineRenderer>();
            lr.useWorldSpace = world;
            lr.sharedMaterial = Mat(c);
            lr.widthMultiplier = width;
            lr.numCapVertices = 2;
            lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            lr.receiveShadows = false;
            return lr;
        }

        GameObject Container()
        {
            if (_container == null)
            {
                _container = new GameObject("Markers");          // under the robot instance
                _container.hideFlags = HideFlags.DontSave;
                _container.transform.SetParent(_robotRoot, false);
            }
            return _container;
        }

        // ── visualize ────────────────────────────────────────────────────────

        /// <summary>Apply a `visualize` payload; returns null or an error message (unknown links are reported).</summary>
        public string Apply(JObject payload)
        {
            bool replace = payload["replace"]?.Value<bool>() ?? true;
            var keep = new HashSet<string>();
            var errors = new List<string>();
            foreach (var token in payload["items"] as JArray ?? new JArray())
            {
                if (!(token is JObject it)) continue;
                var id = it["id"]?.Value<string>();
                if (string.IsNullOrEmpty(id)) { errors.Add("item without id"); continue; }
                keep.Add(id);
                try
                {
                    if (it["remove"]?.Value<bool>() == true) { Remove(id); continue; }
                    var kind = it["kind"]?.Value<string>() ?? "frame";
                    if (kind == "frame") ApplyFrame(id, it);
                    else if (kind == "chain") ApplyChain(id, it);
                    else errors.Add($"{id}: unknown kind '{kind}'");
                }
                catch (Exception e) { errors.Add($"{id}: {e.Message}"); }
            }
            if (replace)   // update in place, drop what is no longer listed (re-creating would lose the selection)
                foreach (var id in _items.Keys.Where(k => !keep.Contains(k)).ToList()) Remove(id);
            return errors.Count == 0 ? null : string.Join("; ", errors);
        }

        void ApplyFrame(string id, JObject it)
        {
            // resolve the parent link before creating anything: a frame on a link this robot lacks is reported in the
            // ack and leaves nothing behind (it used to leave an unstyled 1 m sphere handle in the scene)
            var parentName = it["parent"]?.Value<string>() ?? "";
            var parent = Link(parentName);
            if (!_items.TryGetValue(id, out var item) || item.Kind != "frame")
            {
                Remove(id);
                item = new Item { Id = id, Kind = "frame" };
                item.Go = new GameObject(id);
                item.Go.hideFlags = HideFlags.DontSave;
                var marker = item.Go.AddComponent<RemoteControlMarker>();
                marker.owner = _owner;
                item.Axes = new[]
                {
                    NewLine(item.Go.transform, "x", AxisX, 0.004f, false),
                    NewLine(item.Go.transform, "y", AxisY, 0.004f, false),
                    NewLine(item.Go.transform, "z", AxisZ, 0.004f, false),
                };
                var handle = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                handle.name = "handle";
                UnityEngine.Object.Destroy(handle.GetComponent<Collider>());   // never collides with the robot
                handle.transform.SetParent(item.Go.transform, false);
                handle.transform.localScale = Vector3.one * 0.02f;        // Style() sets the real size
                var mr = handle.GetComponent<MeshRenderer>();
                mr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                mr.receiveShadows = false;
                item.Handle = handle.transform;
                _items[id] = item;
            }
            item.Label = it["label"]?.Value<string>() ?? id;
            item.Style = it["style"]?.Value<string>() ?? "frame";
            item.Selectable = it["selectable"]?.Value<bool>() ?? false;
            item.Editable = it["editable"]?.Value<bool>() ?? false;
            item.Size = it["size"]?.Value<float>() ?? (item.Style == "tcp" ? 0.08f : 0.1f);
            item.Go.name = ObjectName(it, item.Style == "tcp" ? "TCP" : "Frame", id);
            item.Parent = parentName;
            item.Go.transform.SetParent(parent != null ? parent : Container().transform, false);
            if (item.ChangedAt < 0)   // don't yank a frame the user is dragging right now
            {
                var pose = it["pose"] as JObject;
                item.Go.transform.localPosition = item.SetPosition = FromRos(Vec(pose?["position"], Vector3.zero));
                item.Go.transform.localRotation = item.SetRotation = FromRos(Quat(pose?["orientation"]));
            }
            if (it["selected"]?.Value<bool>() == true) SelectedId = id;
            if (it["focus"]?.Value<bool>() == true) _focusRequest = id;
            var m = item.Go.GetComponent<RemoteControlMarker>();
            m.itemId = id;
            m.label = item.Label;
            m.selectable = item.Selectable;
            m.editable = item.Editable;
            Style(item);
        }

        void ApplyChain(string id, JObject it)
        {
            // validate before creating anything: a chain naming links this robot lacks (e.g. one meant for another
            // robot) is reported in the ack and leaves no marker behind
            var links = (it["links"] as JArray ?? new JArray()).Select(t => t.Value<string>()).ToArray();
            foreach (var l in links) Link(l);
            if (!_items.TryGetValue(id, out var item) || item.Kind != "chain")
            {
                Remove(id);
                item = new Item { Id = id, Kind = "chain" };
                item.Go = new GameObject(id);
                item.Go.hideFlags = HideFlags.DontSave;
                item.Go.transform.SetParent(Container().transform, false);
                item.Line = NewLine(item.Go.transform, "line", ChainColor, 0.006f, true);
                _items[id] = item;
            }
            item.Go.name = ObjectName(it, "Chain", id);
            item.Links = links;
            item.End = it["end"]?.Value<string>();
            item.Label = it["label"]?.Value<string>() ?? id;
            item.Style = it["style"]?.Value<string>() ?? "chain";
            UpdateChain(item);
        }

        void Style(Item item)
        {
            bool selected = item.Id == SelectedId;
            if (item.Handle != null)
            {
                item.Handle.localScale = Vector3.one * item.Size * (selected ? 0.3f : 0.2f);
                item.Handle.GetComponent<MeshRenderer>().sharedMaterial =
                    Mat(selected ? SelectedColor : item.Style == "tcp" ? ChainColor : Color.white);
            }
            float width = selected ? 0.008f : item.Style == "tcp" ? 0.005f : 0.004f;
            var dirs = new[] { Vector3.forward, Vector3.left, Vector3.up };   // ROS x, y, z in Unity
            for (int i = 0; i < 3; i++)
            {
                var lr = item.Axes[i];
                lr.positionCount = 2;
                lr.SetPosition(0, Vector3.zero);
                lr.SetPosition(1, dirs[i] * item.Size);
                lr.widthMultiplier = width;
            }
        }

        void UpdateChain(Item item)
        {
            var points = new List<Vector3>();
            foreach (var l in item.Links)
            {
                Transform t;
                try { t = Link(l); }
                catch (ArgumentException) { continue; }      // never throw every frame (the link may be gone)
                if (t != null) points.Add(t.position);
            }
            if (item.End != null && _items.TryGetValue(item.End, out var end) && end.Go != null)
                points.Add(end.Go.transform.position);
            item.Line.positionCount = points.Count;
            item.Line.SetPositions(points.ToArray());
        }

        public void Remove(string id)
        {
            if (!_items.TryGetValue(id, out var item)) return;
            if (item.Go != null) UnityEngine.Object.Destroy(item.Go);
            _items.Remove(id);
        }

        public void Clear()
        {
            foreach (var item in _items.Values)
                if (item.Go != null) UnityEngine.Object.Destroy(item.Go);
            _items.Clear();
        }

        public void Dispose()
        {
            Clear();
            if (_container != null) UnityEngine.Object.Destroy(_container);
            foreach (var m in _materials.Values) UnityEngine.Object.Destroy(m);
            _materials.Clear();
        }

        /// <summary>Call from LateUpdate: chains follow the links as the robot moves; user edits of editable
        /// frames are reported once they stop changing.</summary>
        public void LateUpdate()
        {
            double now = Time.realtimeSinceStartupAsDouble;
            foreach (var item in _items.Values.ToList())
            {
                if (item.Kind == "chain") { UpdateChain(item); continue; }
                if (!item.Editable || item.Go == null) continue;
                var t = item.Go.transform;
                bool moved = Vector3.Distance(t.localPosition, item.SetPosition) > 1e-5f
                             || Quaternion.Angle(t.localRotation, item.SetRotation) > 0.01f;
                if (moved)
                {
                    if (item.ChangedAt < 0 || t.hasChanged) item.ChangedAt = now;
                    t.hasChanged = false;
                    if (now - item.ChangedAt >= EditSettleTime)
                    {
                        item.SetPosition = t.localPosition;
                        item.SetRotation = t.localRotation;
                        item.ChangedAt = -1;
                        Edited?.Invoke(item.Id, item.Parent, RosPose(t.localPosition, t.localRotation));
                    }
                }
                else
                {
                    item.ChangedAt = -1;
                }
            }
        }

        // ── selection ────────────────────────────────────────────────────────

        /// <summary>Highlight an item (also used when the controller or the Editor selects it).</summary>
        public void Highlight(string id)
        {
            var previous = SelectedId;
            SelectedId = id;
            foreach (var key in new[] { previous, id })
                if (key != null && _items.TryGetValue(key, out var item) && item.Kind == "frame") Style(item);
        }

        public bool IsSelectable(string id) => id != null && _items.TryGetValue(id, out var it) && it.Selectable;

        public bool IsEditable(string id) => id != null && _items.TryGetValue(id, out var it) && it.Editable;

        /// <summary>World rotation of the TCP frame shown (style "tcp"), or null.</summary>
        public Quaternion? TcpRotation()
        {
            var tcp = _items.Values.FirstOrDefault(i => i.Kind == "frame" && i.Style == "tcp" && i.Go != null);
            return tcp?.Go.transform.rotation;
        }

        public GameObject GameObjectOf(string id) => id != null && _items.TryGetValue(id, out var it) ? it.Go : null;

        /// <summary>An item the controller asked to focus (Toolbox "Edit in Unity"); null if none. Clears it.</summary>
        public string TakeFocusRequest()
        {
            var id = _focusRequest;
            _focusRequest = null;
            return id;
        }

        /// <summary>Call from OnGUI: draws labels; a click near a selectable frame's origin returns its id.</summary>
        public string OnGUI(Camera cam)
        {
            if (cam == null || _items.Count == 0) return null;
            if (_labelStyle == null)
                _labelStyle = new GUIStyle(GUI.skin.label) { fontSize = 12, normal = { textColor = Color.white } };
            string clicked = null;
            var e = Event.current;
            float best = 16f;     // pixels
            foreach (var item in _items.Values)
            {
                if (item.Kind != "frame" || item.Go == null) continue;
                var sp = cam.WorldToScreenPoint(item.Go.transform.position);
                if (sp.z <= 0) continue;
                var gui = new Vector2(sp.x, Screen.height - sp.y);
                if (e.type == EventType.Repaint)
                {
                    var old = GUI.color;
                    GUI.color = item.Id == SelectedId ? SelectedColor : Color.white;
                    GUI.Label(new Rect(gui.x + 6, gui.y - 18, 220, 20), item.Label, _labelStyle);
                    GUI.color = old;
                }
                if (e.type == EventType.MouseDown && e.button == 0 && (item.Selectable || item.Editable))
                {
                    float d = Vector2.Distance(gui, e.mousePosition);
                    if (d < best) { best = d; clicked = item.Id; }
                }
            }
            if (clicked != null) e.Use();
            return clicked;
        }
    }
}
