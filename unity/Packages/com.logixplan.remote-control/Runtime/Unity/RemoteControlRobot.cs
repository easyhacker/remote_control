// Put this on the root ArticulationBody of a robot. It connects to a Remote Control controller,
// announces the robot's joints, and executes motion goals (pause / resume / cancel / stop included).

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace RobotMarket.RemoteControl.Unity
{
    [DisallowMultipleComponent]
    [AddComponentMenu("IVI Dynamic/Remote Control Robot")]
    public sealed class RemoteControlRobot : MonoBehaviour
    {
        public enum ConnectionSource
        {
            [Tooltip("%RC_CONFIG_DIR%\\remote_control.json - the one config file shared with the controller")]
            ConfigFile,
            [Tooltip("The Controller Url field below (quick tests)")]
            ControllerUrl,
        }

        [Header("Connection")]
        [Tooltip("Where the communication type and parameters come from. Config File reads " +
                 "%RC_CONFIG_DIR%\\remote_control.json, the same file the controller uses.")]
        public ConnectionSource connectionSource = ConnectionSource.ConfigFile;
        [Tooltip("Used when Connection Source = Controller Url: ws://host:8765/motion, or mqtt://[user:pass@]broker:1883/<prefix>")]
        public string controllerUrl = "ws://localhost:8765/motion";
        [Tooltip("Identifies this robot to the controller (and names its data folder). Empty = generated from the " +
                 "GameObject name, unique in the scene; the controller also renames robots whose id is already online")]
        public string robotId = "";
        [Tooltip("Human-readable name shown by the controller. Empty = the robot id")]
        public string displayName = "";

        /// <summary>Robot id in use: the session's (the controller may have renamed it), else the Robot Id field,
        /// else one generated from the GameObject name (lowercase, '-' for unusual characters, '-2', '-3' … for
        /// robots in the loaded scenes that would get the same id).</summary>
        public string RobotId => Session != null ? Session.RobotId
            : !string.IsNullOrWhiteSpace(robotId) ? robotId.Trim() : AutoRobotId();

        static string Slug(string name)
        {
            var sb = new System.Text.StringBuilder();
            foreach (var c in (name ?? "").Trim().ToLowerInvariant())
                sb.Append(char.IsLetterOrDigit(c) && c < 128 || c == '_' || c == '-' ? c : '-');
            var slug = System.Text.RegularExpressions.Regex.Replace(sb.ToString(), "-{2,}", "-").Trim('-');
            return slug.Length > 0 ? slug : "robot";
        }

        static string HierarchyPath(Transform t)
        {
            var path = t.GetSiblingIndex().ToString("D4");
            for (var p = t.parent; p != null; p = p.parent) path = p.GetSiblingIndex().ToString("D4") + "/" + path;
            return t.gameObject.scene.buildIndex.ToString("D4") + ":" + t.gameObject.scene.name + "/" + path;
        }

        string AutoRobotId()
        {
            var baseId = Slug(gameObject.name);
            var taken = new HashSet<string>();
            foreach (var r in FindObjectsByType<RemoteControlRobot>(FindObjectsInactive.Exclude, FindObjectsSortMode.None))
                if (r != this && !string.IsNullOrWhiteSpace(r.robotId)) taken.Add(r.robotId.Trim());
            var same = FindObjectsByType<RemoteControlRobot>(FindObjectsInactive.Exclude, FindObjectsSortMode.None)
                .Where(r => string.IsNullOrWhiteSpace(r.robotId) && Slug(r.gameObject.name) == baseId)
                .OrderBy(r => HierarchyPath(r.transform), StringComparer.Ordinal).ToList();
            int n = 1;
            foreach (var r in same)
            {
                var candidate = n == 1 ? baseId : $"{baseId}-{n}";
                while (taken.Contains(candidate)) candidate = $"{baseId}-{++n}";
                taken.Add(candidate);
                n++;
                if (r == this) return candidate;
            }
            return baseId;
        }

        /// <summary>Display name actually used: the Display Name field, else the robot id.</summary>
        public string DisplayName => !string.IsNullOrWhiteSpace(displayName) ? displayName.Trim() : RobotId;

        [Header("Data names")]
        [Tooltip("Project name reported to the controller, which files this robot's data (description, saved poses) " +
                 "under <data_dir>/<project>/<stage>/<robot id>. Empty = the Unity project's folder name")]
        public string projectName = "";
        [Tooltip("Stage name reported to the controller. Empty = the name of the scene this robot is in")]
        public string stageName = "";

        [Header("Joints")]
        [Tooltip("Root of the articulation; defaults to this GameObject's ArticulationBody")]
        public ArticulationBody articulationRoot;
        [Tooltip("Used for joints without an override (rad/s or m/s). 0 = unlimited")]
        public float defaultMaxVelocity = 2.0f;
        public List<JointOverride> jointOverrides = new List<JointOverride>();

        [Tooltip("Keep Unity running when its window loses focus (e.g. while you type in the controller's " +
                 "terminal). Without it Unity pauses, heartbeats stop, and the controller drops the robot.")]
        public bool runInBackground = true;

        [Tooltip("Turn off gravity on the robot's own links while this component runs (like Isaac Sim's position-" +
                 "controlled robots). Light links on Acceleration drives otherwise sag under gravity, giving pose-dependent " +
                 "tracking errors. Other objects keep their gravity; the scene file is not changed.")]
        public bool disableRobotGravity = true;

        [Tooltip("Ignore collisions between this robot's own links (like Isaac Sim). Imported convex collision " +
                 "shapes often overlap neighbouring links and would jam the joints. Contact with other objects is kept.")]
        public bool ignoreSelfCollisions = true;

        [Header("Motion")]
        [Tooltip("Also drive each joint with the target's speed, not just its position. Removes most of the lag " +
                 "of damped drives behind moving targets.")]
        public bool velocityFeedForward = true;

        [Tooltip("Seconds to slow to a halt on pause / cancel / stop")]
        [Min(0.01f)] public float decelTime = 0.4f;

        [Header("Debug")]
        [Tooltip("Write commanded vs measured joint motion for every physics step to Logs/rc_tracking.csv")]
        public bool recordTracking = false;
        public bool showOverlay = true;
        [Tooltip("Overlay starts expanded (joint list, recent messages) or as one summary line; click it or press the toggle key to switch")]
        public bool overlayExpanded = true;
        [Tooltip("Key that opens / closes the overlay while the Game view has focus (None = click only)")]
        public KeyCode overlayToggleKey = KeyCode.F1;
        [Tooltip("Overlay text size in pixels at 1080p (scaled up for larger Game view resolutions)")]
        [Range(8, 40)] public int overlayFontSize = 14;
        public bool logMessages = false;

        [Header("Markers")]
        [Tooltip("Draw the frames and kinematic chains the controller sends (TCP, targets, user frames). Click a " +
                 "frame in the Game view, or select it in the Hierarchy, to pick it as the target in the controller")]
        public bool showMarkers = true;

        [Header("Targets")]
        [Tooltip("Ctrl+click on an object creates a target there. On: the target is attached to the clicked object " +
                 "(moves with it). Off: it goes under the scene's \"Targets\" object")]
        public bool attachNewTargets = false;
        [Tooltip("Orientation of targets made by Ctrl+click. Approach: z into the surface, x towards the robot. " +
                 "Surface: z out of the surface. Keep TCP: the TCP's current orientation")]
        public ClickOrientation clickTargetOrientation = ClickOrientation.Approach;

        /// <summary>World rotation of the TCP marker currently shown (null if none).</summary>
        public Quaternion? TcpRotation => _markers?.TcpRotation();

        MarkerLayer _markers;

        public RobotSession Session { get; private set; }

        /// <summary>Project name actually reported (Inspector value, else the project folder / product name).</summary>
        public string ProjectName => !string.IsNullOrWhiteSpace(projectName) ? projectName.Trim()
            : Application.isEditor ? System.IO.Path.GetFileName(System.IO.Path.GetDirectoryName(Application.dataPath))
            : Application.productName;

        /// <summary>Stage name actually reported (Inspector value, else this robot's scene).</summary>
        public string StageName => !string.IsNullOrWhiteSpace(stageName) ? stageName.Trim()
            : !string.IsNullOrEmpty(gameObject.scene.name) ? gameObject.scene.name : RobotSession.DefaultName;

        readonly Queue<string> _recent = new Queue<string>();
        string _endpoint = "";
        ArticulationJointDriver _driver;
        GUIStyle _style;

        void OnEnable()
        {
            ApplyCommandLine();
            if (articulationRoot == null) articulationRoot = FindArticulationRoot();
            if (articulationRoot == null)
            {
                Debug.LogError("[RemoteControl] no ArticulationBody on this object or its children — " +
                               "put this component on (or above) a robot built from ArticulationBody joints", this);
                enabled = false;
                return;
            }
            IRobotTransport transport;
            try
            {
                if (connectionSource == ConnectionSource.ConfigFile)
                {
                    string path = RemoteControlConfig.SystemConfigPath();
                    var config = RemoteControlConfig.Load(path);
                    transport = RemoteControlConfig.RobotTransportFromConfig(config);
                    _endpoint = RemoteControlConfig.Describe(config);
                    Debug.Log($"[RemoteControl] config: {path} → {_endpoint}", this);
                }
                else
                {
                    transport = TransportFactory.FromUrl(controllerUrl);
                    _endpoint = controllerUrl;
                }
            }
            catch (Exception e)
            {
                Debug.LogError($"[RemoteControl] {e.Message}", this);
                enabled = false;
                return;
            }
            if (runInBackground) Application.runInBackground = true;
            DisableConflictingControllers();
            if (ignoreSelfCollisions) IgnoreSelfCollisions();
            if (disableRobotGravity) DisableRobotGravity();
            var driver = new ArticulationJointDriver(articulationRoot, jointOverrides, defaultMaxVelocity)
            {
                VelocityFeedForward = velocityFeedForward,
            };
            _driver = driver;
            if (recordTracking)
            {
                System.IO.Directory.CreateDirectory("Logs");
                driver.Trace = new System.IO.StreamWriter("Logs/rc_tracking.csv", false) { AutoFlush = false };
                Debug.Log("[RemoteControl] recording joint tracking to Logs/rc_tracking.csv", this);
            }
            Session = new RobotSession(transport, driver, RobotId, DisplayName, decelTime)
            {
                Project = ProjectName,
                Stage = StageName,
            };
            var root = articulationRoot;
            Session.Describer = tree =>
            {
                var d = ArticulationDescriber.Describe(root, driver, tree, gameObject);
                d["robot_pose"] = ArticulationDescriber.Pose(transform.position, transform.rotation);
                d["targets"] = ListTargets();
                return d;
            };
            Session.TargetHandler = HandleTarget;
            RemoteControlTarget.AttachNewTargets = attachNewTargets;
            RemoteControlTarget.NewTargetOrientation = clickTargetOrientation;
            RemoteControlTarget.AdoptChildrenOfRoot(gameObject.scene);
            Session.MessageTraced += Trace;
            Session.ConnectionChanged += OnConnectionChanged;
            Session.Renamed += (oldId, newId) =>
                Debug.LogWarning($"[RemoteControl] the controller already has a robot '{oldId}' - this robot is now '{newId}'", this);
            if (showMarkers)
            {
                _markers = new MarkerLayer(this);
                Session.Visualizer = payload => _markers.Apply(payload);
                _markers.Edited += (itemId, parent, pose) =>
                {
                    Session?.Edited(itemId, parent, pose, "editor");
                    if (logMessages) Debug.Log($"[RemoteControl] edited {itemId} in {parent}: {pose.ToString(Newtonsoft.Json.Formatting.None)}", this);
                };
            }
            Session.Start();
            Debug.Log($"[RemoteControl] '{RobotId}' in {Session.Project} / {Session.Stage} with {driver.Joints.Count} joints " +
                      $"({string.Join(", ", driver.Joints.Select(j => j.Name))}) → {_endpoint}", this);
        }

        /// <summary>
        /// This object's ArticulationBody, else the first chain root among its children
        /// (URDF Importer robots: the top object has none; the chain starts at e.g. base_link).
        /// </summary>
        ArticulationBody FindArticulationRoot()
        {
            var own = GetComponent<ArticulationBody>();
            if (own != null) return own;
            foreach (var body in GetComponentsInChildren<ArticulationBody>(true))
                if (body.isRoot) return body;
            return null;
        }

        // Other components that write ArticulationBody drive targets every frame would fight the remote control.
        // Matched by type name so this package doesn't depend on theirs.
        static readonly string[] ConflictingControllers =
        {
            "Unity.Robotics.UrdfImporter.Control.Controller",   // URDF Importer keyboard jogging
            "RoboSynth.UnityTools.JointTargetController",       // RoboSynth inspector sliders
            "RobotJogController",                               // RoboSynth jog window helper (global namespace)
        };

        void DisableConflictingControllers()
        {
            foreach (var mb in GetComponentsInChildren<MonoBehaviour>(true))
            {
                if (mb == null || mb == this || !mb.enabled) continue;
                if (Array.IndexOf(ConflictingControllers, mb.GetType().FullName) < 0) continue;
                mb.enabled = false;
                Debug.Log($"[RemoteControl] disabled {mb.GetType().Name} on '{mb.name}' — it would fight the remote control " +
                          "over the joint targets (re-enable it when you remove Remote Control Robot)", mb);
            }
        }

        void DisableRobotGravity()
        {
            int n = 0;
            foreach (var body in articulationRoot.GetComponentsInChildren<ArticulationBody>(true))
                if (body.useGravity) { body.useGravity = false; n++; }
            if (n > 0)
                Debug.Log($"[RemoteControl] gravity off on {n} robot link(s) — drives hold the pose like real servos", this);
        }

        void IgnoreSelfCollisions()
        {
            var colliders = articulationRoot.GetComponentsInChildren<Collider>(true);
            for (int i = 0; i < colliders.Length; i++)
                for (int j = i + 1; j < colliders.Length; j++)
                    Physics.IgnoreCollision(colliders[i], colliders[j], true);
            if (colliders.Length > 1)
                Debug.Log($"[RemoteControl] ignoring collisions between the robot's own {colliders.Length} colliders", this);
        }

        void Reset()   // when the component is added in the editor: pre-fill the root
        {
            articulationRoot = FindArticulationRoot();
        }

        /// <summary>Player builds: -controllerUrl ws://host:8765/motion  -robotId my-arm</summary>
        void ApplyCommandLine()
        {
            var args = Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length - 1; i++)
            {
                if (args[i] == "-controllerUrl") { controllerUrl = args[i + 1]; connectionSource = ConnectionSource.ControllerUrl; }
                else if (args[i] == "-robotId") robotId = args[i + 1];
            }
        }

        void FixedUpdate()
        {
            if (Session == null) return;
            double now = Time.realtimeSinceStartupAsDouble;
            if (_lastStep > 0 && now - _lastStep > 1.0) ReportHitch(now - _lastStep, now);
            _lastStep = now;
            Session.Update(Time.fixedDeltaTime, now);
            _driver?.EndStep();
        }

        double _lastStep, _lastHitchLog = double.NegativeInfinity, _worstHitch;
        int _hitches;

        /// <summary>Unity went a while without a physics step (slow render / Editor frame). The session keeps the
        /// link alive from a background thread; motion just takes longer. Logged at most every 30 s.</summary>
        void ReportHitch(double seconds, double now)
        {
            _hitches++;
            _worstHitch = Math.Max(_worstHitch, seconds);
            if (now - _lastHitchLog < 30) return;
            Debug.LogWarning($"[RemoteControl] Unity stalled {_hitches}x, up to {_worstHitch:0.0} s without a physics step " +
                             "(slow rendering or Editor work - see Window > Analysis > Profiler). Motion slows down during " +
                             "stalls; the connection is kept alive.", this);
            _lastHitchLog = now;
            _hitches = 0;
            _worstHitch = 0;
        }

        void OnConnectionChanged(bool connected, string reason)
        {
            if (connected) Debug.Log($"[RemoteControl] connected → {_endpoint}", this);
            else Debug.LogWarning($"[RemoteControl] disconnected: {reason ?? "unknown reason"}", this);
        }

        void LateUpdate()
        {
            if (_markers == null) return;
            _markers.LateUpdate();
            var focus = _markers.TakeFocusRequest();
            if (focus != null) FocusInEditor(focus);
        }

        /// <summary>Editor: select a marker (TCP, frame) and show it in the Scene view, ready for the Move / Rotate
        /// tools. Clicking it in the Scene view often hits the robot's mesh instead (the TCP sits inside the tool).</summary>
        public void FocusInEditor(string itemId) => FocusInEditor(_markers?.GameObjectOf(itemId));

        public void FocusInEditor(GameObject go)
        {
#if UNITY_EDITOR
            if (go == null) return;
            UnityEditor.Selection.activeGameObject = go;
            UnityEditor.EditorGUIUtility.PingObject(go);
            var view = UnityEditor.SceneView.lastActiveSceneView;
            if (view != null)
            {
                view.Focus();
                view.Frame(new Bounds(go.transform.position, Vector3.one * 0.5f), false);
            }
            if (UnityEditor.Tools.current != UnityEditor.Tool.Rotate) UnityEditor.Tools.current = UnityEditor.Tool.Move;
#endif
        }

        /// <summary>Pick a marker as the target: highlights it and reports `selected` to the controller.</summary>
        public void SelectMarker(string itemId, string source)
        {
            if (_markers == null || !_markers.IsSelectable(itemId)) return;
            _markers.Highlight(itemId);
            Session?.Select(itemId, source);
            if (logMessages) Debug.Log($"[RemoteControl] selected {itemId} ({source})", this);
        }

        void OnValidate()
        {
            RemoteControlTarget.AttachNewTargets = attachNewTargets;
            RemoteControlTarget.NewTargetOrientation = clickTargetOrientation;
        }

        /// <summary>Rotation for a target clicked at `point` (uses this robot's settings, TCP and position).</summary>
        public Quaternion ClickTargetRotation(Vector3 point, Vector3 normal, Vector3 viewForward) =>
            RemoteControlTarget.Orientation(clickTargetOrientation, point, normal, viewForward, TcpRotation,
                                            articulationRoot != null ? articulationRoot.transform.position : transform.position);

        // ── targets (owned by the scene) ─────────────────────────────────────

        static JObject LocalPose(Transform frame, Vector3 position, Quaternion rotation) =>
            ArticulationDescriber.Pose(frame.InverseTransformPoint(position), Quaternion.Inverse(frame.rotation) * rotation);

        /// <summary>The scene's targets with their pose in the scene, in this robot and in its root link
        /// (ROS convention), for `describe` replies.</summary>
        JArray ListTargets()
        {
            var list = new JArray();
            foreach (var t in RemoteControlTarget.All)
            {
                if (t == null || !t.isActiveAndEnabled) continue;
                var tr = t.transform;
                list.Add(new JObject
                {
                    ["id"] = t.Id,
                    ["name"] = t.name,
                    ["path"] = RemoteControlTarget.PathOf(tr),
                    ["parent"] = tr.parent != null ? RemoteControlTarget.PathOf(tr.parent) : "",
                    ["pose_in_scene"] = ArticulationDescriber.Pose(tr.position, tr.rotation),
                    ["pose_in_robot"] = LocalPose(transform, tr.position, tr.rotation),
                    ["pose_in_root"] = LocalPose(articulationRoot.transform, tr.position, tr.rotation),
                });
            }
            return list;
        }

        static Vector3 FromRos(Vector3 v) => new Vector3(-v.y, v.z, v.x);
        static Quaternion FromRos(Quaternion r) => new Quaternion(r.y, -r.z, -r.x, r.w);

        /// <summary>`target` requests from the controller. op: create (name, reference, pose, parent?),
        /// update (id, reference, pose), delete (id), select (id), settings (attach).
        /// reference: "scene", "robot" or "link:&lt;name&gt;"; pose in ROS convention.</summary>
        string HandleTarget(JObject p)
        {
            var op = p["op"]?.Value<string>() ?? "";
            if (op == "settings")
            {
                if (p["attach"] != null) attachNewTargets = RemoteControlTarget.AttachNewTargets = p["attach"].Value<bool>();
                var orientation = p["orientation"]?.Value<string>();
                if (orientation != null)
                {
                    if (orientation == "approach") clickTargetOrientation = ClickOrientation.Approach;
                    else if (orientation == "surface") clickTargetOrientation = ClickOrientation.Surface;
                    else if (orientation == "tcp") clickTargetOrientation = ClickOrientation.KeepTcp;
                    else return $"unknown orientation '{orientation}' (approach, surface, tcp)";
                    RemoteControlTarget.NewTargetOrientation = clickTargetOrientation;
                }
                return null;
            }
            RemoteControlTarget target = null;
            if (op != "create")
            {
                var id = p["id"]?.Value<string>();
                target = RemoteControlTarget.Find(id);
                if (target == null) return $"no target '{id}'";
            }
            if (op == "delete")
            {
                Destroy(target.gameObject);
                return null;
            }
            if (op == "select")
            {
                foreach (var t in RemoteControlTarget.All) if (t != null) t.SetHighlighted(t == target);
                FocusInEditor(target.gameObject);
                return null;
            }
            if (op != "create" && op != "update") return $"unknown target op '{op}'";

            // world pose from the reference frame
            Transform reference = null;
            var refName = p["reference"]?.Value<string>() ?? "scene";
            if (refName == "robot") reference = transform;
            else if (refName.StartsWith("link:"))
            {
                var link = refName.Substring(5);
                reference = GetComponentsInChildren<Transform>(true).FirstOrDefault(x => x.name == link);
                if (reference == null) return $"no link '{link}'";
            }
            else if (refName != "scene") return $"unknown reference '{refName}'";
            var pose = p["pose"] as JObject;
            var pa = pose?["position"] as JArray;
            var qa = pose?["orientation"] as JArray;
            var pos = FromRos(pa != null ? new Vector3(pa[0].Value<float>(), pa[1].Value<float>(), pa[2].Value<float>()) : Vector3.zero);
            var rot = FromRos(qa != null ? new Quaternion(qa[0].Value<float>(), qa[1].Value<float>(), qa[2].Value<float>(), qa[3].Value<float>())
                                         : Quaternion.identity);
            if (reference != null)
            {
                pos = reference.TransformPoint(pos);
                rot = reference.rotation * rot;
            }

            if (op == "create")
            {
                var name = p["name"]?.Value<string>();
                if (string.IsNullOrWhiteSpace(name)) return "target needs a name";
                Transform parent = null;
                var parentPath = p["parent"]?.Value<string>();
                if (!string.IsNullOrEmpty(parentPath))
                {
                    parent = FindByPath(parentPath);
                    if (parent == null) return $"no scene object '{parentPath}'";
                }
                parent = parent != null ? parent : RemoteControlTarget.Root(gameObject.scene, true);
                var existing = parent.Find(name.Trim());
                var go = existing != null ? existing.gameObject : new GameObject(name.Trim());
                go.transform.SetParent(parent, true);
                target = go.GetComponent<RemoteControlTarget>() ?? go.AddComponent<RemoteControlTarget>();
            }
            target.transform.SetPositionAndRotation(pos, rot);
            return null;
        }

        static Transform FindByPath(string path)
        {
            foreach (var go in FindObjectsByType<Transform>(FindObjectsInactive.Include, FindObjectsSortMode.None))
                if (RemoteControlTarget.PathOf(go) == path) return go;
            return null;
        }

        /// <summary>Ctrl+click: make a target on the clicked object (Game view).</summary>
        void CreateTargetAt(Vector2 guiPosition, Camera cam)
        {
            var ray = cam.ScreenPointToRay(new Vector3(guiPosition.x, Screen.height - guiPosition.y, 0));
            if (!RemoteControlTarget.Pick(ray, out var point, out var normal, out var hitObject))
            {
                Debug.Log("[RemoteControl] Ctrl+click: nothing there to put a target on", this);
                return;
            }
            var t = RemoteControlTarget.CreateAt(point, ClickTargetRotation(point, normal, cam.transform.forward),
                                                 hitObject, attachNewTargets);
            Debug.Log($"[RemoteControl] new target '{RemoteControlTarget.PathOf(t.transform)}' on {hitObject.name}", this);
            SelectTarget(t);
        }

        void SelectTarget(RemoteControlTarget target)
        {
            foreach (var t in RemoteControlTarget.All) if (t != null) t.SetHighlighted(t == target);
            Session?.Select(target.Id, "click");
            FocusInEditor(target.gameObject);
        }

        /// <summary>Game view: the target whose origin is nearest a click (within 16 px).</summary>
        static RemoteControlTarget TargetAt(Vector2 guiPosition, Camera cam)
        {
            RemoteControlTarget best = null;
            float bestDistance = 16f;
            foreach (var t in RemoteControlTarget.All)
            {
                if (t == null || !t.isActiveAndEnabled) continue;
                var sp = cam.WorldToScreenPoint(t.transform.position);
                if (sp.z <= 0) continue;
                float d = Vector2.Distance(new Vector2(sp.x, Screen.height - sp.y), guiPosition);
                if (d < bestDistance) { bestDistance = d; best = t; }
            }
            return best;
        }

        void OnDisable()
        {
            _markers?.Dispose();
            _markers = null;
            if (Session == null) return;
            if (_driver?.Trace != null) { _driver.Trace.Flush(); _driver.Trace.Dispose(); _driver.Trace = null; }
            Session.MessageTraced -= Trace;
            Session.ConnectionChanged -= OnConnectionChanged;
            Session.Shutdown();
            Session = null;
        }

        void Trace(Envelope env, bool outgoing)
        {
            // Routine traffic is never logged: feedback streams during goals, and controllers poll positions with
            // describe several times a second. Logged with stack traces, they flood the Console and slow the Editor.
            if (env.Type == MsgType.Feedback || env.Type == MsgType.Describe || env.Type == MsgType.Description) return;
            string line = (outgoing ? "→ " : "← ") + env.Type + (env.GoalId != null ? " " + env.GoalId : "");
            if (env.Type == MsgType.Result || env.Type == MsgType.Rejected || env.Type == MsgType.Ack)
            {
                var p = env.Payload;
                string extra = (string)p["status"] ?? (string)p["reason"] ?? ((bool?)p["ok"] == false ? (string)p["message"] : null);
                if (!string.IsNullOrEmpty(extra)) line += ": " + extra;
            }
            _recent.Enqueue(line);
            while (_recent.Count > 8) _recent.Dequeue();
            if (logMessages) Debug.Log("[RemoteControl] " + line, this);
        }

        const int OverlayMaxChars = 44;

        /// <summary>"localhost:8765/motion", "127.0.0.1:1883/rc": the address without scheme or credentials.</summary>
        static string ShortAddress(IRobotTransport t)
        {
            string url = t is WebSocketRobotTransport ws ? ws.Url : t is MqttRobotTransport mq ? mq.Url : null;
            if (string.IsNullOrEmpty(url)) return "";
            int scheme = url.IndexOf("://", StringComparison.Ordinal);
            if (scheme >= 0) url = url.Substring(scheme + 3);
            int at = url.IndexOf('@');
            return at >= 0 ? url.Substring(at + 1) : url;
        }

        static string CommName(IRobotTransport t) =>
            t is MqttRobotTransport ? "MQTT" : t is WebSocketRobotTransport ? "WebSocket" : t?.GetType().Name ?? "none";

        string StatusText() =>
            Session.Connected
                ? (Session.Welcomed ? "● connected" : "◐ connecting")
                : "○ offline" + (Session.LastDisconnectReason != null ? "  (" + Session.LastDisconnectReason + ")" : "");

        void OnGUI()
        {
            if (Session == null) return;
            var ev = Event.current;
            var cam = Camera.main;
            if (ev.type == EventType.MouseDown && ev.button == 0 && cam != null)
            {
                if (ev.control)
                {
                    CreateTargetAt(ev.mousePosition, cam);
                    ev.Use();
                    return;
                }
                var hitTarget = TargetAt(ev.mousePosition, cam);
                if (hitTarget != null)
                {
                    SelectTarget(hitTarget);
                    ev.Use();
                }
            }
            var clicked = _markers?.OnGUI(cam);
            if (clicked != null)
            {
                if (_markers.IsSelectable(clicked)) SelectMarker(clicked, "click");   // target in the controller
                if (_markers.IsEditable(clicked)) FocusInEditor(clicked);              // Move / Rotate it in the Scene view
            }
            if (!showOverlay) return;
            if (_style == null || _style.fontSize != overlayFontSize)
                _style = new GUIStyle(GUI.skin.box) { alignment = TextAnchor.UpperLeft, fontSize = overlayFontSize, wordWrap = false };

            var e = Event.current;
            if (e.type == EventType.KeyDown && e.keyCode == overlayToggleKey && overlayToggleKey != KeyCode.None)
            {
                overlayExpanded = !overlayExpanded;
                e.Use();
            }

            var ex = Session.Executor;
            var lines = new List<string>();
            if (!overlayExpanded)
            {
                lines.Add($"[+] {DisplayName}   {CommName(Session.Transport)}   {StatusText()}   {ex.State}");
            }
            else
            {
                lines.Add($"[-] {DisplayName} ({RobotId})");
                lines.Add($"data: {Session.Project} / {Session.Stage} / {RobotId}");
                lines.Add($"comm: {CommName(Session.Transport)} {ShortAddress(Session.Transport)}");
                lines.Add(StatusText());
                lines.Add($"state: {ex.State}" + (ex.ActiveGoalId != null ? $"  goal {ex.ActiveGoalId}" : "")
                    + (ex.PauseReason != null ? $"  [{ex.PauseReason}]" : "")
                    + (ex.QueuedCount > 0 ? $"  +{ex.QueuedCount} queued" : ""));
                var pos = ex.StatePayload()["positions"];
                foreach (var j in ex.JointMap.Values)
                {
                    double x = pos?[j.Name]?.ToObject<double?>() ?? 0;
                    lines.Add(string.Format(CultureInfo.InvariantCulture, "  {0,-14} {1,8:0.000} {2}",
                        j.Name, x, j.Type == "prismatic" ? "m" : "rad"));
                }
                if (_recent.Count > 0)
                {
                    lines.Add("recent:");
                    lines.AddRange(_recent.Reverse().Select(l => "  " + l));
                }
            }
            var text = string.Join("\n", lines.Select(l => l.Length > OverlayMaxChars ? l.Substring(0, OverlayMaxChars - 1) + "…" : l));
            // keep the text readable on high-resolution Game views (e.g. QHD / 4K)
            var scale = Mathf.Max(1f, Screen.height / 1080f);
            var saved = GUI.matrix;
            GUI.matrix = Matrix4x4.Scale(new Vector3(scale, scale, 1f));
            var size = _style.CalcSize(new GUIContent(text));
            if (GUI.Button(new Rect(10, 10, size.x + 12, size.y + 8), text, _style))   // click anywhere on it to open / close
                overlayExpanded = !overlayExpanded;
            GUI.matrix = saved;
        }
    }
}
