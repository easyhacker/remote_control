// Put this on the root ArticulationBody of a robot. It connects to a Remote Control controller,
// announces the robot's joints, and executes motion goals (pause / resume / cancel / stop included).

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using UnityEngine;

namespace RobotMarket.RemoteControl.Unity
{
    [DisallowMultipleComponent]
    [AddComponentMenu("RobotMarket/Remote Control Robot")]
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
        public string robotId = "unity-arm";
        public string displayName = "Unity arm";

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
        [Range(8, 40)] public int overlayFontSize = 18;
        public bool logMessages = false;

        public RobotSession Session { get; private set; }

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
            Session = new RobotSession(transport, driver, robotId, displayName, decelTime);
            Session.MessageTraced += Trace;
            Session.Start();
            Debug.Log($"[RemoteControl] '{robotId}' with {driver.Joints.Count} joints " +
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
            Session.Update(Time.fixedDeltaTime, Time.realtimeSinceStartupAsDouble);
            _driver?.EndStep();
        }

        void OnDisable()
        {
            if (Session == null) return;
            if (_driver?.Trace != null) { _driver.Trace.Flush(); _driver.Trace.Dispose(); _driver.Trace = null; }
            Session.MessageTraced -= Trace;
            Session.Shutdown();
            Session = null;
        }

        void Trace(Envelope env, bool outgoing)
        {
            if (env.Type == MsgType.Feedback) return;
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

        static string CommName(IRobotTransport t) =>
            t is MqttRobotTransport ? "MQTT" : t is WebSocketRobotTransport ? "WebSocket" : t?.GetType().Name ?? "none";

        string StatusText() =>
            Session.Connected
                ? (Session.Welcomed ? "● connected" : "◐ connecting")
                : "○ offline" + (Session.LastDisconnectReason != null ? "  (" + Session.LastDisconnectReason + ")" : "");

        void OnGUI()
        {
            if (!showOverlay || Session == null) return;
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
                lines.Add($"[+] {displayName}   {CommName(Session.Transport)}   {StatusText()}   {ex.State}");
            }
            else
            {
                lines.Add($"[-] {displayName} ({robotId})");
                lines.Add($"comm: {CommName(Session.Transport)}   {_endpoint}");
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
            var text = string.Join("\n", lines);
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
