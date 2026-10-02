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
        [Header("Connection")]
        [Tooltip("Controller endpoint, e.g. ws://localhost:8765/motion")]
        public string controllerUrl = "ws://localhost:8765/motion";
        public string robotId = "unity-arm";
        public string displayName = "Unity arm";

        [Header("Joints")]
        [Tooltip("Root of the articulation; defaults to this GameObject's ArticulationBody")]
        public ArticulationBody articulationRoot;
        [Tooltip("Used for joints without an override (rad/s or m/s). 0 = unlimited")]
        public float defaultMaxVelocity = 2.0f;
        public List<JointOverride> jointOverrides = new List<JointOverride>();

        [Header("Motion")]
        [Tooltip("Seconds to slow to a halt on pause / cancel / stop")]
        [Min(0.01f)] public float decelTime = 0.4f;

        [Header("Debug")]
        public bool showOverlay = true;
        public bool logMessages = false;

        public RobotSession Session { get; private set; }

        readonly Queue<string> _recent = new Queue<string>();
        GUIStyle _style;

        void OnEnable()
        {
            ApplyCommandLine();
            if (articulationRoot == null) articulationRoot = GetComponent<ArticulationBody>();
            if (articulationRoot == null)
            {
                Debug.LogError("[RemoteControl] needs an ArticulationBody root", this);
                enabled = false;
                return;
            }
            IRobotTransport transport;
            try { transport = TransportFactory.FromUrl(controllerUrl); }
            catch (Exception e)
            {
                Debug.LogError($"[RemoteControl] {e.Message}", this);
                enabled = false;
                return;
            }
            var driver = new ArticulationJointDriver(articulationRoot, jointOverrides, defaultMaxVelocity);
            Session = new RobotSession(transport, driver, robotId, displayName, decelTime);
            Session.MessageTraced += Trace;
            Session.Start();
            Debug.Log($"[RemoteControl] '{robotId}' with {driver.Joints.Count} joints " +
                      $"({string.Join(", ", driver.Joints.Select(j => j.Name))}) → {controllerUrl}", this);
        }

        /// <summary>Player builds: -controllerUrl ws://host:8765/motion  -robotId my-arm</summary>
        void ApplyCommandLine()
        {
            var args = Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length - 1; i++)
            {
                if (args[i] == "-controllerUrl") controllerUrl = args[i + 1];
                else if (args[i] == "-robotId") robotId = args[i + 1];
            }
        }

        void FixedUpdate() => Session?.Update(Time.fixedDeltaTime);

        void OnDisable()
        {
            if (Session == null) return;
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

        void OnGUI()
        {
            if (!showOverlay || Session == null) return;
            if (_style == null)
                _style = new GUIStyle(GUI.skin.box) { alignment = TextAnchor.UpperLeft, fontSize = 12, wordWrap = false };

            var ex = Session.Executor;
            var lines = new List<string>
            {
                $"{displayName} ({robotId})",
                Session.Connected
                    ? (Session.Welcomed ? "● connected  " : "◐ connecting  ") + controllerUrl
                    : "○ offline  " + controllerUrl + (Session.LastDisconnectReason != null ? "  (" + Session.LastDisconnectReason + ")" : ""),
                $"state: {ex.State}" + (ex.ActiveGoalId != null ? $"  goal {ex.ActiveGoalId}" : "")
                    + (ex.PauseReason != null ? $"  [{ex.PauseReason}]" : "")
                    + (ex.QueuedCount > 0 ? $"  +{ex.QueuedCount} queued" : ""),
            };
            var pos = Session.Executor.StatePayload()["positions"];
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
            var text = string.Join("\n", lines);
            var size = _style.CalcSize(new GUIContent(text));
            GUI.Box(new Rect(10, 10, size.x + 12, size.y + 8), text, _style);
        }
    }
}
