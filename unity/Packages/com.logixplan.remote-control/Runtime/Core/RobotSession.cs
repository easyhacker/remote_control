// Robot-side session: binds a transport + MotionExecutor + driver; handles hello/welcome, sequence
// numbers, heartbeats and the connection watchdog. Mirrors python/remote_control/robot.py.
//
// Call Update(dt) from one thread at a steady rate (Unity: FixedUpdate). Nothing here touches Unity.

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using Newtonsoft.Json.Linq;

namespace RobotMarket.RemoteControl
{
    public sealed class RobotSession : IDisposable
    {
        public const string SoftwareVersion = "remote-control-unity/0.1";
        public const string DefaultName = "default";
        public const string Frame = "ros: x forward, y left, z up; metres; orientation quaternion [x, y, z, w]";

        /// <summary>The id the controller knows this robot by; it can change once if the controller already
        /// has a robot with the requested id (see <see cref="Renamed"/>).</summary>
        public string RobotId { get; private set; }

        /// <summary>Random per session: tells the controller a reconnect of this robot from a different robot
        /// that happens to use the same id.</summary>
        public string Instance { get; } = Guid.NewGuid().ToString("N").Substring(0, 12);

        /// <summary>Raised on the Update thread when the controller assigned a new id (old, new).</summary>
        public event Action<string, string> Renamed;

        /// <summary>
        /// Shows / updates / removes markers sent by the controller (`visualize` payload: items + replace).
        /// Return null on success or an error message. Leave null if this robot has no viewer.
        /// </summary>
        public Func<JObject, string> Visualizer;

        /// <summary>
        /// Creates / updates / deletes targets owned by the robot's scene (`target` payload: op, …). Return null on
        /// success or an error message. Leave null if targets live with the controller.
        /// </summary>
        public Func<JObject, string> TargetHandler;
        public string DisplayName { get; }

        /// <summary>Names reported in hello / description; the controller files this robot's data under
        /// &lt;data_dir&gt;/&lt;Project&gt;/&lt;Stage&gt;/&lt;RobotId&gt;. Set before Start().</summary>
        public string Project = DefaultName;
        public string Stage = DefaultName;

        /// <summary>
        /// Adds the robot's geometry to `description` replies: called with tree = false (base_pose only) or
        /// true (also root / links / joints, see PROTOCOL.md). Runs on the Update thread. Without it the reply
        /// lists the joints without geometry.
        /// </summary>
        public Func<bool, JObject> Describer;
        public IRobotTransport Transport { get; }
        public MotionExecutor Executor { get; }

        public double HeartbeatInterval { get; private set; } = 0.5;
        public double HeartbeatTimeout { get; private set; } = 2.0;
        public bool Welcomed { get => _welcomed; private set => _welcomed = value; }
        volatile bool _welcomed;

        /// <summary>
        /// Heartbeats normally go out from Update. A slow frame (a render or Editor hitch) can block that thread
        /// longer than the controller's timeout, which would drop a healthy robot; so a background timer also sends
        /// them, but only while Update has run within this many seconds. A real freeze still stops the heartbeats and
        /// the controller drops the robot. 0 = Update only. Set before Start().
        /// </summary>
        public double BackgroundHeartbeatMaxStall = 10.0;

        Timer _heartbeatTimer;
        long _lastUpdateTicks, _lastHeartbeatTicks;
        public string LastDisconnectReason { get; private set; }
        public string LastProtocolError { get; private set; }

        /// <summary>Raised on the Update thread for every message received / sent (for logging UIs).</summary>
        public event Action<Envelope, bool> MessageTraced;   // (envelope, outgoing)

        /// <summary>Raised on the Update thread when the transport connects (true) or drops (false, reason).</summary>
        public event Action<bool, string> ConnectionChanged;

        readonly IJointDriver _driver;
        readonly Sequencer _seq = new Sequencer();
        readonly SeqTracker _rx = new SeqTracker();
        readonly List<(string type, string goalId, JObject payload)> _outbox = new List<(string, string, JObject)>();
        double _now, _lastRx, _lastHeartbeat, _lastHello;

        public RobotSession(IRobotTransport transport, IJointDriver driver, string robotId,
                            string displayName = null, double decelTime = 0.4)
        {
            Transport = transport;
            _driver = driver;
            RobotId = robotId;
            DisplayName = string.IsNullOrEmpty(displayName) ? robotId : displayName;
            transport.Bind(robotId);
            Executor = new MotionExecutor(driver, (t, g, p) => _outbox.Add((t, g, p)), decelTime);
        }

        public bool Connected => Transport.Connected;

        public void Start()
        {
            Interlocked.Exchange(ref _lastUpdateTicks, Stopwatch.GetTimestamp());
            Transport.Start();
            if (BackgroundHeartbeatMaxStall > 0 && _heartbeatTimer == null)
                _heartbeatTimer = new Timer(_ => BackgroundHeartbeat(), null, 100, 100);
        }

        /// <summary>Timer thread: send a heartbeat if Update has fallen behind, but not if it has stopped.</summary>
        void BackgroundHeartbeat()
        {
            try
            {
                if (!Transport.Connected || !_welcomed) return;
                long now = Stopwatch.GetTimestamp();
                double sinceUpdate = (now - Interlocked.Read(ref _lastUpdateTicks)) / (double)Stopwatch.Frequency;
                double sinceHeartbeat = (now - Interlocked.Read(ref _lastHeartbeatTicks)) / (double)Stopwatch.Frequency;
                if (sinceUpdate > BackgroundHeartbeatMaxStall || sinceHeartbeat < HeartbeatInterval) return;
                Interlocked.Exchange(ref _lastHeartbeatTicks, now);
                // seq 0 = unsequenced: the sequence counter belongs to the Update thread
                var env = new Envelope(MsgType.Heartbeat, RobotId) { Seq = 0, Ts = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0 };
                Transport.Send(env.ToJson(), Channel.Control);
            }
            catch (Exception) { /* transport closing; the next tick or Update retries */ }
        }

        /// <summary>Advance by dt seconds: process incoming messages, move, heartbeat, send.</summary>
        public void Update(double dt) => Update(dt, _now + dt);

        /// <summary>
        /// As Update(dt), but heartbeats and the connection watchdog follow <paramref name="clock"/> (seconds, wall
        /// time). Simulators must pass real time here: their simulated time falls behind the wall clock whenever a
        /// frame is slow (Unity caps each frame's physics at Time.maximumDeltaTime), and heartbeats paced by
        /// simulated time then arrive too late for the controller's timeout.
        /// </summary>
        public void Update(double dt, double clock)
        {
            _now = clock;
            Interlocked.Exchange(ref _lastUpdateTicks, Stopwatch.GetTimestamp());
            while (Transport.Poll(out var ev))
            {
                switch (ev.Kind)
                {
                    case TransportEventKind.Connected: OnConnected(); break;
                    case TransportEventKind.Disconnected: OnDisconnected(ev.Text); break;
                    case TransportEventKind.Message: OnMessage(ev.Text); break;
                }
            }

            Executor.Tick(dt);

            if (Transport.Connected)
            {
                if (_now - _lastRx > HeartbeatTimeout)
                {
                    Executor.PauseFor("connection_lost");
                    if (Welcomed)   // the controller lost us — announce again until welcomed
                    {
                        Welcomed = false;
                        _lastHello = double.NegativeInfinity;
                    }
                }
                if (!Welcomed && _now - _lastHello >= Math.Max(1.0, HeartbeatTimeout)) SendHello();
                if (_now - _lastHeartbeat >= HeartbeatInterval)
                {
                    _lastHeartbeat = _now;
                    _outbox.Add((MsgType.Heartbeat, null, new JObject()));
                }
            }
            Flush();
        }

        /// <summary>Abort all goals, tell the controller, and close the connection.</summary>
        public void Shutdown(string reason = "robot shutting down")
        {
            _heartbeatTimer?.Dispose();
            _heartbeatTimer = null;
            Executor.AbortAll(reason);
            Flush();
            Transport.Stop();
        }

        public void Dispose() => Shutdown();

        public JObject HelloPayload() => new JObject
        {
            ["protocol"] = Envelope.Version,
            ["name"] = DisplayName,
            ["software"] = SoftwareVersion,
            ["joints"] = new JArray(_driver.Joints.Select(j => j.ToJson())),
            ["supports"] = new JObject
            {
                ["pause"] = true, ["report_points"] = true, ["report_progress"] = true, ["pose_targets"] = false,
                ["describe"] = true, ["parallel_goals"] = true, ["visualize"] = Visualizer != null, ["targets"] = TargetHandler != null,
            },
            ["instance"] = Instance,
            ["project"] = Project,
            ["stage"] = Stage,
            ["state"] = Executor.StatePayload(),
        };

        /// <summary>Body of a `description` reply (without ref_seq / ok / message).</summary>
        public JObject Describe(bool tree)
        {
            var state = Executor.StatePayload();
            var d = new JObject
            {
                ["project"] = Project, ["stage"] = Stage, ["robot"] = RobotId, ["name"] = DisplayName,
                ["software"] = SoftwareVersion, ["frame"] = Frame,
                ["base_pose"] = new JObject { ["position"] = new JArray(0.0, 0.0, 0.0), ["orientation"] = new JArray(0.0, 0.0, 0.0, 1.0) },
                ["positions"] = state["positions"],
                ["state"] = state["state"],
            };
            if (Describer != null)
            {
                d.Merge(Describer(tree), new JsonMergeSettings { MergeArrayHandling = MergeArrayHandling.Replace });
                return d;
            }
            if (!tree) return d;
            var pos = _driver.ReadPositions();
            d["root"] = null;
            d["links"] = new JArray();
            d["joints"] = new JArray(_driver.Joints.Select(j => new JObject
            {
                ["name"] = j.Name, ["type"] = j.Type, ["parent"] = null, ["child"] = null, ["command_name"] = j.Name,
                ["lower"] = j.Lower, ["upper"] = j.Upper, ["max_velocity"] = j.MaxVelocity,
                ["position"] = pos.TryGetValue(j.Name, out var x) ? x : (double?)null,
            }));
            return d;
        }

        void OnConnected()
        {
            _seq.Reset();
            _rx.Reset();
            _lastRx = _now;
            _outbox.Clear();   // superseded by hello.state
            ConnectionChanged?.Invoke(true, null);
            SendHello();
            Flush();
        }

        void SendHello()
        {
            Welcomed = false;
            _lastHello = _now;
            _outbox.Add((MsgType.Hello, null, HelloPayload()));
        }

        void OnDisconnected(string reason)
        {
            LastDisconnectReason = reason;
            Welcomed = false;
            Executor.PauseFor("connection_lost");
            ConnectionChanged?.Invoke(false, reason);
        }

        void OnMessage(string text)
        {
            Envelope env;
            try { env = Envelope.FromJson(text); }
            catch (ProtocolException e) { LastProtocolError = e.Message; return; }
            if (env.RobotId != RobotId) return;
            if (env.Type == MsgType.Welcome) _rx.Reset();
            if (!_rx.Accept(env.Seq)) return;
            _lastRx = _now;
            if (env.Type != MsgType.Heartbeat) MessageTraced?.Invoke(env, false);

            if (env.Type == MsgType.Welcome)
            {
                var newId = env.Payload["robot_id"]?.Value<string>();
                var forInstance = env.Payload["instance"]?.Value<string>();
                if (!string.IsNullOrEmpty(newId) && newId != RobotId && (forInstance == null || forInstance == Instance))
                {
                    Rename(newId);   // the controller already has a robot with our id
                    return;
                }
                Welcomed = true;
                HeartbeatInterval = env.Payload["heartbeat_interval"]?.Value<double>() ?? HeartbeatInterval;
                HeartbeatTimeout = env.Payload["heartbeat_timeout"]?.Value<double>() ?? HeartbeatTimeout;
            }
            else if (env.Type == MsgType.Visualize || env.Type == MsgType.Target)
            {
                var handler = env.Type == MsgType.Visualize ? Visualizer : TargetHandler;
                string error;
                if (handler == null) error = env.Type + " not supported";
                else
                {
                    try { error = handler(env.Payload); }
                    catch (Exception e) { error = env.Type + " failed: " + e.Message; }
                }
                _outbox.Add((MsgType.Ack, null, new JObject
                {
                    ["ref_seq"] = env.Seq, ["ref_type"] = env.Type, ["ok"] = error == null, ["message"] = error ?? "",
                }));
            }
            else if (env.Type == MsgType.Describe)
            {
                JObject reply;
                try
                {
                    reply = Describe(env.Payload["tree"]?.Value<bool>() ?? true);
                    reply["ok"] = true;
                    reply["message"] = "";
                }
                catch (Exception e)   // report, don't drop the connection
                {
                    reply = new JObject { ["ok"] = false, ["message"] = "describe failed: " + e.Message };
                }
                reply.AddFirst(new JProperty("ref_seq", env.Seq));
                _outbox.Add((MsgType.Description, null, reply));
            }
            else if (env.Type != MsgType.Heartbeat)
            {
                Executor.Handle(env);
            }
        }

        /// <summary>Report that the user picked a visualized item (e.g. clicked a frame).</summary>
        public void Select(string itemId, string source = "user")
        {
            _outbox.Add((MsgType.Selected, null, new JObject { ["id"] = itemId, ["source"] = source }));
            Flush();
        }

        /// <summary>Report that the user moved an editable item: pose in its parent link (ROS convention).</summary>
        public void Edited(string itemId, string parent, JObject pose, string source = "user")
        {
            _outbox.Add((MsgType.Edited, null, new JObject
            {
                ["id"] = itemId, ["parent"] = parent, ["pose"] = pose, ["source"] = source,
            }));
            Flush();
        }

        void Rename(string newId)
        {
            var old = RobotId;
            Welcomed = false;
            Transport.Stop();
            RobotId = newId;
            Transport.Bind(newId);
            Transport.Start();
            Renamed?.Invoke(old, newId);
        }

        void Flush()
        {
            if (_outbox.Count == 0) return;
            var items = _outbox.ToArray();
            _outbox.Clear();
            if (!Transport.Connected) return;   // results of goals finished while offline are lost; state goes in hello
            foreach (var (type, goalId, payload) in items)
            {
                var env = _seq.Stamp(new Envelope(type, RobotId, payload, goalId));
                Transport.Send(env.ToJson(), Envelope.ChannelOf(type, fromRobot: true));
                if (type == MsgType.Heartbeat) Interlocked.Exchange(ref _lastHeartbeatTicks, Stopwatch.GetTimestamp());
                if (type != MsgType.Heartbeat) MessageTraced?.Invoke(env, true);
            }
        }
    }
}
