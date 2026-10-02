// Robot-side session: binds a transport + MotionExecutor + driver; handles hello/welcome, sequence
// numbers, heartbeats and the connection watchdog. Mirrors python/remote_control/robot.py.
//
// Call Update(dt) from one thread at a steady rate (Unity: FixedUpdate). Nothing here touches Unity.

using System;
using System.Collections.Generic;
using System.Linq;
using Newtonsoft.Json.Linq;

namespace RobotMarket.RemoteControl
{
    public sealed class RobotSession : IDisposable
    {
        public const string SoftwareVersion = "remote-control-unity/0.1";

        public string RobotId { get; }
        public string DisplayName { get; }
        public IRobotTransport Transport { get; }
        public MotionExecutor Executor { get; }

        public double HeartbeatInterval { get; private set; } = 0.5;
        public double HeartbeatTimeout { get; private set; } = 2.0;
        public bool Welcomed { get; private set; }
        public string LastDisconnectReason { get; private set; }
        public string LastProtocolError { get; private set; }

        /// <summary>Raised on the Update thread for every message received / sent (for logging UIs).</summary>
        public event Action<Envelope, bool> MessageTraced;   // (envelope, outgoing)

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

        public void Start() => Transport.Start();

        /// <summary>Advance by dt seconds: process incoming messages, move, heartbeat, send.</summary>
        public void Update(double dt)
        {
            _now += dt;
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
            },
            ["state"] = Executor.StatePayload(),
        };

        void OnConnected()
        {
            _seq.Reset();
            _rx.Reset();
            _lastRx = _now;
            _outbox.Clear();   // superseded by hello.state
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
                Welcomed = true;
                HeartbeatInterval = env.Payload["heartbeat_interval"]?.Value<double>() ?? HeartbeatInterval;
                HeartbeatTimeout = env.Payload["heartbeat_timeout"]?.Value<double>() ?? HeartbeatTimeout;
            }
            else if (env.Type != MsgType.Heartbeat)
            {
                Executor.Handle(env);
            }
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
                if (type != MsgType.Heartbeat) MessageTraced?.Invoke(env, true);
            }
        }
    }
}
