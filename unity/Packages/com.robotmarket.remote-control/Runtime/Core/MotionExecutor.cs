// Robot-side motion executor — the protocol's behaviour, independent of transport and of Unity.
// Mirrors python/remote_control/executor.py — keep them in step.
//
// Driven from ONE thread (Unity: FixedUpdate):
//   Handle(env) — a message from the controller (execute / pause / resume / cancel / stop)
//   Tick(dt)    — advance motion by dt seconds and write joint targets to the driver
// Reports go out through the Emit callback (type, goalId, payload).
//
// Interruptions never stop instantly: pause/cancel/stop ramp the goal's time scale (rate) 1 → 0 over
// DecelTime, so the motion slows along its planned path; resume ramps it back up.

using System;
using System.Collections.Generic;
using System.Linq;
using Newtonsoft.Json.Linq;

namespace RobotMarket.RemoteControl
{
    public interface IJointDriver
    {
        IReadOnlyList<Joint> Joints { get; }
        /// <summary>Measured positions by joint name (rad / m).</summary>
        IReadOnlyDictionary<string, double> ReadPositions();
        /// <summary>Command new targets (rad / m). Joints not listed keep their previous target.</summary>
        void WriteTargets(IReadOnlyDictionary<string, double> targets);
    }

    /// <summary>Perfect-tracking driver for tests.</summary>
    public sealed class FakeDriver : IJointDriver
    {
        readonly List<Joint> _joints;
        public readonly Dictionary<string, double> Positions;

        public FakeDriver(IEnumerable<Joint> joints)
        {
            _joints = joints.ToList();
            Positions = _joints.ToDictionary(j => j.Name, j => 0.0);
        }

        public IReadOnlyList<Joint> Joints => _joints;
        public IReadOnlyDictionary<string, double> ReadPositions() => new Dictionary<string, double>(Positions);

        public void WriteTargets(IReadOnlyDictionary<string, double> targets)
        {
            foreach (var kv in targets) Positions[kv.Key] = kv.Value;
        }
    }

    public sealed class MotionExecutor
    {
        sealed class Goal
        {
            public string Id;
            public GoalSpec Spec;
            public Trajectory Traj;
            public double Time;
            public double Rate = 1;
            public double TargetRate = 1;
            public string EndStatus;        // set while slowing down to cancel/stop
            public string EndMessage = "";
            public string PauseReason;
            public int NextPoint;
            public double SinceFeedback;
        }

        public delegate void EmitFn(string type, string goalId, JObject payload);

        readonly IJointDriver _driver;
        readonly EmitFn _emit;
        readonly Dictionary<string, Joint> _joints;
        readonly LinkedList<Goal> _queue = new LinkedList<Goal>();
        readonly HashSet<string> _knownIds = new HashSet<string>();
        Goal _active;
        string _lastStateKey;

        public double DecelTime { get; }
        public IReadOnlyDictionary<string, Joint> JointMap => _joints;

        public MotionExecutor(IJointDriver driver, EmitFn emit, double decelTime = 0.4)
        {
            _driver = driver;
            _emit = emit;
            DecelTime = Math.Max(decelTime, 1e-3);
            _joints = driver.Joints.ToDictionary(j => j.Name);
        }

        // ── state ────────────────────────────────────────────────────────────

        public string State
        {
            get
            {
                var g = _active;
                if (g == null) return RobotState.Idle;
                if (g.EndStatus != null) return RobotState.Stopping;
                if (g.TargetRate == 0) return g.Rate == 0 ? RobotState.Paused : RobotState.Pausing;
                return g.Rate == 1 ? RobotState.Executing : RobotState.Resuming;
            }
        }

        public string ActiveGoalId => _active?.Id;
        public string PauseReason => _active?.PauseReason;
        public int QueuedCount => _queue.Count;

        public JObject StatePayload()
        {
            var pos = _driver.ReadPositions();
            var positions = new JObject();
            foreach (var name in _joints.Keys)
                positions[name] = pos.TryGetValue(name, out var x) ? new JValue(x) : JValue.CreateNull();
            return new JObject
            {
                ["state"] = State,
                ["goal_id"] = _active?.Id,
                ["queued"] = new JArray(_queue.Select(q => q.Id)),
                ["pause_reason"] = _active?.PauseReason,
                ["positions"] = positions,
            };
        }

        void PublishState()
        {
            string key = State + "|" + (_active?.Id ?? "") + "|" + string.Join(",", _queue.Select(q => q.Id)) + "|" + (_active?.PauseReason ?? "");
            if (key == _lastStateKey) return;
            _lastStateKey = key;
            _emit(MsgType.State, null, StatePayload());
        }

        // ── incoming messages ────────────────────────────────────────────────

        public void Handle(Envelope env)
        {
            switch (env.Type)
            {
                case MsgType.Execute: OnExecute(env); break;
                case MsgType.Pause: Ack(env, Pause(env.GoalId, "requested")); break;
                case MsgType.Resume: Ack(env, Resume(env.GoalId)); break;
                case MsgType.Cancel: Ack(env, Cancel(env.GoalId)); break;
                case MsgType.Stop: Ack(env, Stop()); break;
            }
            PublishState();
        }

        void Ack(Envelope env, (bool ok, string message) r) =>
            _emit(MsgType.Ack, env.GoalId, new JObject
            {
                ["ref_seq"] = env.Seq, ["ref_type"] = env.Type, ["ok"] = r.ok, ["message"] = r.message,
            });

        void Reject(string goalId, string reason) =>
            _emit(MsgType.Rejected, goalId, new JObject { ["reason"] = reason });

        void OnExecute(Envelope env)
        {
            var goalId = env.GoalId;
            if (string.IsNullOrEmpty(goalId)) { Reject(null, "execute needs a goal_id"); return; }
            if (_knownIds.Contains(goalId)) { Reject(goalId, "duplicate goal_id"); return; }
            GoalSpec spec;
            try { spec = GoalParser.Parse(env.Payload, _joints); }
            catch (GoalException e) { Reject(goalId, e.Message); return; }

            bool busy = _active != null || _queue.Count > 0;
            if (busy && spec.OnBusy == "reject") { Reject(goalId, "robot is busy"); return; }
            if (busy && spec.OnBusy == "replace")
            {
                foreach (var q in _queue.ToList()) Finish(q, GoalStatus.Canceled, "replaced by a new goal");
                _queue.Clear();
                if (_active != null && _active.EndStatus == null)
                    BeginEnd(_active, GoalStatus.Canceled, "replaced by a new goal");
            }
            var goal = new Goal { Id = goalId, Spec = spec };
            _knownIds.Add(goalId);
            _queue.AddLast(goal);
            int position = _queue.Count - (_active != null ? 0 : 1);
            _emit(MsgType.Accepted, goalId, new JObject { ["queue_position"] = position });
            if (_active == null) StartNext();
        }

        // ── control (also used locally, e.g. by the heartbeat watchdog) ──────

        public (bool ok, string message) Pause(string goalId, string reason = "requested")
        {
            var g = _active;
            if (g == null || g.Id != goalId)
                return _queue.Any(q => q.Id == goalId) ? (false, "goal is queued, not running") : (false, "no such goal");
            if (g.EndStatus != null) return (false, "goal is ending");
            if (g.TargetRate == 0) return (true, "already paused");
            g.TargetRate = 0;
            g.PauseReason = reason;
            return (true, "");
        }

        public (bool ok, string message) Resume(string goalId)
        {
            var g = _active;
            if (g == null || g.Id != goalId) return (false, "no such goal");
            if (g.EndStatus != null) return (false, "goal is ending");
            if (g.TargetRate == 1) return (true, "already running");
            g.TargetRate = 1;
            g.PauseReason = null;
            return (true, "");
        }

        public (bool ok, string message) Cancel(string goalId)
        {
            var queued = _queue.FirstOrDefault(q => q.Id == goalId);
            if (queued != null)
            {
                _queue.Remove(queued);
                Finish(queued, GoalStatus.Canceled, "canceled while queued");
                return (true, "");
            }
            var g = _active;
            if (g == null || g.Id != goalId) return (false, "no such goal");
            if (g.EndStatus != null) return (true, "already ending");
            BeginEnd(g, GoalStatus.Canceled, "canceled");
            return (true, "");
        }

        public (bool ok, string message) Stop()
        {
            foreach (var q in _queue.ToList()) Finish(q, GoalStatus.Stopped, "robot stopped");
            _queue.Clear();
            if (_active != null) BeginEnd(_active, GoalStatus.Stopped, "robot stopped");
            return (true, "");
        }

        /// <summary>Pause whatever is running (e.g. reason "connection_lost").</summary>
        public void PauseFor(string reason)
        {
            if (_active != null && _active.EndStatus == null && _active.TargetRate != 0)
            {
                Pause(_active.Id, reason);
                PublishState();
            }
        }

        public void AbortAll(string message)
        {
            foreach (var q in _queue.ToList()) Finish(q, GoalStatus.Aborted, message);
            _queue.Clear();
            if (_active != null)
            {
                Finish(_active, GoalStatus.Aborted, message);
                _active = null;
            }
            PublishState();
        }

        static void BeginEnd(Goal g, string status, string message)
        {
            g.EndStatus = status;
            g.EndMessage = message;
            g.TargetRate = 0;   // if already paused (rate 0), the next tick finishes it
        }

        // ── motion ───────────────────────────────────────────────────────────

        void StartNext()
        {
            while (_queue.Count > 0 && _active == null)
            {
                var g = _queue.First.Value;
                _queue.RemoveFirst();
                var pos = _driver.ReadPositions();
                var start = g.Spec.JointNames.Select(n => pos[n]).ToArray();
                try
                {
                    GoalParser.CheckSegmentSpeed(g.Spec.JointNames, _joints, start, g.Spec.Positions[0], g.Spec.Times[0], "start→point 0");
                }
                catch (GoalException e)
                {
                    Finish(g, GoalStatus.Aborted, e.Message);
                    continue;
                }
                g.Traj = new Trajectory(start, g.Spec.Times, g.Spec.Positions, g.Spec.Interpolation);
                _active = g;
            }
            PublishState();
        }

        public void Tick(double dt)
        {
            if (_active == null)
            {
                if (_queue.Count > 0) StartNext();
                return;
            }
            var g = _active;

            // Ramp the time scale towards its target, integrating time with the mean rate
            double step = dt / DecelTime, r0 = g.Rate;
            if (g.Rate < g.TargetRate) g.Rate = Math.Min(g.TargetRate, g.Rate + step);
            else if (g.Rate > g.TargetRate) g.Rate = Math.Max(g.TargetRate, g.Rate - step);
            g.Time = Math.Min(g.Traj.Duration, g.Time + dt * 0.5 * (r0 + g.Rate));

            var cmd = g.Traj.Sample(g.Time);
            var targets = new Dictionary<string, double>();
            for (int j = 0; j < g.Spec.JointNames.Length; j++) targets[g.Spec.JointNames[j]] = cmd[j];
            _driver.WriteTargets(targets);

            while (g.NextPoint < g.Spec.Times.Length && g.Time >= g.Spec.Times[g.NextPoint] - 1e-9)
            {
                if (g.Spec.ReportsPoints) ReportPoint(g, g.NextPoint);
                g.NextPoint++;
            }

            if (g.Spec.ReportsProgress)
            {
                g.SinceFeedback += dt;
                if (g.SinceFeedback >= 1.0 / g.Spec.ProgressHz)
                {
                    g.SinceFeedback = 0;
                    SendFeedback(g);
                }
            }

            if (g.Time >= g.Traj.Duration)
            {
                Finish(g, GoalStatus.Succeeded, "");
                _active = null;
                StartNext();
            }
            else if (g.EndStatus != null && g.Rate == 0)
            {
                Finish(g, g.EndStatus, g.EndMessage);
                _active = null;
                StartNext();
            }
            PublishState();
        }

        double[] Measured(Goal g)
        {
            var pos = _driver.ReadPositions();
            return g.Spec.JointNames.Select(n => pos[n]).ToArray();
        }

        void ReportPoint(Goal g, int index)
        {
            var measured = Measured(g);
            var target = g.Spec.Positions[index];
            double err = 0;
            for (int j = 0; j < measured.Length; j++) err = Math.Max(err, Math.Abs(measured[j] - target[j]));
            _emit(MsgType.PointReached, g.Id, new JObject
            {
                ["point_index"] = index, ["positions"] = new JArray(measured), ["max_error"] = err,
            });
        }

        void SendFeedback(Goal g)
        {
            _emit(MsgType.Feedback, g.Id, new JObject
            {
                ["state"] = State,
                ["point_index"] = Math.Min(g.NextPoint, g.Spec.Times.Length - 1),
                ["time"] = Math.Round(g.Time, 4),
                ["duration"] = g.Traj.Duration,
                ["rate"] = Math.Round(g.Rate, 4),
                ["positions"] = new JArray(Measured(g)),
            });
        }

        void Finish(Goal g, string status, string message)
        {
            _emit(MsgType.Result, g.Id, new JObject
            {
                ["status"] = status, ["positions"] = new JArray(Measured(g)), ["message"] = message,
            });
        }
    }
}
