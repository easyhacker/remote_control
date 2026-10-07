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

    /// <summary>
    /// Runs goals. Several goals run at once when they move different joints (on_busy "parallel", e.g. the two arms
    /// of a dual-arm robot), each paused / resumed / cancelled on its own; stop ends all. "queue" goals wait until
    /// the robot is idle. Mirrors python/remote_control/executor.py — keep them in step.
    /// </summary>
    public sealed class MotionExecutor
    {
        /// <summary>One accepted goal: queued (Traj is null) or running (in _actives). Time advances by
        /// dt × Rate; Rate ramps towards TargetRate (1 = run, 0 = halt), which is how pause / cancel / stop slow it
        /// down along the path. Each goal has its own rate, so pausing one does not affect a parallel one.</summary>
        sealed class Goal
        {
            public string Id;
            public GoalSpec Spec;
            public HashSet<string> JointSet;    // running goals never share a joint
            public Trajectory Traj;
            public double Time;                 // position on the goal's timeline (s)
            public double Rate = 1;             // current time scale
            public double TargetRate = 1;       // time scale being ramped to
            public string EndStatus;        // set while slowing down to cancel/stop
            public string EndMessage = "";
            public string PauseReason;
            public int NextPoint;
            public double SinceFeedback;

            // on_busy "parallel": may run next to goals on other joints; other modes run only when idle
            public bool Parallel => Spec.OnBusy == "parallel";

            public string State
            {
                get
                {
                    if (EndStatus != null) return RobotState.Stopping;
                    if (TargetRate == 0) return Rate == 0 ? RobotState.Paused : RobotState.Pausing;
                    return Rate == 1 ? RobotState.Executing : RobotState.Resuming;
                }
            }
        }

        public delegate void EmitFn(string type, string goalId, JObject payload);

        readonly IJointDriver _driver;
        readonly EmitFn _emit;
        readonly Dictionary<string, Joint> _joints;
        readonly List<Goal> _queue = new List<Goal>();
        readonly List<Goal> _actives = new List<Goal>();
        readonly HashSet<string> _knownIds = new HashSet<string>();
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

        Goal FindActive(string goalId) => _actives.FirstOrDefault(g => g.Id == goalId);

        // ── state ────────────────────────────────────────────────────────────

        /// <summary>Idle, or the "busiest" state of the running goals.</summary>
        public string State
        {
            get
            {
                if (_actives.Count == 0) return RobotState.Idle;
                // "paused" only when every running goal is; one moving goal makes the robot "executing" etc.
                var states = new HashSet<string>(_actives.Select(g => g.State));
                foreach (var s in new[] { RobotState.Executing, RobotState.Resuming, RobotState.Pausing, RobotState.Stopping })
                    if (states.Contains(s)) return s;
                return RobotState.Paused;
            }
        }

        public string ActiveGoalId => _actives.Count > 0 ? _actives[0].Id : null;
        public IReadOnlyList<string> ActiveGoalIds => _actives.Select(g => g.Id).ToList();
        public string PauseReason => _actives.FirstOrDefault(g => g.PauseReason != null)?.PauseReason;
        public int QueuedCount => _queue.Count;

        public JObject StatePayload()
        {
            var pos = _driver.ReadPositions();
            var positions = new JObject();
            foreach (var name in _joints.Keys)
                positions[name] = pos.TryGetValue(name, out var x) ? new JValue(x) : JValue.CreateNull();
            var goals = new JObject();
            foreach (var g in _actives)
                goals[g.Id] = new JObject
                {
                    ["state"] = g.State, ["joints"] = new JArray(g.Spec.JointNames), ["pause_reason"] = g.PauseReason,
                };
            // goal_id / pause_reason keep the single-goal meaning for older controllers; "active" and "goals"
            // list every running goal for controllers that use parallel goals.
            return new JObject
            {
                ["state"] = State,
                ["goal_id"] = ActiveGoalId,
                ["active"] = new JArray(_actives.Select(g => g.Id)),
                ["goals"] = goals,
                ["queued"] = new JArray(_queue.Select(q => q.Id)),
                ["pause_reason"] = PauseReason,
                ["positions"] = positions,
            };
        }

        void PublishState()
        {
            string key = State + "|" + string.Join(",", _actives.Select(g => g.Id + ":" + g.State + ":" + g.PauseReason))
                         + "|" + string.Join(",", _queue.Select(q => q.Id));
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

            bool busy = _actives.Count > 0 || _queue.Count > 0;
            if (busy && spec.OnBusy == "reject") { Reject(goalId, "robot is busy"); return; }
            if (busy && spec.OnBusy == "replace")
            {
                foreach (var q in _queue.ToList()) Finish(q, GoalStatus.Canceled, "replaced by a new goal");
                _queue.Clear();
                foreach (var a in _actives)
                    if (a.EndStatus == null) BeginEnd(a, GoalStatus.Canceled, "replaced by a new goal");
            }
            // Every goal enters the queue; StartReady() decides whether it may start right away.
            var goal = new Goal { Id = goalId, Spec = spec, JointSet = new HashSet<string>(spec.JointNames) };
            _knownIds.Add(goalId);
            _queue.Add(goal);
            StartReady();
            int position;
            if (_actives.Contains(goal)) position = 0;
            else   // goals that still have to finish before this one can start
            {
                int index = _queue.IndexOf(goal);
                int ahead = _actives.Count(a => !goal.Parallel || a.JointSet.Overlaps(goal.JointSet))
                            + _queue.Take(Math.Max(0, index)).Count(q => !goal.Parallel || q.JointSet.Overlaps(goal.JointSet));
                position = Math.Max(1, ahead);
            }
            _emit(MsgType.Accepted, goalId, new JObject { ["queue_position"] = position });
        }

        // ── control (also used locally, e.g. by the heartbeat watchdog) ──────

        public (bool ok, string message) Pause(string goalId, string reason = "requested")
        {
            var g = FindActive(goalId);
            if (g == null)
                return _queue.Any(q => q.Id == goalId) ? (false, "goal is queued, not running") : (false, "no such goal");
            if (g.EndStatus != null) return (false, "goal is ending");
            if (g.TargetRate == 0) return (true, "already paused");
            g.TargetRate = 0;
            g.PauseReason = reason;
            return (true, "");
        }

        public (bool ok, string message) Resume(string goalId)
        {
            var g = FindActive(goalId);
            if (g == null) return (false, "no such goal");
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
            var g = FindActive(goalId);
            if (g == null) return (false, "no such goal");
            if (g.EndStatus != null) return (true, "already ending");
            BeginEnd(g, GoalStatus.Canceled, "canceled");
            return (true, "");
        }

        public (bool ok, string message) Stop()
        {
            foreach (var q in _queue.ToList()) Finish(q, GoalStatus.Stopped, "robot stopped");
            _queue.Clear();
            foreach (var a in _actives) BeginEnd(a, GoalStatus.Stopped, "robot stopped");
            return (true, "");
        }

        /// <summary>Pause everything that is running (e.g. reason "connection_lost").</summary>
        public void PauseFor(string reason)
        {
            bool changed = false;
            foreach (var a in _actives)
                if (a.EndStatus == null && a.TargetRate != 0) { Pause(a.Id, reason); changed = true; }
            if (changed) PublishState();
        }

        public void AbortAll(string message)
        {
            foreach (var q in _queue.ToList()) Finish(q, GoalStatus.Aborted, message);
            _queue.Clear();
            foreach (var a in _actives.ToList()) Finish(a, GoalStatus.Aborted, message);
            _actives.Clear();
            PublishState();
        }

        static void BeginEnd(Goal g, string status, string message)
        {
            g.EndStatus = status;
            g.EndMessage = message;
            g.TargetRate = 0;   // if already paused (rate 0), the next tick finishes it
        }

        // ── motion ───────────────────────────────────────────────────────────

        /// <summary>Start queued goals that may run now, in queue order: a "parallel" goal when none of its joints
        /// is used by a running goal or an earlier queued goal; any other goal only when idle and first in line.
        /// busy collects the joints of running goals and of goals skipped so far, so goals on the same joints start
        /// in the order they arrived, while a goal on free joints may overtake them.</summary>
        void StartReady()
        {
            var busy = new HashSet<string>(_actives.SelectMany(a => a.JointSet));
            foreach (var g in _queue.ToList())
            {
                bool ok = g.Parallel ? !g.JointSet.Overlaps(busy) : _actives.Count == 0 && _queue[0] == g;
                if (ok)
                {
                    _queue.Remove(g);
                    if (Begin(g)) busy.UnionWith(g.JointSet);
                    continue;
                }
                if (!g.Parallel) break;   // a sequential goal waits for idle; everything behind it waits too
                busy.UnionWith(g.JointSet);
            }
            PublishState();
        }

        /// <summary>Start a goal from the joints' current positions. The move to the first point is checked
        /// against max velocity only now, since the start is not known earlier. False if it was aborted.</summary>
        bool Begin(Goal g)
        {
            var pos = _driver.ReadPositions();
            var start = g.Spec.JointNames.Select(n => pos[n]).ToArray();
            try
            {
                GoalParser.CheckSegmentSpeed(g.Spec.JointNames, _joints, start, g.Spec.Positions[0], g.Spec.Times[0], "start→point 0");
            }
            catch (GoalException e)
            {
                Finish(g, GoalStatus.Aborted, e.Message);
                return false;
            }
            g.Traj = new Trajectory(start, g.Spec.Times, g.Spec.Positions, g.Spec.Interpolation);
            _actives.Add(g);
            return true;
        }

        public void Tick(double dt)
        {
            if (_actives.Count == 0)
            {
                if (_queue.Count > 0) StartReady();
                return;
            }
            // Each running goal writes only its own joints; a finished goal frees its joints for queued goals.
            bool finished = false;
            foreach (var g in _actives.ToList())
                if (TickGoal(g, dt)) { _actives.Remove(g); finished = true; }
            if (finished || _queue.Count > 0) StartReady();
            PublishState();
        }

        /// <summary>Advance one goal; true when it has ended.</summary>
        bool TickGoal(Goal g, double dt)
        {
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
                return true;
            }
            if (g.EndStatus != null && g.Rate == 0)
            {
                Finish(g, g.EndStatus, g.EndMessage);
                return true;
            }
            return false;
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
                ["state"] = g.State,
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
