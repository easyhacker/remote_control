// Remote Control Motion Protocol v1 — envelope, message types, channels.
// Mirrors python/remote_control/protocol.py. Spec: PROTOCOL.md at the repository root.
// This assembly has no UnityEngine references so it also builds and tests under plain .NET.

using System;
using System.Collections.Generic;
using System.Globalization;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace RobotMarket.RemoteControl
{
    public static class MsgType
    {
        // robot → controller
        public const string Hello = "hello";
        public const string Accepted = "accepted";
        public const string Rejected = "rejected";
        public const string PointReached = "point_reached";
        public const string Feedback = "feedback";
        public const string Result = "result";
        public const string State = "state";
        public const string Ack = "ack";
        public const string Description = "description";
        public const string Selected = "selected";
        public const string Edited = "edited";
        // controller → robot
        public const string Welcome = "welcome";
        public const string Execute = "execute";
        public const string Pause = "pause";
        public const string Resume = "resume";
        public const string Cancel = "cancel";
        public const string Stop = "stop";
        public const string Describe = "describe";
        public const string Visualize = "visualize";
        public const string Target = "target";
        // both
        public const string Heartbeat = "heartbeat";
    }

    public enum Channel { Command, Control, Status }

    public static class GoalStatus
    {
        public const string Succeeded = "succeeded";
        public const string Canceled = "canceled";
        public const string Stopped = "stopped";
        public const string Aborted = "aborted";
    }

    public static class RobotState
    {
        public const string Idle = "idle";
        public const string Executing = "executing";
        public const string Pausing = "pausing";
        public const string Paused = "paused";
        public const string Resuming = "resuming";
        public const string Stopping = "stopping";
    }

    public class ProtocolException : Exception
    {
        public ProtocolException(string message) : base(message) { }
    }

    public sealed class Envelope
    {
        public const int Version = 1;

        public string Type;
        public string RobotId;
        public string GoalId;
        public long Seq;
        public double Ts;
        public JObject Payload = new JObject();

        public Envelope() { }

        public Envelope(string type, string robotId, JObject payload = null, string goalId = null)
        {
            Type = type;
            RobotId = robotId;
            Payload = payload ?? new JObject();
            GoalId = goalId;
        }

        public string ToJson()
        {
            var o = new JObject
            {
                ["v"] = Version,
                ["type"] = Type,
                ["robot_id"] = RobotId,
                ["seq"] = Seq,
                ["ts"] = Ts,
                ["payload"] = Payload ?? new JObject(),
            };
            if (GoalId != null) o["goal_id"] = GoalId;
            return o.ToString(Formatting.None);
        }

        public static Envelope FromJson(string text)
        {
            JObject o;
            try { o = JObject.Parse(text); }
            catch (JsonException e) { throw new ProtocolException("invalid JSON: " + e.Message); }

            var v = o["v"];
            if (v == null || v.Type != JTokenType.Integer || v.Value<int>() != Version)
                throw new ProtocolException("unsupported protocol version " + (v?.ToString() ?? "null"));
            var type = o["type"];
            var robotId = o["robot_id"];
            if (type == null || type.Type != JTokenType.String || robotId == null || robotId.Type != JTokenType.String)
                throw new ProtocolException("message needs string 'type' and 'robot_id'");
            var payload = o["payload"];
            if (payload != null && payload.Type != JTokenType.Object && payload.Type != JTokenType.Null)
                throw new ProtocolException("'payload' must be an object");
            var goal = o["goal_id"];
            return new Envelope
            {
                Type = type.Value<string>(),
                RobotId = robotId.Value<string>(),
                GoalId = goal == null || goal.Type == JTokenType.Null ? null : goal.ToString(),
                Seq = o["seq"]?.Type == JTokenType.Integer ? o["seq"].Value<long>() : 0,
                Ts = o["ts"] != null && (o["ts"].Type == JTokenType.Float || o["ts"].Type == JTokenType.Integer)
                    ? o["ts"].Value<double>() : 0.0,
                Payload = payload as JObject ?? new JObject(),
            };
        }

        public static Channel ChannelOf(string type, bool fromRobot)
        {
            if (fromRobot) return Channel.Status;
            return type == MsgType.Execute ? Channel.Command : Channel.Control;
        }

        public override string ToString() =>
            string.Format(CultureInfo.InvariantCulture, "{0}#{1} goal={2}", Type, Seq, GoalId ?? "-");
    }

    /// <summary>Stamps outgoing envelopes with seq/ts. Reset on every (re)connection.</summary>
    public sealed class Sequencer
    {
        long _next = 1;
        public void Reset() => _next = 1;

        public Envelope Stamp(Envelope env)
        {
            env.Seq = _next++;
            env.Ts = (DateTime.UtcNow - new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc)).TotalSeconds;
            return env;
        }
    }

    /// <summary>
    /// Drops duplicate incoming messages (MQTT QoS 1 may deliver twice). Transports with several topics
    /// (MQTT cmd + ctrl) don't keep order across them, so out-of-order numbers are accepted; only ones
    /// already seen, or older than Window, are rejected.
    /// </summary>
    public sealed class SeqTracker
    {
        public const long Window = 1024;
        long _max;
        HashSet<long> _seen = new HashSet<long>();

        public void Reset()
        {
            _max = 0;
            _seen = new HashSet<long>();
        }

        public bool Accept(long seq)
        {
            if (seq <= 0) return true;
            if (seq <= _max - Window || !_seen.Add(seq)) return false;
            if (seq > _max) _max = seq;
            if (_seen.Count > 2 * Window) _seen.RemoveWhere(s => s <= _max - Window);
            return true;
        }
    }
}
