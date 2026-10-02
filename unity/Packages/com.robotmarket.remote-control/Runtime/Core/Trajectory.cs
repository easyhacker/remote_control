// Joint descriptions, goal validation and time-parameterised interpolation.
// Mirrors python/remote_control/trajectory.py — keep them in step.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using Newtonsoft.Json.Linq;

namespace RobotMarket.RemoteControl
{
    public sealed class Joint
    {
        public string Name;
        public string Type = "revolute";   // revolute (rad) | prismatic (m)
        public double? Lower;
        public double? Upper;
        public double? MaxVelocity;

        public JObject ToJson() => new JObject
        {
            ["name"] = Name,
            ["type"] = Type,
            ["lower"] = Lower.HasValue ? new JValue(Lower.Value) : JValue.CreateNull(),
            ["upper"] = Upper.HasValue ? new JValue(Upper.Value) : JValue.CreateNull(),
            ["max_velocity"] = MaxVelocity.HasValue ? new JValue(MaxVelocity.Value) : JValue.CreateNull(),
        };
    }

    public class GoalException : Exception
    {
        public GoalException(string message) : base(message) { }
    }

    public sealed class GoalSpec
    {
        public string[] JointNames;
        public double[] Times;
        public double[][] Positions;     // [point][joint in JointNames order]
        public string Report = "points";
        public double ProgressHz = 10;
        public string OnBusy = "queue";
        public string Interpolation = "cubic";

        public bool ReportsPoints => Report == "points" || Report == "all";
        public bool ReportsProgress => Report == "progress" || Report == "all";
    }

    public static class GoalParser
    {
        public const double SpeedPeakFactor = 1.5;
        static readonly string[] ReportModes = { "none", "points", "progress", "all" };
        static readonly string[] OnBusyModes = { "queue", "replace", "reject" };
        static readonly string[] Interpolations = { "cubic", "linear" };

        static string F(double x) => x.ToString("0.######", CultureInfo.InvariantCulture);

        public static GoalSpec Parse(JObject payload, IReadOnlyDictionary<string, Joint> joints)
        {
            var namesTok = payload["joint_names"] as JArray;
            if (namesTok == null || namesTok.Count == 0 || namesTok.Any(t => t.Type != JTokenType.String))
                throw new GoalException("joint_names must be a non-empty list of strings");
            var names = namesTok.Select(t => t.Value<string>()).ToArray();
            if (names.Distinct().Count() != names.Length)
                throw new GoalException("joint_names contains duplicates");
            foreach (var n in names)
                if (!joints.ContainsKey(n)) throw new GoalException($"unknown joint '{n}'");

            var pointsTok = payload["points"] as JArray;
            if (pointsTok == null || pointsTok.Count == 0)
                throw new GoalException("points must be a non-empty list");

            var times = new List<double>();
            var positions = new List<double[]>();
            double prevT = 0;
            for (int i = 0; i < pointsTok.Count; i++)
            {
                double t;
                double[] pos;
                try
                {
                    var p = (JObject)pointsTok[i];
                    t = p["time_from_start"].Value<double>();
                    pos = ((JArray)p["positions"]).Select(x => x.Value<double>()).ToArray();
                }
                catch (Exception)
                {
                    throw new GoalException($"point {i}: needs numeric 'positions' and 'time_from_start'");
                }
                if (pos.Length != names.Length)
                    throw new GoalException($"point {i}: {pos.Length} positions for {names.Length} joints");
                if (!(t > prevT))
                    throw new GoalException($"point {i}: time_from_start must be > {F(prevT)}");
                for (int j = 0; j < names.Length; j++)
                {
                    var jt = joints[names[j]];
                    if (jt.Lower.HasValue && pos[j] < jt.Lower.Value - 1e-9)
                        throw new GoalException($"point {i}: {names[j]}={F(pos[j])} below lower limit {F(jt.Lower.Value)}");
                    if (jt.Upper.HasValue && pos[j] > jt.Upper.Value + 1e-9)
                        throw new GoalException($"point {i}: {names[j]}={F(pos[j])} above upper limit {F(jt.Upper.Value)}");
                }
                if (positions.Count > 0)
                    CheckSegmentSpeed(names, joints, positions[positions.Count - 1], pos, t - prevT, $"points {i - 1}→{i}");
                times.Add(t);
                positions.Add(pos);
                prevT = t;
            }

            var spec = new GoalSpec
            {
                JointNames = names,
                Times = times.ToArray(),
                Positions = positions.ToArray(),
                Report = payload["report"]?.Value<string>() ?? "points",
                OnBusy = payload["on_busy"]?.Value<string>() ?? "queue",
                Interpolation = payload["interpolation"]?.Value<string>() ?? "cubic",
            };
            if (!ReportModes.Contains(spec.Report)) throw new GoalException("report must be one of " + string.Join(", ", ReportModes));
            if (!OnBusyModes.Contains(spec.OnBusy)) throw new GoalException("on_busy must be one of " + string.Join(", ", OnBusyModes));
            if (!Interpolations.Contains(spec.Interpolation)) throw new GoalException("interpolation must be one of " + string.Join(", ", Interpolations));
            var hz = payload["progress_hz"];
            if (hz != null)
            {
                if (hz.Type != JTokenType.Float && hz.Type != JTokenType.Integer) throw new GoalException("progress_hz must be a number");
                spec.ProgressHz = Math.Min(Math.Max(hz.Value<double>(), 0.5), 100.0);
            }
            return spec;
        }

        public static void CheckSegmentSpeed(IReadOnlyList<string> names, IReadOnlyDictionary<string, Joint> joints,
                                             IReadOnlyList<double> a, IReadOnlyList<double> b, double dt, string label)
        {
            for (int j = 0; j < names.Count; j++)
            {
                var vmax = joints[names[j]].MaxVelocity;
                if (!vmax.HasValue || vmax.Value <= 0) continue;
                double peak = SpeedPeakFactor * Math.Abs(b[j] - a[j]) / dt;
                if (peak > vmax.Value + 1e-9)
                    throw new GoalException(string.Format(CultureInfo.InvariantCulture,
                        "{0}: {1} would need ~{2:0.00}/s, max_velocity is {3}/s", label, names[j], peak, F(vmax.Value)));
            }
        }
    }

    /// <summary>
    /// Piecewise interpolation from a start pose through timed points.
    /// cubic: Hermite segments, zero velocity at start and end, interior velocities = mean of neighbouring
    /// slopes (0 where motion reverses), capped at 1.5× the smaller slope so each segment's peak speed stays
    /// within SpeedPeakFactor × its average speed. linear: straight segments.
    /// </summary>
    public sealed class Trajectory
    {
        readonly double[] _t;
        readonly double[][] _x;
        readonly double[][] _v;   // null for linear
        readonly int _n;

        public Trajectory(double[] start, double[] times, double[][] positions, string interpolation = "cubic")
        {
            _n = start.Length;
            _t = new double[times.Length + 1];
            _x = new double[times.Length + 1][];
            _x[0] = (double[])start.Clone();
            for (int i = 0; i < times.Length; i++)
            {
                _t[i + 1] = times[i];
                _x[i + 1] = (double[])positions[i].Clone();
            }
            _v = interpolation == "cubic" ? Velocities() : null;
        }

        public double Duration => _t[_t.Length - 1];

        double[][] Velocities()
        {
            int k = _t.Length;
            var v = new double[k][];
            for (int i = 0; i < k; i++) v[i] = new double[_n];
            for (int i = 1; i < k - 1; i++)
                for (int j = 0; j < _n; j++)
                {
                    double s0 = (_x[i][j] - _x[i - 1][j]) / (_t[i] - _t[i - 1]);
                    double s1 = (_x[i + 1][j] - _x[i][j]) / (_t[i + 1] - _t[i]);
                    if (s0 * s1 <= 0) continue;
                    double cap = GoalParser.SpeedPeakFactor * Math.Min(Math.Abs(s0), Math.Abs(s1));
                    v[i][j] = Math.Max(-cap, Math.Min(cap, 0.5 * (s0 + s1)));
                }
            return v;
        }

        public double[] Sample(double t)
        {
            if (t <= 0) return (double[])_x[0].Clone();
            if (t >= Duration) return (double[])_x[_x.Length - 1].Clone();
            int i = 1;
            while (_t[i] < t) i++;
            double t0 = _t[i - 1], h = _t[i] - t0, u = (t - t0) / h;
            var a = _x[i - 1];
            var b = _x[i];
            var r = new double[_n];
            if (_v == null)
            {
                for (int j = 0; j < _n; j++) r[j] = a[j] + (b[j] - a[j]) * u;
                return r;
            }
            double u2 = u * u, u3 = u2 * u;
            double h00 = 2 * u3 - 3 * u2 + 1, h10 = u3 - 2 * u2 + u, h01 = -2 * u3 + 3 * u2, h11 = u3 - u2;
            var va = _v[i - 1];
            var vb = _v[i];
            for (int j = 0; j < _n; j++)
                r[j] = h00 * a[j] + h10 * h * va[j] + h01 * b[j] + h11 * h * vb[j];
            return r;
        }
    }
}
