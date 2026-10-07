// dotnet run -- ws://127.0.0.1:8765/motion [robot_id [project stage]]   → fake robot with the test joints (also mqtt://host:port/prefix)
// dotnet run -- --selftest                               → trajectory / parser checks
using System;
using System.Diagnostics;
using System.Threading;
using Newtonsoft.Json.Linq;
using RobotMarket.RemoteControl;

static class Program
{
    static readonly Joint[] TestJoints =
    {
        new Joint { Name = "shoulder", Type = "revolute", Lower = -3.0, Upper = 3.0, MaxVelocity = 4.0 },
        new Joint { Name = "elbow", Type = "revolute", Lower = -2.0, Upper = 2.0, MaxVelocity = 4.0 },
        new Joint { Name = "slide", Type = "prismatic", Lower = 0.0, Upper = 0.5, MaxVelocity = 1.0 },
    };

    static int Main(string[] args)
    {
        if (args.Length > 0 && args[0] == "--selftest") return SelfTest();
        var url = args.Length > 0 ? args[0] : "ws://127.0.0.1:8765/motion";
        var robotId = args.Length > 1 ? args[1] : "arm-test";

        var driver = new FakeDriver(TestJoints);
        var transport = TransportFactory.FromUrl(url);
        if (transport is WebSocketRobotTransport ws) ws.MinBackoff = 0.1;
        var session = new RobotSession(transport, driver, robotId, "dotnet fake robot", decelTime: 0.1);
        if (args.Length > 3) { session.Project = args[2]; session.Stage = args[3]; }
        session.Start();
        Console.WriteLine($"fake robot '{robotId}' → {url}");

        // Exit when the parent closes our stdin (or on Ctrl+C). "stall <seconds>" blocks the update loop, like a
        // slow frame in Unity (tests that heartbeats survive it).
        var quit = new ManualResetEventSlim(false);
        double stallSeconds = 0;
        Console.CancelKeyPress += (_, e) => { e.Cancel = true; quit.Set(); };
        new Thread(() =>
        {
            string line;
            while ((line = Console.In.ReadLine()) != null)
                if (line.StartsWith("stall ")) Volatile.Write(ref stallSeconds, double.Parse(line.Substring(6), System.Globalization.CultureInfo.InvariantCulture));
            quit.Set();
        }) { IsBackground = true }.Start();

        var sw = Stopwatch.StartNew();
        double last = 0;
        while (!quit.IsSet)
        {
            Thread.Sleep(5);
            double stall = Interlocked.Exchange(ref stallSeconds, 0);
            if (stall > 0) Thread.Sleep(TimeSpan.FromSeconds(stall));
            double now = sw.Elapsed.TotalSeconds;
            session.Update(now - last);
            last = now;
        }
        session.Shutdown();
        return 0;
    }

    static int SelfTest()
    {
        int fails = 0;
        void Check(string name, bool ok) { Console.WriteLine((ok ? "PASS " : "FAIL ") + name); if (!ok) fails++; }

        var tr = new Trajectory(new[] { 0.0 }, new[] { 1.0, 2.0, 3.0 }, new[] { new[] { 1.0 }, new[] { 3.0 }, new[] { 2.0 } });
        Check("cubic passes points", Math.Abs(tr.Sample(1)[0] - 1) < 1e-9 && Math.Abs(tr.Sample(2)[0] - 3) < 1e-9 && Math.Abs(tr.Sample(3)[0] - 2) < 1e-9);
        Check("zero start velocity", Math.Abs((tr.Sample(1e-4)[0] - tr.Sample(0)[0]) / 1e-4) < 0.01);
        // Same numbers as the Python implementation (python -c "from remote_control import Trajectory; ...")
        Check("matches python samples", Math.Abs(tr.Sample(1.5)[0] - 2.1875) < 1e-9 && Math.Abs(tr.Sample(0.5)[0] - 0.3125) < 1e-9 && Math.Abs(tr.Sample(2.7)[0] - 2.216) < 1e-9);

        var joints = new System.Collections.Generic.Dictionary<string, Joint>();
        foreach (var j in TestJoints) joints[j.Name] = j;
        string Reject(string json)
        {
            try { GoalParser.Parse(JObject.Parse(json), joints); return null; }
            catch (GoalException e) { return e.Message; }
        }
        Check("rejects limit", Reject("{\"joint_names\":[\"shoulder\"],\"points\":[{\"positions\":[3.5],\"time_from_start\":1}]}")?.Contains("above upper limit") == true);
        Check("rejects speed", Reject("{\"joint_names\":[\"slide\"],\"points\":[{\"positions\":[0.1],\"time_from_start\":1},{\"positions\":[0.5],\"time_from_start\":1.1}]}")?.Contains("max_velocity") == true);
        Check("accepts valid", Reject("{\"joint_names\":[\"shoulder\",\"elbow\"],\"points\":[{\"positions\":[0.5,0.1],\"time_from_start\":1}],\"report\":\"all\"}") == null);

        var env = Envelope.FromJson("{\"v\":1,\"type\":\"pause\",\"robot_id\":\"r\",\"goal_id\":\"g1\",\"seq\":7,\"ts\":1.5,\"payload\":{}}");
        Check("envelope roundtrip", env.Type == "pause" && env.GoalId == "g1" && env.Seq == 7 && Envelope.FromJson(env.ToJson()).Seq == 7);
        bool threw = false;
        try { Envelope.FromJson("{\"v\":2,\"type\":\"x\",\"robot_id\":\"r\"}"); } catch (ProtocolException) { threw = true; }
        Check("rejects other protocol version", threw);

        // ── system config file (shared with Python) ──
        var ws = RemoteControlConfig.RobotTransportFromConfig(JObject.Parse(
            "{\"connector\":{\"type\":\"websocket\",\"host\":\"10.1.2.3\",\"port\":9100,\"path\":\"/m\",\"listen_host\":\"0.0.0.0\"}}"));
        Check("config websocket", ws is WebSocketRobotTransport && ws.Url == "ws://10.1.2.3:9100/m");
        Environment.SetEnvironmentVariable("RC_TEST_PW", "s3cret");
        var mq = RemoteControlConfig.RobotTransportFromConfig(JObject.Parse(
            "{\"connector\":{\"type\":\"mqtt\",\"host\":\"b\",\"tls\":true,\"prefix\":\"lab/rc\",\"username\":\"u\",\"password_env\":\"RC_TEST_PW\",\"keepalive\":30}}")
            is JObject j0 ? (JObject)RemoteControlConfig.ResolveEnv(j0) : null);
        Check("config mqtt", mq is MqttRobotTransport && mq.Url == "mqtts://u@b:8883/lab/rc");
        bool ros2Rejected = false;
        try { RemoteControlConfig.RobotTransportFromConfig(JObject.Parse("{\"connector\":{\"type\":\"ros2\"}}")); }
        catch (ConfigException e) { ros2Rejected = e.Message.Contains("not available in Unity"); }
        Check("config ros2 rejected in Unity", ros2Rejected);
        var saved = Environment.GetEnvironmentVariable(RemoteControlConfig.DirEnv);
        Environment.SetEnvironmentVariable(RemoteControlConfig.DirEnv, null);
        var userLevel = OperatingSystem.IsWindows() ? Environment.GetEnvironmentVariable(RemoteControlConfig.DirEnv, EnvironmentVariableTarget.User) : null;
        if (string.IsNullOrEmpty(userLevel))
        {
            bool envNamed = false;
            try { RemoteControlConfig.SystemConfigPath(); } catch (ConfigException e) { envNamed = e.Message.Contains("RC_CONFIG_DIR"); }
            Check("missing RC_CONFIG_DIR is reported", envNamed);
        }
        else   // process copy cleared, but setx stored it for the user: the fallback must find it
            Check("RC_CONFIG_DIR read from user settings when the process lacks it", RemoteControlConfig.ConfigDir() == userLevel);
        var dir = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "rc_cfg_" + Guid.NewGuid().ToString("N"));
        System.IO.Directory.CreateDirectory(dir);
        System.IO.File.WriteAllText(System.IO.Path.Combine(dir, "remote_control.json"),
            "{\"connector\":{\"type\":\"websocket\",\"host\":\"h\",\"port\":1234}}");
        Environment.SetEnvironmentVariable(RemoteControlConfig.DirEnv, dir);
        var sys = RemoteControlConfig.RobotTransportFromConfig(RemoteControlConfig.LoadSystemConfig());
        Check("system config from RC_CONFIG_DIR", sys.Url == "ws://h:1234/motion");
        Environment.SetEnvironmentVariable(RemoteControlConfig.DirEnv, saved);

        Console.WriteLine(fails == 0 ? "ALL PASS" : $"{fails} FAILED");
        return fails == 0 ? 0 : 1;
    }
}
