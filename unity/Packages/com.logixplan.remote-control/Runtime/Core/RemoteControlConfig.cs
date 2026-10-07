// The ONE config file shared by the controller and every robot (Python, ROS 2 bridge, Unity):
//
//     %RC_CONFIG_DIR%\remote_control.json
//
// Mirrors python/remote_control/connectors/config.py. Its "connector" section holds the communication type and
// parameters; Unity robots support "websocket" and "mqtt". Secrets: "<key>_env": "VAR" reads environment
// variable VAR into <key>, and "${VAR}" is expanded inside strings.

using System;
using System.IO;
using System.Text.RegularExpressions;
using Newtonsoft.Json.Linq;

namespace RobotMarket.RemoteControl
{
    public class ConfigException : Exception
    {
        public ConfigException(string message) : base(message) { }
    }

    public static class RemoteControlConfig
    {
        public const string DirEnv = "RC_CONFIG_DIR";
        public const string FileName = "remote_control.json";
        static readonly Regex Var = new Regex(@"\$\{([A-Za-z_][A-Za-z0-9_]*)\}");

        /// <summary>%RC_CONFIG_DIR%\remote_control.json; throws ConfigException naming what is missing.</summary>
        public static string SystemConfigPath()
        {
            var dir = ConfigDir();
            if (string.IsNullOrEmpty(dir))
                throw new ConfigException($"environment variable {DirEnv} is not set - set it to the directory containing {FileName} " +
                                          @"(e.g. setx RC_CONFIG_DIR D:\path\to\config)");
            var path = Path.Combine(dir, FileName);
            if (!File.Exists(path))
                throw new ConfigException($"config file not found: {path} ({DirEnv}={dir})");
            return path;
        }

        /// <summary>
        /// RC_CONFIG_DIR from this process; on Windows also from the user / machine settings, because a variable set
        /// with `setx` reaches only programs started afterwards — the Unity Editor inherits Unity Hub's environment,
        /// and the Hub usually keeps running in the tray for days.
        /// </summary>
        public static string ConfigDir()
        {
            var dir = Environment.GetEnvironmentVariable(DirEnv);
            if (!string.IsNullOrEmpty(dir)) return dir;
            try
            {
                dir = Environment.GetEnvironmentVariable(DirEnv, EnvironmentVariableTarget.User);
                if (string.IsNullOrEmpty(dir))
                    dir = Environment.GetEnvironmentVariable(DirEnv, EnvironmentVariableTarget.Machine);
            }
            catch (Exception) { dir = null; }   // targets other than Process are Windows-only
            return dir;
        }

        public static JObject LoadSystemConfig() => Load(SystemConfigPath());

        public static JObject Load(string path)
        {
            JObject o;
            try { o = JObject.Parse(File.ReadAllText(path)); }
            catch (Exception e) when (!(e is ConfigException)) { throw new ConfigException($"{path}: {e.Message}"); }
            return (JObject)ResolveEnv(o);
        }

        public static JToken ResolveEnv(JToken token)
        {
            switch (token)
            {
                case JObject obj:
                    var result = new JObject();
                    foreach (var prop in obj.Properties())
                    {
                        if (prop.Name.EndsWith("_env") && prop.Value.Type == JTokenType.String)
                        {
                            var name = prop.Value.Value<string>();
                            var value = Environment.GetEnvironmentVariable(name);
                            if (value == null)
                                throw new ConfigException($"environment variable {name} (for '{prop.Name.Substring(0, prop.Name.Length - 4)}') is not set");
                            result[prop.Name.Substring(0, prop.Name.Length - 4)] = value;
                        }
                        else result[prop.Name] = ResolveEnv(prop.Value);
                    }
                    return result;
                case JArray arr:
                    var a = new JArray();
                    foreach (var item in arr) a.Add(ResolveEnv(item));
                    return a;
                case JValue v when v.Type == JTokenType.String:
                    return new JValue(Var.Replace(v.Value<string>(), m =>
                        Environment.GetEnvironmentVariable(m.Groups[1].Value)
                        ?? throw new ConfigException($"environment variable {m.Groups[1].Value} is not set")));
                default:
                    return token.DeepClone();
            }
        }

        static string Str(JObject c, string key, string fallback = null)
        {
            var t = c[key];
            return t == null || t.Type == JTokenType.Null ? fallback : t.Value<string>();
        }

        static double Num(JObject c, string key, double fallback)
        {
            var t = c[key];
            return t == null || t.Type == JTokenType.Null ? fallback : t.Value<double>();
        }

        static bool Flag(JObject c, string key) => c[key] != null && c[key].Type == JTokenType.Boolean && c[key].Value<bool>();

        /// <summary>Robot-side transport from a whole config or its "connector" section.</summary>
        public static IRobotTransport RobotTransportFromConfig(JObject config)
        {
            var c = config["connector"] as JObject ?? config;
            var url = Str(c, "url");
            var type = (Str(c, "type") ?? (url != null ? new Uri(url).Scheme : null) ?? "").ToLowerInvariant();
            switch (type)
            {
                case "websocket": case "ws": case "wss":
                {
                    url = url ?? $"{(Flag(c, "tls") ? "wss" : "ws")}://{Str(c, "host", "localhost")}:{(int)Num(c, "port", 8765)}{Str(c, "path", "/motion")}";
                    return new WebSocketRobotTransport(url) { MinBackoff = Num(c, "min_backoff", 0.5), MaxBackoff = Num(c, "max_backoff", 5.0) };
                }
                case "mqtt": case "mqtts":
                {
                    var s = url != null ? MqttSettings.FromUrl(url) : new MqttSettings
                    {
                        Host = Str(c, "host", "localhost"),
                        Port = (int)Num(c, "port", 0),
                        Prefix = Str(c, "prefix", "rc"),
                        Username = Str(c, "username"),
                        Password = Str(c, "password"),
                        Tls = Flag(c, "tls"),
                    };
                    s.KeepAlive = (int)Num(c, "keepalive", s.KeepAlive);
                    s.ClientId = Str(c, "client_id", s.ClientId);
                    return new MqttRobotTransport(s);
                }
                case "ros2": case "ros":
                    throw new ConfigException("connector type 'ros2' is not available in Unity - use 'websocket' or 'mqtt' " +
                                              "(ROS 2 robots and controllers can share the system through a websocket/mqtt controller)");
                case "":
                    throw new ConfigException("connector config needs a 'type' (websocket, mqtt) or a 'url'");
                default:
                    throw new ConfigException($"unknown connector type '{type}' (Unity supports: websocket, mqtt)");
            }
        }

        /// <summary>One-line description without secrets, for logs and the overlay.</summary>
        public static string Describe(JObject config)
        {
            var c = config["connector"] as JObject ?? config;
            var url = Str(c, "url");
            if (url != null) return url;
            var parts = new System.Collections.Generic.List<string>();
            foreach (var p in c.Properties())
                if (p.Name != "type" && p.Name != "password" && p.Value.Type != JTokenType.Null)
                    parts.Add($"{p.Name}={p.Value}");
            return $"{Str(c, "type")} ({string.Join(", ", parts)})";
        }
    }
}
