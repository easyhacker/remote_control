// Menu: IVI Dynamic → Robotic Toolbox → Open Robotic Toolbox.
// Starts the LogixPlan Robotic Toolbox named in the "toolbox" section of %RC_CONFIG_DIR%\remote_control.json:
//
//     "toolbox": { "command": "C:\\...\\RoboticToolbox\\python\\pythonw.exe",
//                  "args": "-m robotic_toolbox",
//                  "working_dir": "C:\\...\\RoboticToolbox" }          (working_dir is optional)
//
// The installer writes this section. The Toolbox gets this Editor's RC_CONFIG_DIR, so both use the same config, and it
// is single-instance per config: if it already runs, the running window comes to the front instead.

using System.Diagnostics;
using System.IO;
using Newtonsoft.Json.Linq;
using UnityEditor;

namespace RobotMarket.RemoteControl.Editor
{
    public static class RoboticToolboxMenu
    {
        const string Title = "Robotic Toolbox";

        [MenuItem("IVI Dynamic/Robotic Toolbox/Open Robotic Toolbox", priority = 0)]
        public static void Open()
        {
            JObject config;
            string configPath;
            try
            {
                configPath = RemoteControlConfig.SystemConfigPath();
                config = RemoteControlConfig.Load(configPath);
            }
            catch (ConfigException e)
            {
                EditorUtility.DisplayDialog(Title, "Cannot read the Remote Control config:\n\n" + e.Message, "OK");
                return;
            }

            var toolbox = config["toolbox"] as JObject;
            string command = toolbox?["command"]?.Value<string>();
            if (string.IsNullOrEmpty(command))
            {
                EditorUtility.DisplayDialog(Title,
                    $"{configPath} has no \"toolbox\" section saying where the Robotic Toolbox is.\n\n" +
                    "Install the Toolbox (install.cmd adds it), or add:\n\n" +
                    "\"toolbox\": { \"command\": \"<install folder>\\\\python\\\\pythonw.exe\", " +
                    "\"args\": \"-m robotic_toolbox\" }", "OK");
                return;
            }
            if (!File.Exists(command))
            {
                EditorUtility.DisplayDialog(Title, $"The Robotic Toolbox was not found:\n\n{command}\n\n" +
                    $"(from \"toolbox\" in {configPath}). Reinstall it, or correct the path.", "OK");
                return;
            }

            var start = new ProcessStartInfo(command, toolbox["args"]?.Value<string>() ?? "")
            {
                WorkingDirectory = toolbox["working_dir"]?.Value<string>() ?? Path.GetDirectoryName(command),
                UseShellExecute = false,      // needed to pass the environment below
            };
            // The Editor may have found RC_CONFIG_DIR only in the user settings (see RemoteControlConfig.ConfigDir):
            // hand it on explicitly so the Toolbox reads the same config as this Editor's robots.
            start.EnvironmentVariables[RemoteControlConfig.DirEnv] = RemoteControlConfig.ConfigDir();
            try
            {
                Process.Start(start);
            }
            catch (System.Exception e)
            {
                EditorUtility.DisplayDialog(Title, $"Could not start the Robotic Toolbox:\n\n{e.Message}", "OK");
            }
        }
    }
}
