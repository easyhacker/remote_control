// Windows player builds for releases, with IL2CPP by default: the C# (this package included) is converted to C++ and
// compiled into a native GameAssembly.dll, so no .NET assemblies ship with the player.
//
// Batch mode (any project that uses this package):
//   Unity.exe -batchmode -quit -projectPath <project> -logFile build.log
//       -executeMethod RobotMarket.RemoteControl.Editor.PlayerBuilder.BuildBatch
//       -buildPath Builds/MyRobot/MyRobot.exe          (default: Builds/<product name>/<product name>.exe)
//       -scene Assets/Scenes/A.unity -scene …          (default: the enabled scenes in Build Settings)
//       -backend il2cpp | mono                         (default: il2cpp)
// IL2CPP needs the "Windows Build Support (IL2CPP)" module in Unity Hub and Visual Studio's C++ tools.

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEngine;

namespace RobotMarket.RemoteControl.Editor
{
    public static class PlayerBuilder
    {
        /// <summary>Batch entry point: reads -buildPath / -scene / -backend from the command line and exits with
        /// 0 on success, 1 on failure.</summary>
        public static void BuildBatch()
        {
            var scenes = Args("-scene");
            if (scenes.Count == 0)
                scenes = EditorBuildSettings.scenes.Where(s => s.enabled).Select(s => s.path).ToList();
            string product = PlayerSettings.productName;
            string path = Args("-buildPath").FirstOrDefault() ?? $"Builds/{product}/{product}.exe";
            var backend = ParseBackend(Args("-backend").FirstOrDefault());
            var result = Build(scenes.ToArray(), path, backend);
            EditorApplication.Exit(result == BuildResult.Succeeded ? 0 : 1);
        }

        /// <summary>Build a Windows x64 player. The project's scripting backend is set for the build and restored
        /// afterwards, so a release build does not change the project for Play mode or other builds.</summary>
        public static BuildResult Build(string[] scenes, string path, ScriptingImplementation backend)
        {
            if (scenes.Length == 0)
            {
                Debug.LogError("[RemoteControl] build: no scenes (pass -scene or enable scenes in Build Settings)");
                return BuildResult.Failed;
            }
            var target = NamedBuildTarget.Standalone;
            var previousBackend = PlayerSettings.GetScriptingBackend(target);
            bool previousRunInBackground = PlayerSettings.runInBackground;
            try
            {
                PlayerSettings.SetScriptingBackend(target, backend);
                PlayerSettings.runInBackground = true;      // keep serving the controller when not focused
                Debug.Log($"[RemoteControl] building {path} ({backend}) from {string.Join(", ", scenes)}");
                var report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
                {
                    scenes = scenes,
                    locationPathName = path,
                    target = BuildTarget.StandaloneWindows64,
                    options = BuildOptions.None,
                });
                var summary = report.summary;
                Debug.Log($"[RemoteControl] build: {summary.result}, {summary.totalErrors} error(s), " +
                          $"{summary.totalSize / 1e6:F0} MB, {summary.totalTime.TotalSeconds:F0} s");
                if (summary.result == BuildResult.Succeeded && backend == ScriptingImplementation.IL2CPP &&
                    !File.Exists(Path.Combine(Path.GetDirectoryName(path) ?? ".", "GameAssembly.dll")))
                {
                    Debug.LogError("[RemoteControl] build: IL2CPP was requested but GameAssembly.dll is missing");
                    return BuildResult.Failed;
                }
                return summary.result;
            }
            finally
            {
                PlayerSettings.SetScriptingBackend(target, previousBackend);
                PlayerSettings.runInBackground = previousRunInBackground;
            }
        }

        static ScriptingImplementation ParseBackend(string value)
        {
            switch ((value ?? "il2cpp").ToLowerInvariant())
            {
                case "il2cpp": return ScriptingImplementation.IL2CPP;
                case "mono": return ScriptingImplementation.Mono2x;
                default: throw new ArgumentException($"-backend must be il2cpp or mono, not '{value}'");
            }
        }

        /// <summary>Every value that follows `name` on the command line (an option may be repeated).</summary>
        static List<string> Args(string name)
        {
            var args = Environment.GetCommandLineArgs();
            var values = new List<string>();
            for (int i = 0; i < args.Length - 1; i++)
                if (string.Equals(args[i], name, StringComparison.OrdinalIgnoreCase)) values.Add(args[i + 1]);
            return values;
        }
    }
}
