// Menu: RobotMarket → Remote Control → Create Demo Arm / Create Demo Scene.
// Builds a 5-joint arm (4 revolute + 1 prismatic gripper) from primitives with ArticulationBody joints and
// a RemoteControlRobot component, ready to connect to examples/controller_demo.py.
// Batch mode: Unity -batchmode -projectPath <proj> -executeMethod RobotMarket.RemoteControl.Editor.DemoArmBuilder.CreateDemoSceneBatch -quit

using RobotMarket.RemoteControl.Unity;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace RobotMarket.RemoteControl.Editor
{
    public static class DemoArmBuilder
    {
        const string ScenePath = "Assets/Scenes/RemoteControlDemo.unity";

        [MenuItem("RobotMarket/Remote Control/Create Demo Arm")]
        public static void CreateDemoArmMenu()
        {
            var arm = CreateDemoArm();
            Undo.RegisterCreatedObjectUndo(arm, "Create Demo Arm");
            Selection.activeGameObject = arm;
        }

        [MenuItem("RobotMarket/Remote Control/Create Demo Scene")]
        public static void CreateDemoSceneMenu()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            CreateDemoScene();
        }

        public static void CreateDemoSceneBatch()
        {
            CreateDemoScene();
            EditorApplication.Exit(0);
        }

        /// <summary>Batch: build a Windows player of the demo scene to Builds/RemoteControlDemo/.
        /// Run it with: RemoteControlDemo.exe -batchmode -nographics -controllerUrl ws://host:8765/motion</summary>
        public static void BuildDemoPlayerBatch()
        {
            if (!System.IO.File.Exists(ScenePath)) CreateDemoScene();
            PlayerSettings.runInBackground = true;
            var report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
            {
                scenes = new[] { ScenePath },
                locationPathName = "Builds/RemoteControlDemo/RemoteControlDemo.exe",
                target = BuildTarget.StandaloneWindows64,
                options = BuildOptions.None,
            });
            Debug.Log("[RemoteControl] build: " + report.summary.result);
            EditorApplication.Exit(report.summary.result == UnityEditor.Build.Reporting.BuildResult.Succeeded ? 0 : 1);
        }

        static void CreateDemoScene()
        {
            var scene = EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);
            var cam = Camera.main;
            if (cam != null)
            {
                cam.transform.position = new Vector3(1.4f, 1.1f, -1.6f);
                cam.transform.LookAt(new Vector3(0, 0.55f, 0));
                cam.backgroundColor = new Color(0.12f, 0.13f, 0.15f);
            }
            var ground = GameObject.CreatePrimitive(PrimitiveType.Plane);
            ground.name = "Ground";
            ground.transform.localScale = new Vector3(0.4f, 1, 0.4f);
            Tint(ground, new Color(0.25f, 0.26f, 0.28f));

            CreateDemoArm();

            System.IO.Directory.CreateDirectory("Assets/Scenes");
            EditorSceneManager.SaveScene(scene, ScenePath);
            AddToBuildSettings(ScenePath);
            Debug.Log("[RemoteControl] demo scene saved to " + ScenePath);
        }

        public static GameObject CreateDemoArm()
        {
            var root = new GameObject("DemoArm");
            var rootBody = root.AddComponent<ArticulationBody>();
            rootBody.immovable = true;
            rootBody.useGravity = false;
            Visual(root.transform, PrimitiveType.Cylinder, new Vector3(0, 0.025f, 0), new Vector3(0.36f, 0.025f, 0.36f), new Color(0.2f, 0.2f, 0.22f));

            // name, parent, local offset, axis (Y = yaw/twist, Z = pitch), limits (deg)
            var yaw = Revolute("shoulder_yaw", root.transform, new Vector3(0, 0.05f, 0), Vector3.up, -170, 170);
            Visual(yaw.transform, PrimitiveType.Cylinder, new Vector3(0, 0.05f, 0), new Vector3(0.16f, 0.05f, 0.16f), new Color(0.95f, 0.55f, 0.1f));

            var pitch = Revolute("shoulder_pitch", yaw.transform, new Vector3(0, 0.1f, 0), Vector3.forward, -100, 100);
            Visual(pitch.transform, PrimitiveType.Cube, new Vector3(0, 0.25f, 0), new Vector3(0.08f, 0.5f, 0.08f), new Color(0.85f, 0.85f, 0.88f));

            var elbow = Revolute("elbow", pitch.transform, new Vector3(0, 0.5f, 0), Vector3.forward, -140, 140);
            Visual(elbow.transform, PrimitiveType.Sphere, Vector3.zero, Vector3.one * 0.1f, new Color(0.95f, 0.55f, 0.1f));
            Visual(elbow.transform, PrimitiveType.Cube, new Vector3(0, 0.2f, 0), new Vector3(0.07f, 0.4f, 0.07f), new Color(0.85f, 0.85f, 0.88f));

            var wrist = Revolute("wrist", elbow.transform, new Vector3(0, 0.4f, 0), Vector3.up, -180, 180);
            Visual(wrist.transform, PrimitiveType.Cube, new Vector3(0, 0.03f, 0), new Vector3(0.14f, 0.04f, 0.06f), new Color(0.3f, 0.3f, 0.32f));

            var grip = Prismatic("gripper", wrist.transform, new Vector3(0, 0.05f, 0), 0f, 0.05f);
            Visual(grip.transform, PrimitiveType.Cube, new Vector3(0.0f, 0.05f, 0), new Vector3(0.02f, 0.1f, 0.05f), new Color(0.2f, 0.6f, 1f));

            var rc = root.AddComponent<RemoteControlRobot>();
            rc.articulationRoot = rootBody;
            rc.robotId = "unity-arm";
            rc.displayName = "Unity demo arm";
            rc.defaultMaxVelocity = 2.0f;
            rc.jointOverrides.Add(new JointOverride { bodyName = "gripper", maxVelocity = 0.1f });
            return root;
        }

        static ArticulationBody Revolute(string name, Transform parent, Vector3 offset, Vector3 axis, float lowerDeg, float upperDeg)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            go.transform.localPosition = offset;
            var b = go.AddComponent<ArticulationBody>();
            b.jointType = ArticulationJointType.RevoluteJoint;
            b.useGravity = false;
            b.mass = 1f;
            // An articulation joint turns about the anchor's local X axis — rotate the anchor onto `axis`
            b.anchorRotation = Quaternion.FromToRotation(Vector3.right, axis);
            b.twistLock = ArticulationDofLock.LimitedMotion;
            var d = b.xDrive;
            d.lowerLimit = lowerDeg;
            d.upperLimit = upperDeg;
            d.stiffness = 5000f;
            d.damping = 400f;
            d.forceLimit = float.MaxValue;
            b.xDrive = d;
            return b;
        }

        static ArticulationBody Prismatic(string name, Transform parent, Vector3 offset, float lower, float upper)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            go.transform.localPosition = offset;
            var b = go.AddComponent<ArticulationBody>();
            b.jointType = ArticulationJointType.PrismaticJoint;
            b.useGravity = false;
            b.mass = 0.2f;
            b.linearLockX = ArticulationDofLock.LimitedMotion;
            var d = b.xDrive;
            d.lowerLimit = lower;
            d.upperLimit = upper;
            d.stiffness = 20000f;
            d.damping = 500f;
            d.forceLimit = float.MaxValue;
            b.xDrive = d;
            return b;
        }

        static void Visual(Transform parent, PrimitiveType type, Vector3 pos, Vector3 scale, Color color)
        {
            var v = GameObject.CreatePrimitive(type);
            v.name = "visual";
            Object.DestroyImmediate(v.GetComponent<Collider>());   // visuals only — avoid self-collision
            v.transform.SetParent(parent, false);
            v.transform.localPosition = pos;
            v.transform.localScale = scale;
            Tint(v, color);
        }

        static void Tint(GameObject go, Color color)
        {
            var r = go.GetComponent<Renderer>();
            if (r == null) return;
            var shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
            var m = new Material(shader) { color = color };
            if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", color);
            System.IO.Directory.CreateDirectory("Assets/RemoteControlDemo/Materials");
            var path = AssetDatabase.GenerateUniqueAssetPath($"Assets/RemoteControlDemo/Materials/{go.name}.mat");
            AssetDatabase.CreateAsset(m, path);
            r.sharedMaterial = m;
        }

        static void AddToBuildSettings(string path)
        {
            var scenes = new System.Collections.Generic.List<EditorBuildSettingsScene>(EditorBuildSettings.scenes);
            if (scenes.Exists(s => s.path == path)) return;
            scenes.Add(new EditorBuildSettingsScene(path, true));
            EditorBuildSettings.scenes = scenes.ToArray();
        }
    }
}
