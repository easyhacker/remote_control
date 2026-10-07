// Ctrl+click on an object in the Scene view creates a Remote Control target on its surface (Edit and Play mode).
// In Edit mode it is an ordinary scene object (Undo works, it is saved with the scene).

using UnityEditor;
using UnityEngine;
using RobotMarket.RemoteControl.Unity;

namespace RobotMarket.RemoteControl.Editor
{
    [InitializeOnLoad]
    static class TargetPlacement
    {
        static TargetPlacement()
        {
            SceneView.duringSceneGui += OnSceneGui;
        }

        static void OnSceneGui(SceneView view)
        {
            var e = Event.current;
            if (e.type != EventType.MouseDown || e.button != 0 || !(e.control || e.command) || e.alt) return;
            var ray = HandleUtility.GUIPointToWorldRay(e.mousePosition);
            if (!RemoteControlTarget.Pick(ray, out var point, out var normal, out var hitObject)) return;

            var robot = Object.FindAnyObjectByType<RemoteControlRobot>();
            bool attach = robot != null ? robot.attachNewTargets : RemoteControlTarget.AttachNewTargets;
            var rotation = robot != null
                ? robot.ClickTargetRotation(point, normal, view.camera.transform.forward)
                : RemoteControlTarget.Orientation(RemoteControlTarget.NewTargetOrientation, point, normal,
                                                  view.camera.transform.forward, null, null);
            bool hadRoot = RemoteControlTarget.Root(hitObject.gameObject.scene, false) != null;
            var target = RemoteControlTarget.CreateAt(point, rotation, hitObject, attach);

            if (!EditorApplication.isPlaying)
            {
                if (!hadRoot && !attach)
                    Undo.RegisterCreatedObjectUndo(target.transform.parent.gameObject, "Create target");
                Undo.RegisterCreatedObjectUndo(target.gameObject, "Create target");
                UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(target.gameObject.scene);
            }
            Selection.activeGameObject = target.gameObject;
            Debug.Log($"[RemoteControl] new target '{RemoteControlTarget.PathOf(target.transform)}' on {hitObject.name}");
            e.Use();
        }
    }
}
