// Selecting a Remote Control marker (a frame drawn for the controller) or a target in the Hierarchy or Scene view
// picks it as the target in the controller, like clicking it in the Game view.

using UnityEditor;
using UnityEngine;
using RobotMarket.RemoteControl.Unity;

namespace RobotMarket.RemoteControl.Editor
{
    [InitializeOnLoad]
    static class MarkerSelection
    {
        static MarkerSelection()
        {
            Selection.selectionChanged += OnSelectionChanged;
        }

        static void OnSelectionChanged()
        {
            if (!EditorApplication.isPlaying) return;
            var go = Selection.activeGameObject;
            if (go == null) return;
            var marker = go.GetComponentInParent<RemoteControlMarker>();
            if (marker != null && marker.selectable && marker.owner != null)
            {
                marker.owner.SelectMarker(marker.itemId, "editor");
                return;
            }
            var target = go.GetComponent<RemoteControlTarget>();
            if (target == null) return;
            foreach (var t in RemoteControlTarget.All) if (t != null) t.SetHighlighted(t == target);
            foreach (var robot in Object.FindObjectsByType<RemoteControlRobot>(FindObjectsSortMode.None))
                robot.Session?.Select(target.Id, "editor");
        }
    }
}
