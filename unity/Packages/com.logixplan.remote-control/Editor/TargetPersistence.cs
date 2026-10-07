// Targets created, moved or deleted in Play mode (Ctrl+click, the Move tool, the controller) would vanish when Play
// stops, like any Play-mode change. On leaving Play mode this offers to keep them: the changes are applied to the
// scene (with Undo) and the scene is marked dirty.

using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using RobotMarket.RemoteControl.Unity;

namespace RobotMarket.RemoteControl.Editor
{
    [InitializeOnLoad]
    static class TargetPersistence
    {
        const string SnapshotKey = "RemoteControl.Targets.Snapshot";
        const string PendingKey = "RemoteControl.Targets.Pending";

        [Serializable] class Entry
        {
            public string path, parent, name;
            public Vector3 position;
            public Quaternion rotation;
            public float size;
        }

        [Serializable] class Snapshot { public List<Entry> targets = new List<Entry>(); }

        [Serializable] class Pending
        {
            public List<Entry> upsert = new List<Entry>();
            public List<string> delete = new List<string>();
        }

        static TargetPersistence()
        {
            EditorApplication.playModeStateChanged += OnPlayModeChanged;
        }

        static Snapshot Capture()
        {
            var s = new Snapshot();
            foreach (var t in RemoteControlTarget.All)
            {
                if (t == null) continue;
                var tr = t.transform;
                s.targets.Add(new Entry
                {
                    path = RemoteControlTarget.PathOf(tr), name = tr.name,
                    parent = tr.parent != null ? RemoteControlTarget.PathOf(tr.parent) : "",
                    position = tr.localPosition, rotation = tr.localRotation, size = t.size,
                });
            }
            return s;
        }

        static void OnPlayModeChanged(PlayModeStateChange change)
        {
            if (change == PlayModeStateChange.EnteredPlayMode)
            {
                SessionState.SetString(SnapshotKey, JsonUtility.ToJson(Capture()));
                SessionState.EraseString(PendingKey);
            }
            else if (change == PlayModeStateChange.ExitingPlayMode)
            {
                var before = JsonUtility.FromJson<Snapshot>(SessionState.GetString(SnapshotKey, "{}")) ?? new Snapshot();
                var now = Capture();
                var old = before.targets.ToDictionary(e => e.path, e => e);
                var pending = new Pending();
                foreach (var e in now.targets)
                {
                    if (!old.TryGetValue(e.path, out var o) || o.parent != e.parent
                        || Vector3.Distance(o.position, e.position) > 1e-5f || Quaternion.Angle(o.rotation, e.rotation) > 0.01f)
                        pending.upsert.Add(e);
                }
                var current = new HashSet<string>(now.targets.Select(e => e.path));
                pending.delete.AddRange(before.targets.Where(e => !current.Contains(e.path)).Select(e => e.path));
                SessionState.SetString(PendingKey, JsonUtility.ToJson(pending));
            }
            else if (change == PlayModeStateChange.EnteredEditMode)
            {
                var json = SessionState.GetString(PendingKey, "");
                SessionState.EraseString(PendingKey);
                if (string.IsNullOrEmpty(json)) return;
                var pending = JsonUtility.FromJson<Pending>(json);
                if (pending == null || pending.upsert.Count + pending.delete.Count == 0) return;
                var names = pending.upsert.Select(e => e.name).Concat(pending.delete.Select(p => p.Split('/').Last())).ToList();
                if (!EditorUtility.DisplayDialog("Remote Control targets",
                        $"Keep the target changes made in Play mode?\n\n{pending.upsert.Count} created or moved, " +
                        $"{pending.delete.Count} deleted:\n{string.Join(", ", names.Take(12))}{(names.Count > 12 ? ", …" : "")}",
                        "Keep", "Discard"))
                    return;
                Apply(pending);
            }
        }

        static Transform FindByPath(string path)
        {
            if (string.IsNullOrEmpty(path)) return null;
            foreach (var t in UnityEngine.Object.FindObjectsByType<Transform>(FindObjectsInactive.Include, FindObjectsSortMode.None))
                if (RemoteControlTarget.PathOf(t) == path) return t;
            return null;
        }

        static void Apply(Pending pending)
        {
            Undo.SetCurrentGroupName("Keep Play-mode targets");
            foreach (var path in pending.delete)
            {
                var t = FindByPath(path);
                if (t != null && t.GetComponent<RemoteControlTarget>() != null) Undo.DestroyObjectImmediate(t.gameObject);
            }
            var scene = EditorSceneManager.GetActiveScene();
            foreach (var e in pending.upsert)
            {
                var parent = FindByPath(e.parent);
                if (parent == null)
                {
                    bool hadRoot = RemoteControlTarget.Root(scene, false) != null;
                    parent = RemoteControlTarget.Root(scene, true);
                    if (!hadRoot) Undo.RegisterCreatedObjectUndo(parent.gameObject, "Create Targets");
                }
                var t = FindByPath(e.path);
                if (t == null)
                {
                    var go = new GameObject(e.name);
                    Undo.RegisterCreatedObjectUndo(go, "Create target");
                    t = go.transform;
                }
                Undo.SetTransformParent(t, parent, "Move target");
                Undo.RecordObject(t, "Move target");
                t.localPosition = e.position;
                t.localRotation = e.rotation;
                var target = t.GetComponent<RemoteControlTarget>() ?? Undo.AddComponent<RemoteControlTarget>(t.gameObject);
                target.size = e.size;
                EditorSceneManager.MarkSceneDirty(t.gameObject.scene);
            }
            Debug.Log($"[RemoteControl] kept {pending.upsert.Count} created / moved and {pending.delete.Count} deleted " +
                      "target(s) from Play mode - save the scene to keep them");
        }
    }
}
