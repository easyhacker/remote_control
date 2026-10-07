// Menu: IVI Dynamic → Remote Control → Debug: Dump Renderers of Selection
// Writes what Unity actually has loaded for every renderer under the selected object (active state,
// enabled, world bounds, mesh + vertex count, material/shader, layer, scene-visibility) to
// Logs/renderer_dump.txt — for diagnosing parts that exist in the Hierarchy but don't show up.

using System.Globalization;
using System.IO;
using System.Text;
using UnityEditor;
using UnityEngine;

namespace RobotMarket.RemoteControl.Editor
{
    public static class RendererDump
    {
        [MenuItem("IVI Dynamic/Remote Control/Debug: Dump Renderers of Selection")]
        static void Dump()
        {
            var root = Selection.activeGameObject;
            if (root == null)
            {
                EditorUtility.DisplayDialog("Dump Renderers", "Select a robot in the Hierarchy first.", "OK");
                return;
            }
            var ci = CultureInfo.InvariantCulture;
            var sb = new StringBuilder();
            sb.AppendLine($"root: {Path(root.transform)}  layer={LayerMask.LayerToName(root.layer)}");
            var cam = SceneView.lastActiveSceneView != null ? SceneView.lastActiveSceneView.camera : null;
            if (cam != null)
                sb.AppendLine($"scene camera: pos={cam.transform.position} near={cam.nearClipPlane} far={cam.farClipPlane} cullingMask={cam.cullingMask}");
            sb.AppendLine($"Tools.visibleLayers={Tools.visibleLayers}");
            foreach (var r in root.GetComponentsInChildren<Renderer>(true))
            {
                var go = r.gameObject;
                var mf = go.GetComponent<MeshFilter>();
                var mesh = mf != null ? mf.sharedMesh : null;
                var mat = r.sharedMaterial;
                var b = r.bounds;
                sb.AppendLine(string.Format(ci,
                    "{0}\n   activeInHierarchy={1} enabled={2} hiddenInScene={3} layer={4} forceOff={5} shadowsOnly={6}\n" +
                    "   bounds center=({7:0.###},{8:0.###},{9:0.###}) size=({10:0.###},{11:0.###},{12:0.###})\n" +
                    "   mesh={13} verts={14} subMeshes={15} readable={16}  material={17} shader={18} lossyScale={19}",
                    Path(go.transform), go.activeInHierarchy, r.enabled, SceneVisibilityManager.instance.IsHidden(go),
                    LayerMask.LayerToName(go.layer), r.forceRenderingOff, r.shadowCastingMode == UnityEngine.Rendering.ShadowCastingMode.ShadowsOnly,
                    b.center.x, b.center.y, b.center.z, b.size.x, b.size.y, b.size.z,
                    mesh != null ? mesh.name : "NONE", mesh != null ? mesh.vertexCount : -1, mesh != null ? mesh.subMeshCount : -1,
                    mesh != null && mesh.isReadable, mat != null ? mat.name : "NONE", mat != null && mat.shader != null ? mat.shader.name : "NONE",
                    go.transform.lossyScale));
            }
            Directory.CreateDirectory("Logs");
            File.WriteAllText("Logs/renderer_dump.txt", sb.ToString());
            Debug.Log($"[RemoteControl] renderer dump of '{root.name}' written to Logs/renderer_dump.txt");
            EditorUtility.RevealInFinder("Logs/renderer_dump.txt");
        }

        static string Path(Transform t)
        {
            var s = t.name;
            while (t.parent != null) { t = t.parent; s = t.name + "/" + s; }
            return s;
        }
    }
}
