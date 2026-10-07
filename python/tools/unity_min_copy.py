"""
Make a minimal, self-contained copy of a Unity project: only what the given scenes need.

    python tools/unity_min_copy.py "D:\\Data\\unity\\project1\\My project" Assets/Scenes/robot0625.unity
    python tools/unity_min_copy.py <project> <scene> [<scene> …] --out D:\\Temp\\MyProjectMin --zip

What the copy holds:
  - the scenes and every asset they reference, followed through GUIDs in text assets (prefabs, materials,
    ScriptableObjects …), each with its .meta (GUIDs must not change) and the .meta of every folder above it
  - the assets the project settings reference (render pipeline, input actions …)
  - every C# script / assembly definition / plugin DLL in Assets: scripts can depend on each other without GUIDs
  - ProjectSettings, with the build scene list reduced to the copied scenes
  - Packages/manifest.json; local "file:" packages are copied into Packages/ (embedded), so the copy does not need
    the original paths, and git packages are embedded from Library/PackageCache, so it does not need git.
    packages-lock.json is left out: Unity writes a new one.
Left out: Library (Unity rebuilds it on first open), Temp, Logs, UserSettings, Builds, and unreferenced assets.
"""
from __future__ import annotations

import argparse
import json
import re
import shutil
import sys
from pathlib import Path

GUID = re.compile(rb"guid: ([0-9a-f]{32})")
# Unity YAML assets: they can reference other assets. Everything else (meshes, textures, audio …) ends the walk.
TEXT_ASSETS = {".unity", ".prefab", ".mat", ".asset", ".controller", ".overrideController", ".anim",
               ".physicMaterial", ".physicsMaterial2D", ".mask", ".playable", ".lighting", ".renderTexture",
               ".shadergraph", ".shadersubgraph", ".terrainlayer", ".spriteatlas", ".spriteatlasv2", ".mixer",
               ".flare", ".guiskin", ".fontsettings", ".cubemap", ".brush", ".preset", ".inputactions", ".vfx",
               ".signal", ".giparams", ".uxml", ".uss", ".tss"}
CODE = {".cs", ".asmdef", ".asmref", ".dll", ".rsp", ".jslib", ".cginc", ".hlsl", ".shader", ".compute"}
SKIP_PACKAGE_DIRS = {"bin", "obj", "Tests~", ".git", "Library"}


def build_guid_index(assets: Path) -> dict:
    index = {}
    for meta in assets.rglob("*.meta"):
        m = re.search(rb"^guid: ([0-9a-f]{32})", meta.read_bytes()[:512], re.M)
        if m:
            index[m.group(1)] = meta.with_suffix("")
    return index


def settings_roots(project: Path) -> set:
    """GUIDs referenced by ProjectSettings, except the build scene list (it may name scenes we leave out)."""
    guids = set()
    for f in (project / "ProjectSettings").glob("*.asset"):
        data = f.read_bytes()
        if f.name == "EditorBuildSettings.asset":
            data = re.sub(rb"m_Scenes:.*?(?=\n  m_|\Z)", b"", data, flags=re.S)
        guids |= set(GUID.findall(data))
    return guids


def dependencies(project: Path, scenes: list) -> set:
    index = build_guid_index(project / "Assets")
    todo = [project / s for s in scenes] + [index[g] for g in settings_roots(project) if g in index]
    needed = set()
    while todo:
        f = todo.pop()
        if f in needed or not f.exists():
            continue
        needed.add(f)
        if f.is_dir():                       # a referenced folder (e.g. a default folder setting): keep it, not its contents
            continue
        if f.suffix.lower() in TEXT_ASSETS or f.suffix in TEXT_ASSETS:
            for g in set(GUID.findall(f.read_bytes())):
                p = index.get(g)
                if p is not None and p not in needed:
                    todo.append(p)
    needed |= {f for f in (project / "Assets").rglob("*") if f.is_file() and f.suffix.lower() in CODE}
    return needed


def copy_with_meta(project: Path, out: Path, f: Path) -> None:
    rel = f.relative_to(project)
    dst = out / rel
    if f.is_dir():
        dst.mkdir(parents=True, exist_ok=True)
    else:
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(f, dst)
    meta = f.with_name(f.name + ".meta")
    if meta.exists():
        shutil.copy2(meta, out / rel.parent / meta.name)
    parent = f.parent                        # folder .meta files keep the folders' GUIDs
    while parent != project / "Assets" and parent != project:
        pmeta = parent.with_name(parent.name + ".meta")
        if pmeta.exists() and not (out / pmeta.relative_to(project)).exists():
            (out / pmeta.relative_to(project)).parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(pmeta, out / pmeta.relative_to(project))
        parent = parent.parent


def copy_project_settings(project: Path, out: Path, scenes: list) -> None:
    shutil.copytree(project / "ProjectSettings", out / "ProjectSettings")
    ebs = out / "ProjectSettings" / "EditorBuildSettings.asset"
    if ebs.exists():
        text = ebs.read_text(encoding="utf-8")
        keep = {s.replace("\\", "/") for s in scenes}
        entries = re.findall(r"  - enabled: \d+\n    path: (.*)\n    guid: .*\n", text)
        for path in entries:
            if path not in keep:
                text = re.sub(r"  - enabled: \d+\n    path: " + re.escape(path) + r"\n    guid: .*\n", "", text)
        if "  m_Scenes:\n  m_" in text or text.rstrip().endswith("m_Scenes:"):
            text = text.replace("  m_Scenes:\n", "  m_Scenes: []\n", 1)
        ebs.write_text(text, encoding="utf-8")


def copy_packages(project: Path, out: Path) -> list:
    """Copy manifest.json; embed local file: packages. Returns the embedded package names."""
    (out / "Packages").mkdir(parents=True, exist_ok=True)
    manifest = json.loads((project / "Packages" / "manifest.json").read_text(encoding="utf-8"))
    embedded = []
    for name, version in list(manifest.get("dependencies", {}).items()):
        if isinstance(version, str) and version.startswith("file:"):
            src = Path(version[5:])
            if not src.is_absolute():
                src = (project / "Packages" / src).resolve()
            if src.suffix == ".tgz":
                shutil.copy2(src, out / "Packages" / src.name)
                manifest["dependencies"][name] = f"file:{src.name}"
                continue
            shutil.copytree(src, out / "Packages" / name, ignore=shutil.ignore_patterns(*SKIP_PACKAGE_DIRS))
            del manifest["dependencies"][name]     # an embedded package (Packages/<name>) is found automatically
            embedded.append(name)
        elif isinstance(version, str) and (".git" in version or version.startswith("git")):
            # Git packages need git on the other PC; embed the copy Unity already fetched into Library/PackageCache.
            cached = sorted((project / "Library" / "PackageCache").glob(name + "@*"))
            if not cached:
                print(f"  warning: {name} comes from git and is not in Library/PackageCache - the other PC needs git")
                continue
            shutil.copytree(cached[-1], out / "Packages" / name, ignore=shutil.ignore_patterns(*SKIP_PACKAGE_DIRS))
            del manifest["dependencies"][name]
            embedded.append(name)
    for d in (project / "Packages").iterdir():     # packages already embedded in the project
        if d.is_dir() and not (out / "Packages" / d.name).exists():
            shutil.copytree(d, out / "Packages" / d.name, ignore=shutil.ignore_patterns(*SKIP_PACKAGE_DIRS))
            embedded.append(d.name)
    (out / "Packages" / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return embedded


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("project", help="Unity project folder")
    ap.add_argument("scenes", nargs="+", help="scenes to keep, relative to the project (Assets/…/X.unity)")
    ap.add_argument("--out", default=None, help="output folder (default: <project> (minimal) next to the project)")
    ap.add_argument("--zip", action="store_true", help="also make <out>.zip")
    args = ap.parse_args()

    project = Path(args.project).resolve()
    scenes = [s.replace("\\", "/") for s in args.scenes]
    for s in scenes:
        if not (project / s).is_file():
            raise SystemExit(f"no scene {project / s}")
    out = Path(args.out).resolve() if args.out else project.with_name(project.name + " (minimal)")
    if out.exists():
        raise SystemExit(f"{out} exists - delete it or pick another --out")
    # Keep the project folder's name: the Remote Control robot reports it as its project name (the data folder).
    out = out / project.name if args.out is None else out

    files = dependencies(project, scenes)
    for f in sorted(files):
        copy_with_meta(project, out, f)
    copy_project_settings(project, out, scenes)
    embedded = copy_packages(project, out)

    size = sum(f.stat().st_size for f in out.rglob("*") if f.is_file())
    print(f"{out}\n  {len([f for f in files if f.is_file()])} assets + metas, {size / 1e6:.0f} MB")
    print(f"  scenes: {', '.join(scenes)}")
    if embedded:
        print(f"  embedded packages: {', '.join(embedded)}")
    if args.zip:
        archive = shutil.make_archive(str(out), "zip", root_dir=out.parent, base_dir=out.name)
        print(f"  zip: {archive} ({Path(archive).stat().st_size / 1e6:.0f} MB)")
    print("Open it in Unity (same editor version); the first open rebuilds Library and takes a while.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
