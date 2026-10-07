"""
Build the Windows release: compiled Python packages (Cython), the Unity demo robot compiled with IL2CPP, a private
Python, dependency wheels and the install scripts, in one folder and one zip that installs without internet access.

    python tools/build_release.py                # → dist/LogixPlan-RoboticToolbox-<version>-win64(.zip)
    python tools/build_release.py --test         # … also run the test suites against the compiled Python build
    python tools/build_release.py --no-python    # leave Python out (install.ps1 then downloads it)
    python tools/build_release.py --no-unity     # leave the Unity demo robot out (no Unity needed)

Steps:
  1. tools/build_cython.py          → build/cython (every module compiled to .pyd)
  2. a binary wheel of the compiled remote_control + robotic_toolbox packages (cp312-win_amd64)
  3. the dependency wheels (websockets, paho-mqtt, wxPython, numpy) at the versions in this venv
  4. Python itself: python.org's NuGet package (a complete, relocatable Python - no installer, no registry)
  5. the Unity demo robot: a Windows player built with IL2CPP (C# → C++ → native GameAssembly.dll, so neither the
     C# source nor .NET assemblies ship). Needs Unity's "Windows Build Support (IL2CPP)" module.
  6. installer/ scripts, examples and the docs (the Unity package as C# source only with --unity-source)
  7. the zip

The release runs only on the Python minor version it was built with (the .pyd files are tied to it), so the bundled
Python has this interpreter's exact version.
"""
from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
import urllib.request
import zipfile
from importlib import metadata
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent          # python/
REPO = ROOT.parent
APP = "LogixPlan-RoboticToolbox"
DEPENDENCIES = ["websockets", "paho-mqtt", "wxPython", "numpy"]
NUGET_URL = "https://www.nuget.org/api/v2/package/python/{version}"
UNITY_PROJECT = REPO / "unity"
UNITY_PACKAGE = UNITY_PROJECT / "Packages" / "com.logixplan.remote-control"
UNITY_PLAYER = UNITY_PROJECT / "Builds" / "RemoteControlDemo"
UNITY_HUB_EDITORS = Path(r"C:\Program Files\Unity\Hub\Editor")
# IL2CPP build output that must never ship: the generated C++ sources and debug information
NOT_SHIPPED = ("*_BackUpThisFolder_ButDontShipItWithYourGame", "*_BurstDebugInformation_DoNotShip", "*.pdb")

# setup.py written next to the compiled packages: a platform wheel holding the .pyd files (no sources)
WHEEL_SETUP = '''
from pathlib import Path
from setuptools import Distribution, find_packages, setup

class BinaryDistribution(Distribution):
    def has_ext_modules(self):          # tag the wheel cp312-cp312-win_amd64, not py3-none-any
        return True

packages = find_packages(include=["remote_control*", "robotic_toolbox*"])
# every compiled module and resource, listed per package (a catch-all "*.pyd" pattern is ignored by setuptools)
package_data = {{p: [f.name for f in Path(*p.split(".")).iterdir() if f.suffix in (".pyd", ".ico")]
                 for p in packages}}
package_data["robotic_toolbox"].append("resources/*.ico")

setup(name="remote-control", version="{version}",
      description="Remote Control motion protocol and the LogixPlan Robotic Toolbox (compiled)",
      packages=packages,
      package_data=package_data,
      python_requires="=={pyver}.*",
      extras_require={{"toolbox": {deps!r}}},
      distclass=BinaryDistribution, zip_safe=False)
'''


def run(*cmd, cwd=None) -> None:
    print("  $ " + " ".join(str(c) for c in cmd))
    subprocess.run([str(c) for c in cmd], cwd=cwd, check=True)


def project_version() -> str:
    m = re.search(r'^version\s*=\s*"([^"]+)"', (ROOT / "pyproject.toml").read_text(encoding="utf-8"), re.M)
    return m.group(1) if m else "0.0.0"


def pinned_dependencies() -> list:
    """The dependency versions installed here (the ones the tests ran with)."""
    out = []
    for name in DEPENDENCIES:
        try:
            out.append(f"{name}=={metadata.version(name)}")
        except metadata.PackageNotFoundError:
            raise SystemExit(f"{name} is not installed in this environment: pip install -e .[toolbox]")
    return out


def build_wheel(cython_dir: Path, wheels: Path, version: str, deps: list) -> Path:
    pyver = f"{sys.version_info.major}.{sys.version_info.minor}"
    setup_py = cython_dir / "setup.py"
    setup_py.write_text(WHEEL_SETUP.format(version=version, pyver=pyver, deps=deps), encoding="utf-8")
    # the copied pyproject.toml's [tool.setuptools] settings would override setup.py's package data
    pyproject = cython_dir / "pyproject.toml"
    aside = cython_dir / "pyproject.toml.aside"
    if pyproject.exists():
        pyproject.replace(aside)
    try:
        run(sys.executable, "-m", "pip", "wheel", ".", "--no-deps", "--no-build-isolation", "-q",
            "-w", wheels, cwd=cython_dir)
    finally:
        setup_py.unlink(missing_ok=True)
        if aside.exists():
            aside.replace(pyproject)
        for junk in ("build", "remote_control.egg-info"):
            shutil.rmtree(cython_dir / junk, ignore_errors=True)
    built = sorted(wheels.glob("remote_control-*.whl"))
    if not built:
        raise SystemExit("the wheel was not built")
    with zipfile.ZipFile(built[-1]) as z:
        names = z.namelist()
    sources = [n for n in names if n.endswith(".py") and not n.endswith(("__init__.py", "__main__.py"))]
    if sources:                                    # only compiled code may ship
        raise SystemExit("the wheel contains Python sources: " + ", ".join(sources))
    expected = {p.relative_to(cython_dir).as_posix() for p in cython_dir.glob("*/**/*.pyd")}
    missing = sorted(expected - set(names))
    if missing:
        raise SystemExit("the wheel lacks compiled modules: " + ", ".join(missing))
    return built[-1]


def download_python(dest: Path) -> Path:
    version = "{}.{}.{}".format(*sys.version_info[:3])
    target = dest / f"python.{version}.nupkg"
    print(f"  downloading Python {version} (NuGet) …")
    with urllib.request.urlopen(NUGET_URL.format(version=version), timeout=120) as r, open(target, "wb") as f:
        shutil.copyfileobj(r, f)
    return target


def unity_editor(explicit: str = "") -> Path:
    """Unity.exe of the version the project uses (ProjectSettings/ProjectVersion.txt), as installed by Unity Hub."""
    if explicit:
        return Path(explicit)
    text = (UNITY_PROJECT / "ProjectSettings" / "ProjectVersion.txt").read_text(encoding="utf-8")
    version = re.search(r"m_EditorVersion:\s*(\S+)", text).group(1)
    return UNITY_HUB_EDITORS / version / "Editor" / "Unity.exe"


def build_unity_player(editor: Path) -> None:
    """Batch-build the demo robot with IL2CPP (DemoArmBuilder.BuildDemoPlayerBatch → PlayerBuilder.Build)."""
    if not editor.is_file():
        raise SystemExit(f"Unity editor not found: {editor} (pass --unity PATH, or --no-unity)")
    variations = editor.parent / "Data" / "PlaybackEngines" / "windowsstandalonesupport" / "Variations"
    if not any(variations.glob("win64_*_il2cpp")):
        raise SystemExit("Unity's IL2CPP module is not installed: Unity Hub > Installs > "
                         f"{editor.parent.parent.name} > Manage > Add modules > Windows Build Support (IL2CPP)")
    lock = UNITY_PROJECT / "Temp" / "UnityLockfile"
    if lock.exists():
        try:
            lock.unlink()                       # a stale lock can be deleted; an open editor keeps it locked
        except OSError:
            raise SystemExit(f"close the Unity editor that has {UNITY_PROJECT} open")
    shutil.rmtree(UNITY_PLAYER, ignore_errors=True)
    log = UNITY_PROJECT / "Logs" / "release-build.log"
    print(f"  building with {editor} (log: {log}) …")
    r = subprocess.run([str(editor), "-batchmode", "-quit", "-projectPath", str(UNITY_PROJECT), "-logFile", str(log),
                        "-executeMethod", "RobotMarket.RemoteControl.Editor.DemoArmBuilder.BuildDemoPlayerBatch"])
    if r.returncode or not (UNITY_PLAYER / "GameAssembly.dll").is_file():
        tail = log.read_text(encoding="utf-8", errors="replace").splitlines()[-40:] if log.exists() else []
        raise SystemExit("Unity IL2CPP build failed:\n" + "\n".join(tail))
    if any(UNITY_PLAYER.glob("*_Data/Managed/*.dll")):
        raise SystemExit("the player contains .NET assemblies - it was not built with IL2CPP")


def copy_unity_package(dest: Path) -> None:
    shutil.copytree(UNITY_PACKAGE, dest, ignore=shutil.ignore_patterns("bin", "obj", "Tests~", "*.csproj.user"))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", default=str(ROOT / "dist"), help="output folder (default: python/dist)")
    ap.add_argument("--test", action="store_true", help="run the tests against the compiled Python build")
    ap.add_argument("--no-python", action="store_true", help="do not bundle Python (the installer downloads it)")
    ap.add_argument("--skip-cython", action="store_true", help="reuse an existing build/cython")
    ap.add_argument("--no-unity", action="store_true", help="leave the Unity demo robot out")
    ap.add_argument("--skip-unity-build", action="store_true", help="reuse an existing Unity player build")
    ap.add_argument("--unity", default="", help="path of Unity.exe (default: the project's version under Unity Hub)")
    ap.add_argument("--unity-source", action="store_true", help="also ship the Unity package as C# source")
    args = ap.parse_args()
    if sys.platform != "win32" or sys.maxsize < 2 ** 32:
        raise SystemExit("build the Windows release with 64-bit Python on Windows")

    version = project_version()
    deps = pinned_dependencies()
    name = f"{APP}-{version}-win64"
    out = Path(args.out).resolve()
    stage = out / name
    if stage.exists():
        shutil.rmtree(stage)
    (stage / "wheels").mkdir(parents=True)

    # The Unity build goes first: it is the step most likely to fail (missing module, editor open).
    print("1. Unity demo robot (IL2CPP)")
    if args.no_unity:
        print("   left out (--no-unity)")
    else:
        if not args.skip_unity_build:
            build_unity_player(unity_editor(args.unity))
        shutil.copytree(UNITY_PLAYER, stage / "robot", ignore=shutil.ignore_patterns(*NOT_SHIPPED))
        print(f"   {stage / 'robot'}")

    cython_dir = ROOT / "build" / "cython"
    print("2. compiling Python with Cython")
    if not args.skip_cython:
        run(sys.executable, ROOT / "tools" / "build_cython.py", *(["--test"] if args.test else []))
    print("3. wheel of the compiled packages")
    wheel = build_wheel(cython_dir, stage / "wheels", version, deps)
    print("   " + wheel.name)
    print("4. dependency wheels: " + ", ".join(deps))
    run(sys.executable, "-m", "pip", "download", "-q", "--only-binary=:all:", "--platform", "win_amd64",
        "--python-version", f"{sys.version_info.major}.{sys.version_info.minor}", "-d", stage / "wheels", *deps)
    print("5. Python")
    if args.no_python:
        print("   not bundled: install.ps1 downloads it")
    else:
        (stage / "python").mkdir()
        download_python(stage / "python")
    print("6. installer, examples, docs")
    for f in (ROOT / "installer").iterdir():
        shutil.copy2(f, stage / f.name)
    shutil.copytree(ROOT / "examples", stage / "examples",
                    ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    shutil.copy2(ROOT / "tools" / "mini_mqtt_broker.py", stage / "examples" / "mini_mqtt_broker.py")
    if args.unity_source:
        copy_unity_package(stage / "unity" / UNITY_PACKAGE.name)
    for doc in ("README.md", "PROTOCOL.md"):
        shutil.copy2(REPO / doc, stage / doc)
    manifest = {"name": APP, "version": version, "wheel": wheel.name,
                "python": "{}.{}.{}".format(*sys.version_info[:3]), "dependencies": deps,
                "robot": None if args.no_unity else "robot/RemoteControlDemo.exe"}
    (stage / "release.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    print("7. zip")
    archive = shutil.make_archive(str(out / name), "zip", root_dir=out, base_dir=name)
    size = Path(archive).stat().st_size / 1e6
    print(f"\nrelease: {stage}\nzip:     {archive} ({size:.0f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
