"""
Cythonize the Python packages into native extension modules (.pyd on Windows, .so on Linux / macOS).

    python tools/build_cython.py                 # → build/cython/  (remote_control + robotic_toolbox compiled)
    python tools/build_cython.py --test          # … then run the test suites against the compiled build
    python tools/build_cython.py --keep-c        # keep the generated .c files (debugging)

The output mirrors this folder: the packages contain only compiled modules plus their __init__.py / __main__.py
(package markers and entry points stay Python). tests/, examples/ and tools/ are copied as source so the compiled
build can be tested and run the same way:

    cd build/cython
    python -m robotic_toolbox

Needs Cython and a C compiler: Visual Studio Build Tools ("Desktop development with C++") on Windows, gcc / clang
elsewhere. Build with the same Python version (and bitness) the app will run on.
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import sysconfig
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent          # python/
PACKAGES = ["remote_control", "robotic_toolbox"]
COPY_AS_SOURCE = ["tests", "examples", "tools", "pyproject.toml"]
KEEP_PY = {"__init__.py", "__main__.py"}               # package markers / entry points stay Python

DIRECTIVES = {
    "language_level": "3",
    "binding": True,             # functions keep Python introspection (wx / asyncio callbacks, signatures)
    "annotation_typing": False,  # treat annotations as documentation: no C typing, no exact-type checks
    "embedsignature": True,
}


def copy_tree(src: Path, dst: Path) -> None:
    shutil.copytree(src, dst, ignore=shutil.ignore_patterns("__pycache__", "*.pyc", "*.pyd", "*.so", "*.c",
                                                             "build", ".venv*"))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", default=str(ROOT / "build" / "cython"), help="output folder (default: build/cython)")
    ap.add_argument("--packages", nargs="+", default=PACKAGES)
    ap.add_argument("--keep-c", action="store_true", help="keep generated C sources")
    ap.add_argument("--test", action="store_true", help="run the test suites against the compiled build")
    ap.add_argument("-j", "--jobs", type=int, default=os.cpu_count() or 1)
    args = ap.parse_args()

    try:
        from Cython.Build import cythonize
        from setuptools import Extension, setup
    except ImportError:
        print("needs Cython and setuptools:  pip install cython setuptools")
        return 2

    out = Path(args.out).resolve()
    # Empty the folder rather than delete it: Windows cannot delete a folder that is some process's current
    # directory (e.g. a terminal left in build/cython after running the app from there).
    out.mkdir(parents=True, exist_ok=True)
    for child in out.iterdir():
        if child.is_dir():
            shutil.rmtree(child)
        else:
            child.unlink()
    for pkg in args.packages:
        copy_tree(ROOT / pkg, out / pkg)
    for name in COPY_AS_SOURCE:
        src = ROOT / name
        if src.is_dir():
            copy_tree(src, out / name)
        elif src.exists():
            shutil.copy2(src, out / name)

    modules = []
    for pkg in args.packages:
        for py in sorted((out / pkg).rglob("*.py")):
            if py.name in KEEP_PY:
                continue
            dotted = ".".join(py.relative_to(out).with_suffix("").parts)
            modules.append(Extension(dotted, [str(py.relative_to(out))]))
    print(f"cythonizing {len(modules)} modules from {', '.join(args.packages)} -> {out}")

    cwd = os.getcwd()
    os.chdir(out)
    try:
        try:
            ext = cythonize(modules, compiler_directives=DIRECTIVES, nthreads=args.jobs, quiet=True)
        except Exception as exc:   # the parallel pool occasionally dies on Windows; serial always works
            print(f"parallel cythonize failed ({type(exc).__name__}) - retrying serially")
            ext = cythonize(modules, compiler_directives=DIRECTIVES, nthreads=0, quiet=True)
        setup(name="rc_cython_build", ext_modules=ext,
              script_args=["-q", "build_ext", "--inplace", f"--parallel={args.jobs}"])
    finally:
        os.chdir(cwd)

    # leave only the compiled modules (+ __init__ / __main__) in the packages
    suffix = sysconfig.get_config_var("EXT_SUFFIX") or (".pyd" if os.name == "nt" else ".so")
    missing = []
    for m in modules:
        py = out / m.sources[0]
        compiled = py.with_name(py.stem + suffix)
        if not compiled.exists():
            missing.append(m.name)
            continue
        py.unlink()
        if not args.keep_c:
            py.with_suffix(".c").unlink(missing_ok=True)
    shutil.rmtree(out / "build", ignore_errors=True)
    if missing:
        print("NOT compiled: " + ", ".join(missing))
        return 1
    print(f"compiled {len(modules)} modules ({suffix})")

    if args.test:
        check = ("import remote_control.controller as c, robotic_toolbox.ik as i; "
                 "assert not c.__file__.endswith('.py') and not i.__file__.endswith('.py'), (c.__file__, i.__file__); "
                 "print('using compiled modules:', c.__file__, i.__file__)")
        env = dict(os.environ, PYTHONPATH=str(out), RC_REPO_ROOT=str(ROOT.parent))
        r = subprocess.run([sys.executable, "-c", check], cwd=out, env=env)
        if r.returncode:
            return r.returncode
        r = subprocess.run([sys.executable, "-m", "unittest", "discover", "-s", "tests"], cwd=out, env=env)
        return r.returncode
    return 0


if __name__ == "__main__":
    sys.exit(main())
