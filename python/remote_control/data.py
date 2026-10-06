"""
Controller-side data files, one directory per robot:

    <data_dir>/<project>/<stage>/<robot>/
        description.json    last `describe` reply: names, base location, kinematic tree, joint positions
        poses.json          saved joint poses: {"poses": {"<name>": {"positions": {"<joint>": rad|m}, "saved_at"}}}

`data_dir` comes from the system config (`"data_dir"` in remote_control.json; a relative path is relative to
the config file's directory). project / stage / robot are the names the robot reports (Unity: project folder,
scene, robot_id). Characters that are not allowed in file names are replaced by "_".
"""
from __future__ import annotations

import json
import os
import re
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Union

from .connectors.config import ConfigError

DEFAULT_NAME = "default"
_BAD = re.compile(r'[<>:"/\\|?*\x00-\x1f]')


def safe_name(name: Optional[str]) -> str:
    """A name usable as one directory level on Windows / Linux / macOS."""
    s = _BAD.sub("_", (name or "").strip())
    if not s:
        return DEFAULT_NAME
    return s.rstrip(". ") or "_"   # Windows drops trailing dots / spaces; "." and ".." are not names


def data_dir_from_config(config: Mapping[str, Any], config_path: Optional[Union[str, Path]] = None) -> Path:
    """The `data_dir` of a whole-file config; relative paths are taken from the config file's directory."""
    value = config.get("data_dir")
    if not value:
        raise ConfigError('config has no "data_dir" - add e.g. "data_dir": "D:\\\\Dev\\\\remote_control\\\\data"')
    p = Path(os.path.expandvars(os.path.expanduser(str(value))))
    if not p.is_absolute() and config_path is not None:
        p = Path(config_path).resolve().parent / p
    return p.resolve()


def _write_json(path: Path, data: Any) -> None:
    """Write via a temp file + rename, so a crash never leaves half a file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
            f.write("\n")
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


class PoseNotFound(KeyError):
    pass


class RobotStore:
    """Files of one robot: <data_dir>/<project>/<stage>/<robot>/."""

    DESCRIPTION = "description.json"
    POSES = "poses.json"

    def __init__(self, data_dir: Union[str, Path], project: Optional[str], stage: Optional[str], robot: str) -> None:
        self.project = project or DEFAULT_NAME
        self.stage = stage or DEFAULT_NAME
        self.robot = robot
        self.dir = Path(data_dir) / safe_name(self.project) / safe_name(self.stage) / safe_name(robot)

    def __repr__(self) -> str:
        return f"RobotStore({self.dir})"

    @property
    def names(self) -> Dict[str, str]:
        return {"project": self.project, "stage": self.stage, "robot": self.robot}

    # ── description ──────────────────────────────────────────────────────────

    def save_description(self, description: Mapping[str, Any]) -> Path:
        path = self.dir / self.DESCRIPTION
        _write_json(path, {"saved_at": _now(), **description})
        return path

    def load_description(self) -> Optional[Dict[str, Any]]:
        path = self.dir / self.DESCRIPTION
        return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else None

    # ── poses ────────────────────────────────────────────────────────────────

    def _load_poses(self) -> Dict[str, Any]:
        path = self.dir / self.POSES
        if not path.is_file():
            return {}
        data = json.loads(path.read_text(encoding="utf-8"))
        return dict(data.get("poses", {}))

    def _save_poses(self, poses: Mapping[str, Any]) -> Path:
        path = self.dir / self.POSES
        _write_json(path, {"version": 1, **self.names, "poses": dict(sorted(poses.items()))})
        return path

    def list_poses(self) -> List[str]:
        return sorted(self._load_poses())

    def get_pose(self, name: str) -> Dict[str, Any]:
        poses = self._load_poses()
        if name not in poses:
            raise PoseNotFound(f"no pose '{name}' for {self.project}/{self.stage}/{self.robot}"
                               + (f" (saved: {', '.join(sorted(poses))})" if poses else " (none saved)"))
        return poses[name]

    def save_pose(self, name: str, joint_names: Sequence[str], positions: Sequence[float]) -> Path:
        if not name or not name.strip():
            raise ValueError("pose name must not be empty")
        if len(joint_names) != len(positions):
            raise ValueError("joint_names and positions differ in length")
        poses = self._load_poses()
        poses[name.strip()] = {"positions": {n: float(x) for n, x in zip(joint_names, positions)},
                               "saved_at": _now()}
        return self._save_poses(poses)

    def delete_pose(self, name: str) -> None:
        poses = self._load_poses()
        if name not in poses:
            raise PoseNotFound(f"no pose '{name}' for {self.project}/{self.stage}/{self.robot}")
        del poses[name]
        self._save_poses(poses)


def _now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S%z")
