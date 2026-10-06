"""
Kinematic trees: read one from a URDF, and compute link poses (forward kinematics) from joint positions.

The tree format is the one robots send in a `description` message (see PROTOCOL.md):

    {"root": "base_link",
     "links":  [{"name": "base_link"}, ...],
     "joints": [{"name": "elbow_joint", "type": "revolute", "parent": "upper_arm", "child": "forearm",
                 "origin": {"xyz": [0, 0, 0.3], "rpy": [0, 0, 0]}, "axis": [0, 1, 0],
                 "lower": -2.4, "upper": 2.4, "max_velocity": 2.0, "command_name": "elbow_joint"}, ...]}

Frames follow ROS / URDF: x forward, y left, z up, metres; poses are {"position": [x, y, z],
"orientation": [qx, qy, qz, qw]}. No numpy needed.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

Vec = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]   # x, y, z, w

IDENTITY_POSE: Dict[str, List[float]] = {"position": [0.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]}
FRAME = "ros: x forward, y left, z up; metres; orientation quaternion [x, y, z, w]"


# ── quaternion helpers ───────────────────────────────────────────────────────

def quat_mul(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_rotate(q: Quat, v: Sequence[float]) -> Vec:
    x, y, z, w = q
    vx, vy, vz = v
    # v + 2w(q×v) + 2 q×(q×v)
    cx, cy, cz = y * vz - z * vy, z * vx - x * vz, x * vy - y * vx
    ccx, ccy, ccz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return (vx + 2 * (w * cx + ccx), vy + 2 * (w * cy + ccy), vz + 2 * (w * cz + ccz))


def quat_from_axis_angle(axis: Sequence[float], angle: float) -> Quat:
    n = math.sqrt(sum(a * a for a in axis)) or 1.0
    s = math.sin(angle / 2) / n
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2))


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quat:
    """URDF rpy: fixed-axis roll about x, then pitch about y, then yaw about z (R = Rz·Ry·Rx)."""
    qx = quat_from_axis_angle((1, 0, 0), roll)
    qy = quat_from_axis_angle((0, 1, 0), pitch)
    qz = quat_from_axis_angle((0, 0, 1), yaw)
    return quat_mul(qz, quat_mul(qy, qx))


def pose_mul(a: Mapping[str, Sequence[float]], b: Mapping[str, Sequence[float]]) -> Dict[str, List[float]]:
    """a ∘ b: pose b expressed in a's frame, mapped to a's parent frame."""
    qa = tuple(a["orientation"])
    p = quat_rotate(qa, b["position"])  # type: ignore[arg-type]
    q = quat_mul(qa, tuple(b["orientation"]))  # type: ignore[arg-type]
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    return {"position": [a["position"][i] + p[i] for i in range(3)], "orientation": [c / n for c in q]}


def origin_pose(origin: Mapping[str, Sequence[float]]) -> Dict[str, List[float]]:
    xyz = origin.get("xyz", (0.0, 0.0, 0.0))
    rpy = origin.get("rpy", (0.0, 0.0, 0.0))
    return {"position": [float(c) for c in xyz], "orientation": list(quat_from_rpy(*map(float, rpy)))}


# ── URDF ─────────────────────────────────────────────────────────────────────

def _floats(text: Optional[str], default: Sequence[float]) -> List[float]:
    return [float(t) for t in text.split()] if text else list(default)


def load_urdf_tree(path: str) -> Dict[str, Any]:
    """Links and joints of a URDF in the description `tree` format (joint names are the URDF joint names)."""
    root = ET.parse(path).getroot()
    links = [{"name": l.get("name")} for l in root.findall("link")]
    joints: List[Dict[str, Any]] = []
    children = set()
    for j in root.findall("joint"):
        jtype = j.get("type") or "fixed"
        origin = j.find("origin")
        axis = j.find("axis")
        lim = j.find("limit")
        child = j.find("child").get("link")  # type: ignore[union-attr]
        children.add(child)
        entry: Dict[str, Any] = {
            "name": j.get("name"),
            "type": jtype,
            "parent": j.find("parent").get("link"),  # type: ignore[union-attr]
            "child": child,
            "origin": {"xyz": _floats(origin.get("xyz") if origin is not None else None, (0, 0, 0)),
                       "rpy": _floats(origin.get("rpy") if origin is not None else None, (0, 0, 0))},
        }
        if jtype != "fixed":
            entry["axis"] = _floats(axis.get("xyz") if axis is not None else None, (1, 0, 0))
            lower = upper = vel = None
            if lim is not None:
                if jtype not in ("continuous", "floating", "planar"):
                    lower = float(lim.get("lower", 0.0))
                    upper = float(lim.get("upper", 0.0))
                if lim.get("velocity") is not None and float(lim.get("velocity")) > 0:
                    vel = float(lim.get("velocity"))
            entry.update(lower=lower, upper=upper, max_velocity=vel,
                         command_name=j.get("name") if jtype in ("revolute", "prismatic", "continuous") else None)
        else:
            entry["command_name"] = None
        joints.append(entry)
    roots = [l["name"] for l in links if l["name"] not in children]
    return {"robot": root.get("name"), "root": roots[0] if roots else None, "links": links, "joints": joints}


def forward_kinematics(tree: Mapping[str, Any], positions: Mapping[str, float],
                       base_pose: Optional[Mapping[str, Sequence[float]]] = None) -> Dict[str, Dict[str, List[float]]]:
    """Pose of every link (in the frame of base_pose, default: the root link's frame).

    `positions` is keyed by the joints' `command_name` (missing joints are at 0)."""
    by_parent: Dict[str, List[Mapping[str, Any]]] = {}
    for j in tree["joints"]:
        by_parent.setdefault(j["parent"], []).append(j)
    poses: Dict[str, Dict[str, List[float]]] = {}
    root = tree["root"]
    poses[root] = {"position": list(map(float, (base_pose or IDENTITY_POSE)["position"])),
                   "orientation": list(map(float, (base_pose or IDENTITY_POSE)["orientation"]))}
    stack = [root]
    while stack:
        link = stack.pop()
        for j in by_parent.get(link, []):
            pose = pose_mul(poses[link], origin_pose(j["origin"]))
            q = float(positions.get(j.get("command_name") or "", 0.0) or 0.0)
            if j["type"] in ("revolute", "continuous"):
                pose = pose_mul(pose, {"position": [0, 0, 0],
                                       "orientation": list(quat_from_axis_angle(j["axis"], q))})
            elif j["type"] == "prismatic":
                a = j["axis"]
                n = math.sqrt(sum(c * c for c in a)) or 1.0
                pose = pose_mul(pose, {"position": [a[0] / n * q, a[1] / n * q, a[2] / n * q],
                                       "orientation": [0, 0, 0, 1]})
            poses[j["child"]] = pose
            stack.append(j["child"])
    return poses
