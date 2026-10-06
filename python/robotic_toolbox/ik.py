"""
Inverse kinematics on a robot's kinematic tree (the `joints` of a `description`, see PROTOCOL.md).

    chain = Chain.from_tree(desc, tool="R_claw", base="base")
    pose = chain.tool_pose(positions)                       # 4x4, tool frame in the base frame
    result = chain.solve(target_4x4, positions)             # damped least squares within the joint limits
    if result.reachable: robot.move_to(result.positions)

Frames follow ROS / URDF (x forward, y left, z up, metres, radians). Only the joints on the path from `base` to
`tool` move; `base` must be an ancestor of `tool`.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

MOVABLE = ("revolute", "continuous", "prismatic")


# ── small transform helpers ──────────────────────────────────────────────────

def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF rpy: R = Rz(yaw) · Ry(pitch) · Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def matrix_rpy(r: np.ndarray) -> Tuple[float, float, float]:
    """Inverse of rpy_matrix; at pitch ±90° the whole roll/yaw rotation goes into roll."""
    cp = math.hypot(r[0, 0], r[1, 0])
    pitch = math.atan2(-r[2, 0], cp)
    if cp > 1e-9:
        return math.atan2(r[2, 1], r[2, 2]), pitch, math.atan2(r[1, 0], r[0, 0])
    return math.atan2(-r[1, 2], r[1, 1]), pitch, 0.0


def axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return np.array([[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                     [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                     [t * x * z - s * y, t * y * z + s * x, t * z * z + c]])


def transform(xyz: Sequence[float] = (0, 0, 0), rot: Optional[np.ndarray] = None) -> np.ndarray:
    t = np.eye(4)
    t[:3, 3] = xyz
    if rot is not None:
        t[:3, :3] = rot
    return t


def pose_from_xyz_rpy(xyz: Sequence[float], rpy: Sequence[float]) -> np.ndarray:
    return transform(xyz, rpy_matrix(*rpy))


def xyz_rpy_from_pose(t: np.ndarray) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    return (float(t[0, 3]), float(t[1, 3]), float(t[2, 3])), matrix_rpy(t[:3, :3])


def rotation_error(r_target: np.ndarray, r: np.ndarray) -> np.ndarray:
    """Rotation vector (axis · angle) that turns r into r_target, in the base frame."""
    d = r_target @ r.T
    angle = math.acos(max(-1.0, min(1.0, (np.trace(d) - 1) / 2)))
    if angle < 1e-9:
        return np.zeros(3)
    v = np.array([d[2, 1] - d[1, 2], d[0, 2] - d[2, 0], d[1, 0] - d[0, 1]])
    s = math.sin(angle)
    if abs(s) < 1e-6:   # ~180°: axis from the symmetric part
        m = (d + np.eye(3)) / 2
        axis = m[:, int(np.argmax(np.diag(m)))]
        return axis / np.linalg.norm(axis) * angle
    return v / (2 * s) * angle


# ── chain ────────────────────────────────────────────────────────────────────

@dataclass
class ChainJoint:
    name: str                 # URDF joint name
    command_name: Optional[str]
    type: str
    origin: np.ndarray        # 4x4, child frame at position 0 in the parent frame
    axis: np.ndarray
    lower: Optional[float]
    upper: Optional[float]

    @property
    def movable(self) -> bool:
        return self.type in MOVABLE and bool(self.command_name)

    def motion(self, q: float) -> np.ndarray:
        if self.type in ("revolute", "continuous"):
            return transform(rot=axis_angle_matrix(self.axis, q))
        if self.type == "prismatic":
            return transform(self.axis * q)
        return np.eye(4)


@dataclass
class IkResult:
    reachable: bool
    positions: Dict[str, float]          # command_name → value, for the chain's movable joints
    position_error: float                # metres
    rotation_error: float                # radians (0 when position_only)
    iterations: int
    near_limits: List[str] = field(default_factory=list)
    message: str = ""


class Chain:
    def __init__(self, joints: List[ChainJoint], base: str, tool: str) -> None:
        self.joints = joints
        self.base = base
        self.tool = tool
        self.movable = [j for j in joints if j.movable]

    @property
    def joint_names(self) -> List[str]:
        """Command names of the joints IK moves, base to tool."""
        return [j.command_name for j in self.movable]  # type: ignore[misc]

    @staticmethod
    def links(tree: Mapping[str, Any]) -> List[str]:
        names = [l["name"] for l in tree.get("links", [])]
        for j in tree.get("joints", []):
            for k in ("parent", "child"):
                if j.get(k) and j[k] not in names:
                    names.append(j[k])
        return names

    @staticmethod
    def tool_candidates(tree: Mapping[str, Any]) -> List[str]:
        """Links worth offering as tools: the child of each movable joint that ends a chain of movable joints
        (e.g. a wrist / claw), plus leaves. Ordered as in the tree."""
        joints = [j for j in tree.get("joints", []) if j.get("parent") and j.get("child")]
        children: Dict[str, List[Mapping[str, Any]]] = {}
        for j in joints:
            children.setdefault(j["parent"], []).append(j)

        def rotates_below(link: str) -> bool:
            return any(j["type"] in ("revolute", "continuous") or rotates_below(j["child"])
                       for j in children.get(link, []))

        # the last rotating joint of each branch (gripper fingers below it may still slide)
        out = [j["child"] for j in joints
               if j["type"] in ("revolute", "continuous") and not rotates_below(j["child"])]
        return out or [j["child"] for j in joints if j["child"] not in children]

    @staticmethod
    def ancestors(tree: Mapping[str, Any], link: str) -> List[str]:
        """link, its parent, … up to the root."""
        parent = {j["child"]: j["parent"] for j in tree.get("joints", []) if j.get("parent") and j.get("child")}
        out = [link]
        while out[-1] in parent:
            out.append(parent[out[-1]])
        return out

    @classmethod
    def from_tree(cls, tree: Mapping[str, Any], tool: str, base: Optional[str] = None) -> "Chain":
        by_child = {j["child"]: j for j in tree.get("joints", []) if j.get("child")}
        path = cls.ancestors(tree, tool)
        base = base or tree.get("root") or path[-1]
        if base not in path:
            raise ValueError(f"'{base}' is not an ancestor of '{tool}' - pick a link on the path {' → '.join(reversed(path))}")
        joints: List[ChainJoint] = []
        link = tool
        while link != base:
            j = by_child[link]
            o = j.get("origin") or {}
            axis = np.array(j.get("axis") or (1.0, 0.0, 0.0), dtype=float)
            n = np.linalg.norm(axis)
            joints.append(ChainJoint(
                name=j.get("name") or link, command_name=j.get("command_name"), type=j.get("type", "fixed"),
                origin=pose_from_xyz_rpy(o.get("xyz", (0, 0, 0)), o.get("rpy", (0, 0, 0))),
                axis=axis / n if n > 0 else np.array([1.0, 0, 0]),
                lower=j.get("lower"), upper=j.get("upper")))
            link = j["parent"]
        joints.reverse()
        return cls(joints, base, tool)

    # ── kinematics ───────────────────────────────────────────────────────────

    def _q(self, positions: Mapping[str, float]) -> np.ndarray:
        return np.array([float(positions.get(j.command_name, 0.0) or 0.0) for j in self.movable])  # type: ignore[arg-type]

    def _frames(self, q: np.ndarray) -> Tuple[np.ndarray, List[Tuple[np.ndarray, np.ndarray]]]:
        """Tool pose, and per movable joint its axis and origin in the base frame."""
        t = np.eye(4)
        axes = []
        k = 0
        for j in self.joints:
            t = t @ j.origin
            if j.movable:
                axes.append((t[:3, :3] @ j.axis, t[:3, 3].copy()))
                t = t @ j.motion(q[k])
                k += 1
        return t, axes

    def tool_pose(self, positions: Mapping[str, float]) -> np.ndarray:
        return self._frames(self._q(positions))[0]

    def jacobian(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        t, axes = self._frames(q)
        jac = np.zeros((6, len(self.movable)))
        p_tool = t[:3, 3]
        for i, (j, (z, p)) in enumerate(zip(self.movable, axes)):
            if j.type == "prismatic":
                jac[:3, i] = z
            else:
                jac[:3, i] = np.cross(z, p_tool - p)
                jac[3:, i] = z
        return jac, t

    def _limits(self) -> Tuple[np.ndarray, np.ndarray]:
        lo = np.array([j.lower if j.lower is not None else -math.pi * 2 for j in self.movable], dtype=float)
        hi = np.array([j.upper if j.upper is not None else math.pi * 2 for j in self.movable], dtype=float)
        return lo, hi

    def solve(self, target: np.ndarray, start: Mapping[str, float], position_only: bool = False,
              tol_position: float = 1e-3, tol_rotation: float = math.radians(0.5),
              max_iterations: int = 300, restarts: int = 12, seed: int = 1) -> IkResult:
        """Damped least squares from the current positions, then from random starts within the limits.
        Returns the best solution found; `reachable` when it is within both tolerances."""
        if not self.movable:
            return IkResult(False, {}, float("inf"), float("inf"), 0, message="no movable joints between base and tool")
        lo, hi = self._limits()
        rng = np.random.default_rng(seed)
        q0 = np.clip(self._q(start), lo, hi)
        best: Optional[Tuple[float, np.ndarray, float, float, int]] = None
        total = 0
        for attempt in range(restarts + 1):
            q = q0.copy() if attempt == 0 else rng.uniform(lo, hi)
            damping = 0.05
            for it in range(max_iterations):
                jac, t = self.jacobian(q)
                e_pos = target[:3, 3] - t[:3, 3]
                e_rot = np.zeros(3) if position_only else rotation_error(target[:3, :3], t[:3, :3])
                ep, er = float(np.linalg.norm(e_pos)), float(np.linalg.norm(e_rot))
                if ep <= tol_position and er <= tol_rotation:
                    break
                if position_only:
                    jac, err = jac[:3], e_pos
                else:
                    err = np.concatenate([e_pos, e_rot * 0.3])   # metres vs radians: weigh rotation a bit less
                    jac = np.vstack([jac[:3], jac[3:] * 0.3])
                jj = jac @ jac.T
                dq = jac.T @ np.linalg.solve(jj + damping ** 2 * np.eye(jj.shape[0]), err)
                step = float(np.max(np.abs(dq)))
                if step > 0.3:              # keep steps small so the linearisation holds
                    dq *= 0.3 / step
                q = np.clip(q + dq, lo, hi)
            total += it + 1
            t = self._frames(q)[0]
            ep = float(np.linalg.norm(target[:3, 3] - t[:3, 3]))
            er = 0.0 if position_only else float(np.linalg.norm(rotation_error(target[:3, :3], t[:3, :3])))
            score = ep + 0.1 * er
            if best is None or score < best[0]:
                best = (score, q.copy(), ep, er, total)
            if ep <= tol_position and er <= tol_rotation:
                break
        assert best is not None
        _, q, ep, er, total = best
        ok = ep <= tol_position and er <= tol_rotation
        margin = np.radians(5)
        near = [j.command_name for j, x, a, b in zip(self.movable, q, lo, hi)
                if j.lower is not None and j.upper is not None
                and min(x - a, b - x) < (margin if j.type != "prismatic" else 0.05 * (b - a))]
        msg = "reachable" if ok else (
            f"not reachable: closest pose is {ep * 1000:.1f} mm"
            + ("" if position_only else f" / {math.degrees(er):.1f}°") + " away")
        return IkResult(ok, {j.command_name: float(x) for j, x in zip(self.movable, q)},  # type: ignore[misc]
                        ep, er, total, near, msg)
