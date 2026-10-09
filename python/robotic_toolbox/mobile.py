"""
Drive tab backend: drive a mobile robot (Unity: Mobile Base Drive) by turning its wheel joints.

The robot's description carries a "mobile_base" section (see PROTOCOL.md):
    {"type": "differential", "left_wheel", "right_wheel", "wheel_radius", "track",
     "left_sign", "right_sign",            +1: a positive wheel angle drives the robot forward
     "forward": [x, y, z],                 the robot's forward direction in the base link frame
     "center": [x, y, z],                  the point it turns about in place, in the base link frame
     "casters": [...]}
Units and frames are ROS (metres, radians, z up, yaw counter-clockwise = turning left).

Position and heading: the robot's position is its turning centre in the scene; its heading is the angle of its forward
direction from scene +x, counter-clockwise seen from above (0 = facing +x, +pi/2 = facing +y). Scene +x is Unity's
+z, scene +y is Unity's -x.

Moves are made of segments (distance, turn): travel left = d - turn·track/2, travel right = d + turn·track/2, wheel angle =
sign · travel / radius. Driving to a position is turn → straight → (optionally) turn, each a goal of its own so the robot
stops between them and stays on the straight line. Wheel goals use on_busy="parallel" like every Toolbox move, so the
arm can work while the base drives.
"""
from __future__ import annotations

import asyncio
import math
from typing import TYPE_CHECKING, Any, Dict, List, Mapping, Optional, Tuple

import numpy as np

if TYPE_CHECKING:
    from .backend import Backend

FAR = 100.0             # m (or rad of turning): "hold to drive" heads this far, until released


def wrap(angle: float) -> float:
    """An angle in (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def wheel_deltas(info: Mapping[str, Any], distance: float, turn: float) -> Tuple[float, float]:
    """Wheel angle changes (rad) for driving `distance` m forward and turning `turn` rad (left positive)."""
    r, track = float(info["wheel_radius"]), float(info["track"])
    left = distance - turn * track / 2
    right = distance + turn * track / 2
    return float(info.get("left_sign", 1)) * left / r, float(info.get("right_sign", 1)) * right / r


def quat_matrix(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def base_position(info: Mapping[str, Any], base_pose: Mapping[str, Any]) -> Tuple[float, float, float]:
    """(x, y, heading) of the robot's turning centre in the scene, heading = yaw of its forward direction."""
    rot = quat_matrix(base_pose.get("orientation", (0, 0, 0, 1)))
    pos = np.array(base_pose.get("position", (0, 0, 0)), dtype=float)
    center = pos + rot @ np.array(info.get("center", (0, 0, 0)), dtype=float)
    forward = rot @ np.array(info.get("forward", (1, 0, 0)), dtype=float)
    return float(center[0]), float(center[1]), math.atan2(forward[1], forward[0])


def plan_drive_to(start: Tuple[float, float, float], x: float, y: float, heading: Optional[float] = None,
                  allow_reverse: bool = True) -> List[Tuple[float, float]]:
    """Segments (distance, turn) from `start` (x, y, heading) to (x, y) and, if given, the final heading:
    turn towards the goal, drive straight, turn to the heading. With allow_reverse it backs up instead of turning
    round when the goal is behind."""
    sx, sy, sh = start
    dx, dy = x - sx, y - sy
    dist = math.hypot(dx, dy)
    segments: List[Tuple[float, float]] = []
    h = sh
    if dist > 1e-4:
        turn = wrap(math.atan2(dy, dx) - sh)
        if allow_reverse and abs(turn) > math.pi / 2:
            turn, dist = wrap(turn - math.pi), -dist
        if abs(turn) > 1e-4:
            segments.append((0.0, turn))
        segments.append((dist, 0.0))
        h = sh + turn
    if heading is not None:
        turn = wrap(heading - h)
        if abs(turn) > 1e-4:
            segments.append((0.0, turn))
    return segments


class MobileDrive:
    """Drive commands for the selected robot, on the backend's controller thread."""

    def __init__(self, backend: "Backend") -> None:
        self.b = backend
        self._task: Optional[asyncio.Task] = None
        self.message = ""

    @property
    def info(self) -> Optional[Dict[str, Any]]:
        """The robot's "mobile_base" description, or None if it cannot drive."""
        return (self.b.live or {}).get("mobile_base") or (self.b.description or {}).get("mobile_base")

    @property
    def wheels(self) -> List[str]:
        info = self.info
        return [info["left_wheel"], info["right_wheel"]] if info else []

    def position(self) -> Optional[Tuple[float, float, float]]:
        """(x, y, heading) of the robot in the scene, from the latest live describe."""
        info, pose = self.info, (self.b.live or {}).get("base_pose")
        return base_position(info, pose) if info and pose else None

    def _require(self) -> Dict[str, Any]:
        info = self.info
        if not info:
            raise RuntimeError("this robot cannot drive (Unity: Add Component › IVI Dynamic › Mobile Base Drive)")
        return info

    async def step(self, distance: float, turn: float, speed: float = 0.5, label: str = "drive"):
        """Drive `distance` m and turn `turn` rad (left positive) as one goal on the two wheels."""
        info = self._require()
        dl, dr = wheel_deltas(info, distance, turn)
        cur = await self.b._require_robot().current_positions()
        left, right = info["left_wheel"], info["right_wheel"]
        return await self.b.move_joints({left: cur[left] + dl, right: cur[right] + dr}, speed=speed, label=label)

    async def drive_to(self, x: float, y: float, heading: Optional[float] = None, speed: float = 0.5,
                       allow_reverse: bool = True) -> None:
        """Turn towards (x, y) in the scene, drive there, and turn to `heading` (rad) if given - in the background."""
        start = self.position()
        if start is None:
            raise RuntimeError("the robot's position is not known yet")
        segments = plan_drive_to(start, x, y, heading, allow_reverse)
        await self.stop()
        self._task = asyncio.ensure_future(self._run(segments, speed, f"drive to ({x:.3f}, {y:.3f})"))

    async def _run(self, segments: List[Tuple[float, float]], speed: float, label: str) -> None:
        try:
            for i, (dist, turn) in enumerate(segments):
                what = f"{label}: " + (f"turn {math.degrees(turn):+.1f}°" if turn else f"drive {dist:+.3f} m")
                goal = await self.step(dist, turn, speed, what)
                result = await goal.result()
                if result.get("status") != "succeeded":
                    self.message = f"{label}: {result.get('status')}"
                    return
            self.message = f"{label}: arrived"
        except Exception as exc:
            self.message = f"{label}: {exc}"
            self.b.log(self.message)

    async def start(self, distance_sign: int, turn_sign: int, speed: float = 0.5) -> None:
        """Hold to drive: forward / back (distance_sign) or turn left / right (turn_sign) until stop()."""
        await self.stop()
        await self.step(distance_sign * FAR, turn_sign * FAR, speed, "drive (hold)")

    async def stop(self) -> None:
        """Stop driving: the wheels slow down to a halt."""
        if self._task is not None and not self._task.done():
            self._task.cancel()
        wheels = self.wheels
        if wheels and self.b.running_goals(wheels):
            await self.b.cancel(wheels)
