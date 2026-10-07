"""
Joint descriptions, goal validation and time-parameterised interpolation.

The C# port (unity/.../Runtime/Core/Trajectory.cs) mirrors this file — keep them in step.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence

SPEED_PEAK_FACTOR = 1.5  # cubic segments peak at up to ~1.5× the average segment speed
REPORT_MODES = ("none", "points", "progress", "all")
ON_BUSY_MODES = ("queue", "replace", "reject", "parallel")
INTERPOLATIONS = ("cubic", "linear")


@dataclass
class Joint:
    name: str
    type: str = "revolute"           # revolute (rad) | prismatic (m)
    lower: Optional[float] = None
    upper: Optional[float] = None
    max_velocity: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        return {"name": self.name, "type": self.type, "lower": self.lower,
                "upper": self.upper, "max_velocity": self.max_velocity}

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "Joint":
        return cls(name=d["name"], type=d.get("type", "revolute"), lower=d.get("lower"),
                   upper=d.get("upper"), max_velocity=d.get("max_velocity"))


class GoalError(ValueError):
    """The goal is invalid — the message is sent back as the rejection reason."""


@dataclass
class GoalSpec:
    joint_names: List[str]
    times: List[float]
    positions: List[List[float]]   # [point][joint in joint_names order]
    report: str = "points"
    progress_hz: float = 10.0
    on_busy: str = "queue"
    interpolation: str = "cubic"
    # Stream goals have no points: the controller sends `stream` messages with the newest pose, and the robot
    # follows it every tick at up to stream_speed × each joint's max_velocity (see MotionExecutor._tick_stream).
    stream: bool = False
    stream_speed: float = 1.0

    @property
    def reports_points(self) -> bool:
        return self.report in ("points", "all") and not self.stream

    @property
    def reports_progress(self) -> bool:
        return self.report in ("progress", "all")


def parse_goal(payload: Dict[str, Any], joints: Dict[str, Joint]) -> GoalSpec:
    """Validate an `execute` payload against the robot's joints. Raises GoalError."""
    names = payload.get("joint_names")
    if not isinstance(names, list) or not names or not all(isinstance(n, str) for n in names):
        raise GoalError("joint_names must be a non-empty list of strings")
    if len(set(names)) != len(names):
        raise GoalError("joint_names contains duplicates")
    for n in names:
        if n not in joints:
            raise GoalError(f"unknown joint '{n}'")

    stream = payload.get("stream", False) is True
    points = payload.get("points")
    if stream:
        points = []                  # a stream goal follows `stream` messages instead
    elif not isinstance(points, list) or not points:
        raise GoalError("points must be a non-empty list")
    times: List[float] = []
    positions: List[List[float]] = []
    prev_t = 0.0
    for i, p in enumerate(points):
        try:
            t = float(p["time_from_start"])
            pos = [float(x) for x in p["positions"]]
        except (KeyError, TypeError, ValueError):
            raise GoalError(f"point {i}: needs numeric 'positions' and 'time_from_start'")
        if len(pos) != len(names):
            raise GoalError(f"point {i}: {len(pos)} positions for {len(names)} joints")
        if t <= prev_t:
            raise GoalError(f"point {i}: time_from_start must be > {prev_t:g}")
        for name, x in zip(names, pos):
            j = joints[name]
            if j.lower is not None and x < j.lower - 1e-9:
                raise GoalError(f"point {i}: {name}={x:g} below lower limit {j.lower:g}")
            if j.upper is not None and x > j.upper + 1e-9:
                raise GoalError(f"point {i}: {name}={x:g} above upper limit {j.upper:g}")
        if positions:
            check_segment_speed(names, joints, positions[-1], pos, t - prev_t, f"points {i - 1}→{i}")
        times.append(t)
        positions.append(pos)
        prev_t = t

    report = payload.get("report", "points")
    on_busy = payload.get("on_busy", "queue")
    interpolation = payload.get("interpolation", "cubic")
    if report not in REPORT_MODES:
        raise GoalError(f"report must be one of {', '.join(REPORT_MODES)}")
    if on_busy not in ON_BUSY_MODES:
        raise GoalError(f"on_busy must be one of {', '.join(ON_BUSY_MODES)}")
    if interpolation not in INTERPOLATIONS:
        raise GoalError(f"interpolation must be one of {', '.join(INTERPOLATIONS)}")
    try:
        progress_hz = float(payload.get("progress_hz", 10.0))
    except (TypeError, ValueError):
        raise GoalError("progress_hz must be a number")
    progress_hz = min(max(progress_hz, 0.5), 100.0)
    try:
        stream_speed = float(payload.get("speed", 1.0))
    except (TypeError, ValueError):
        raise GoalError("speed must be a number")
    if stream and not 0.0 < stream_speed <= 1.0:
        raise GoalError("speed must be in (0, 1]: the fraction of each joint's max_velocity")
    return GoalSpec(names, times, positions, report, progress_hz, on_busy, interpolation, stream, stream_speed)


def clamp_to_limits(joint: Joint, x: float) -> float:
    if joint.lower is not None and x < joint.lower:
        return joint.lower
    if joint.upper is not None and x > joint.upper:
        return joint.upper
    return x


def check_segment_speed(names: Sequence[str], joints: Dict[str, Joint], a: Sequence[float],
                        b: Sequence[float], dt: float, label: str) -> None:
    for name, xa, xb in zip(names, a, b):
        vmax = joints[name].max_velocity
        if vmax is None or vmax <= 0:
            continue
        peak = SPEED_PEAK_FACTOR * abs(xb - xa) / dt
        if peak > vmax + 1e-9:
            raise GoalError(f"{label}: {name} would need ~{peak:.2f}/s, max_velocity is {vmax:g}/s")


class Trajectory:
    """
    Piecewise interpolation from a start pose through timed points.

    cubic:  Hermite segments; zero velocity at the start and the end. Interior velocities are the
            mean of the neighbouring segment slopes, 0 where the motion reverses (no overshoot), and
            capped at 1.5× the smaller slope — which keeps every segment's peak speed within
            SPEED_PEAK_FACTOR × its average speed, so the speed check in parse_goal is exact.
    linear: straight segments.
    """

    def __init__(self, start: Sequence[float], times: Sequence[float],
                 positions: Sequence[Sequence[float]], interpolation: str = "cubic") -> None:
        self.t = [0.0] + list(times)
        self.x = [list(start)] + [list(p) for p in positions]
        self.n = len(start)
        self.interpolation = interpolation
        self.v = self._velocities() if interpolation == "cubic" else None

    @property
    def duration(self) -> float:
        return self.t[-1]

    def _velocities(self) -> List[List[float]]:
        k = len(self.t)
        v = [[0.0] * self.n for _ in range(k)]
        for i in range(1, k - 1):
            for j in range(self.n):
                s0 = (self.x[i][j] - self.x[i - 1][j]) / (self.t[i] - self.t[i - 1])
                s1 = (self.x[i + 1][j] - self.x[i][j]) / (self.t[i + 1] - self.t[i])
                if s0 * s1 <= 0:
                    continue
                cap = SPEED_PEAK_FACTOR * min(abs(s0), abs(s1))
                mean = 0.5 * (s0 + s1)
                v[i][j] = max(-cap, min(cap, mean))
        return v

    def sample(self, t: float) -> List[float]:
        if t <= 0:
            return list(self.x[0])
        if t >= self.t[-1]:
            return list(self.x[-1])
        i = 1
        while self.t[i] < t:
            i += 1
        t0, t1 = self.t[i - 1], self.t[i]
        h = t1 - t0
        u = (t - t0) / h
        a, b = self.x[i - 1], self.x[i]
        if self.v is None:
            return [a[j] + (b[j] - a[j]) * u for j in range(self.n)]
        va, vb = self.v[i - 1], self.v[i]
        h00 = 2 * u ** 3 - 3 * u ** 2 + 1
        h10 = u ** 3 - 2 * u ** 2 + u
        h01 = -2 * u ** 3 + 3 * u ** 2
        h11 = u ** 3 - u ** 2
        return [h00 * a[j] + h10 * h * va[j] + h01 * b[j] + h11 * h * vb[j] for j in range(self.n)]
