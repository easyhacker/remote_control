"""
Motion tab backend: motion programs and the external motion planner endpoint.

Scope - which joints a motion owns: all joints, a joint group, a saved chain (needed for TCP targets) or one joint.
Every motion runs as goals on its scope's joints only (on_busy="parallel" when the robot supports it), so other
joints can be jogged or run their own motion at the same time.

Program - steps run in order, automatically, with pause / resume / stop / single step and loop:
    {"type": "pose",   "pose": "<saved pose>"}                       the pose's values for the scope's joints
    {"type": "joints", "positions": {"<joint>": value, …}}           fixed joint values (captured from the robot)
    {"type": "target", "target": "<target id>", "label": "<name>",   a TCP target of the scope's chain
     "move": "joint" | "linear", "position_only": false}
  plus "speed" (joint moves: fraction of each joint's max velocity; linear moves: TCP speed in m/s) and "wait"
  (seconds after the step). Program options: "loop", and "blend" (smooth, the default: pass through the steps without
  stopping - see _segment). Programs are saved per robot in programs.json.

Joint move: IK at the target, then one timed joint move (always smooth). Linear move: the TCP follows a straight line
(position interpolated, orientation slerped); IK is solved every few millimetres from the previous solution, and the
path fails if a point is unreachable or the arm would flip between samples.

External planner: a program on this PC connects to ws://127.0.0.1:<port>/planner and sends joint poses, e.g. one per
frame. While "Follow planner" is on, the Toolbox forwards them to the robot as a stream goal on the scope's joints
(see PROTOCOL.md `stream`): the robot moves to the newest pose every tick within max_velocity; pause / resume / stop
work as for programs. Messages (JSON text):
    planner → toolbox  {"positions": {"<joint>": value, …}}      or {"joint_names": [...], "positions": [...]}
                       optional "end": true (finish at this pose)
    toolbox → planner  {"type": "hello", "robot", "joints": [{"name", "lower", "upper", "max_velocity"}], "scope"}
                       {"type": "state", "following": bool, "state", "positions": {...}}   about 30 times a second
"""
from __future__ import annotations

import asyncio
import json
import math
import time
from typing import TYPE_CHECKING, Any, Callable, Dict, List, Mapping, Optional, Tuple

import numpy as np

from .ik import Chain, pose_from_xyz_rpy, rotation_error

if TYPE_CHECKING:
    from remote_control import GoalHandle
    from .backend import Backend

PROGRAMS_FILE = "programs.json"
STEP_TYPES = ("pose", "joints", "target")
MOVES = ("joint", "linear")
LINEAR_STEP_M = 0.005                  # linear moves: an IK point every 5 mm …
LINEAR_STEP_RAD = math.radians(2)      # … or every 2° of TCP rotation
LINEAR_MAX_POINTS = 400
LINEAR_MAX_JUMP = 0.5                  # rad / m between neighbouring samples: more means the arm flips
SPEED_MARGIN = 1.6                     # cubic segments peak at 1.5 × their average speed, plus a margin
PLANNER_PATH = "/planner"
PLANNER_STATE_HZ = 30.0


class MotionError(RuntimeError):
    """A step cannot be planned (unreachable target, wrong scope …): the message is shown to the user."""


# ── scopes ───────────────────────────────────────────────────────────────────

def scope_label(scope: Mapping[str, Any]) -> str:
    kind, name = scope.get("kind", "all"), scope.get("name")
    return {"all": "All joints", "group": f"Group: {name}", "chain": f"Chain: {name}",
            "joint": f"Joint: {name}"}.get(kind, str(scope))


# ── linear TCP paths ─────────────────────────────────────────────────────────

def _rot_exp(w: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(w))
    if angle < 1e-12:
        return np.eye(3)
    k = w / angle
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * kx + (1 - math.cos(angle)) * kx @ kx


def linear_path(chain: Chain, start: Mapping[str, float], target: np.ndarray, position_only: bool = False,
                ) -> List[Tuple[Dict[str, float], float, float]]:
    """IK solutions along the straight TCP line from the pose at `start` to `target` (4x4 in the chain base).
    Returns [(positions, metres, radians of this segment)] for every sample after the start. Raises MotionError."""
    t0 = chain.tool_pose(start)
    p0, r0 = t0[:3, 3], t0[:3, :3]
    p1, r1 = target[:3, 3], target[:3, :3]
    w = rotation_error(r1, r0)                       # r1 = exp(w) · r0, in the base frame
    dist, ang = float(np.linalg.norm(p1 - p0)), (0.0 if position_only else float(np.linalg.norm(w)))
    n = int(min(LINEAR_MAX_POINTS, max(1, math.ceil(max(dist / LINEAR_STEP_M, ang / LINEAR_STEP_RAD)))))
    q = {k: float(v) for k, v in start.items()}
    out: List[Tuple[Dict[str, float], float, float]] = []
    for i in range(1, n + 1):
        s = i / n
        t = np.eye(4)
        t[:3, 3] = p0 + s * (p1 - p0)
        t[:3, :3] = r0 if position_only else _rot_exp(s * w) @ r0
        res = chain.solve(t, q, position_only=position_only, restarts=0, max_iterations=150)
        if not res.reachable:
            raise MotionError(f"the straight line leaves the reachable space at {s * 100:.0f}% "
                              f"({res.message}) - use a joint move")
        jump = max((abs(res.positions[k] - q.get(k, res.positions[k])) for k in res.positions), default=0.0)
        if jump > LINEAR_MAX_JUMP:
            raise MotionError(f"the arm would flip at {s * 100:.0f}% of the straight line - use a joint move")
        q.update(res.positions)
        out.append((dict(res.positions), dist / n, ang / n))
    return out


# ── runner ───────────────────────────────────────────────────────────────────

class MotionRunner:
    """Runs motion programs and follows the external planner, on the backend's controller thread."""

    def __init__(self, backend: "Backend") -> None:
        self.b = backend
        self.state = "idle"                    # program: idle | running | paused | stopping
        self.program: Optional[Dict[str, Any]] = None
        self.step_index: Optional[int] = None
        self.loop_count = 0
        self.message = ""
        self.goal: Optional["GoalHandle"] = None
        self._task: Optional[asyncio.Task] = None
        self._resume = asyncio.Event()
        self._resume.set()
        self._stop = False
        # planner
        self.planner_goal: Optional["GoalHandle"] = None
        self.planner_scope: List[str] = []
        self.planner_clients: set = set()
        self.planner_poses = 0
        self.planner_url = ""
        self._planner_server: Any = None
        self._planner_task: Optional[asyncio.Task] = None
        self.on_status: Callable[[Dict[str, Any]], None] = lambda status: None   # GUI thread

    # scopes

    def scopes(self) -> List[Dict[str, Any]]:
        """Scopes to offer: all joints, groups, saved chains, single joints."""
        robot = self.b.robot
        if robot is None:
            return []
        out: List[Dict[str, Any]] = [{"kind": "all"}]
        out += [{"kind": "group", "name": g["name"]} for g in self.b.groups()]
        out += [{"kind": "chain", "name": n} for n in sorted(self.b.chains())]
        out += [{"kind": "joint", "name": n} for n in robot.joint_names]
        return out

    def scope_joints(self, scope: Mapping[str, Any]) -> List[str]:
        robot = self.b._require_robot()
        kind, name = scope.get("kind", "all"), scope.get("name")
        if kind == "all":
            return list(robot.joint_names)
        if kind == "group":
            return list(self.b.group_joints(name) or [])
        if kind == "chain":
            return self.scope_chain(scope)[1].joint_names
        if kind == "joint":
            if name not in robot.joint_names:
                raise MotionError(f"no joint '{name}'")
            return [name]
        raise MotionError(f"unknown scope {scope!r}")

    def scope_chain(self, scope: Mapping[str, Any]) -> Tuple[Dict[str, Any], Chain]:
        if scope.get("kind") != "chain":
            raise MotionError("a TCP target needs a chain scope - pick a saved chain in Scope")
        c = self.b.chains().get(scope.get("name"))
        if c is None:
            raise MotionError(f"no saved chain '{scope.get('name')}'")
        return c, self.b.chain(c["end"], c["origin"], scope["name"])

    # programs (programs.json in the robot's data folder)

    def programs(self) -> Dict[str, Dict[str, Any]]:
        return self.b._doc(PROGRAMS_FILE, "programs")

    def save_program(self, name: str, program: Mapping[str, Any]) -> None:
        name = name.strip()
        if not name:
            raise ValueError("program name must not be empty")
        programs = self.programs()
        programs[name] = {"scope": dict(program.get("scope") or {"kind": "all"}),
                          "steps": [dict(s) for s in program.get("steps", [])],
                          "loop": bool(program.get("loop")),
                          "blend": bool(program.get("blend", True)),
                          "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
        self.b._save_doc(PROGRAMS_FILE, "programs", programs)

    def delete_program(self, name: str) -> None:
        programs = self.programs()
        if name not in programs:
            raise KeyError(f"no program '{name}'")
        del programs[name]
        self.b._save_doc(PROGRAMS_FILE, "programs", programs)

    @staticmethod
    def step_label(step: Mapping[str, Any]) -> str:
        kind = step.get("type")
        if kind == "pose":
            return f"pose '{step.get('pose')}'"
        if kind == "joints":
            pos = step.get("positions", {})
            return "joints " + ", ".join(f"{k}={v:.3f}" for k, v in list(pos.items())[:3]) + (" …" if len(pos) > 3 else "")
        if kind == "target":
            return f"target '{step.get('label') or step.get('target')}'"
        return str(step)

    # planning one step

    async def plan_step(self, step: Mapping[str, Any], scope: Mapping[str, Any],
                        start: Optional[Mapping[str, float]] = None) -> Dict[str, Any]:
        """{"positions": {...}} for a joint move, or {"path": [(positions, seconds from the step's start)…]} for a
        linear move. `start`: the joint positions the step starts from (default: the measured ones) - smooth
        programs plan each step from where the previous one ends."""
        start = dict(self.b.positions if start is None else start)
        joints = self.scope_joints(scope)
        kind = step.get("type")
        if kind in ("pose", "joints"):
            if kind == "pose":
                name = step.get("pose")
                if name == self.b.HOME and not self.b.has_saved_pose(name):
                    pose = self.b.home_positions()
                else:
                    pose = self.b._require_robot().get_pose(name)
            else:
                pose = dict(step.get("positions", {}))
            positions = {n: float(x) for n, x in pose.items() if n in joints}
            if not positions:
                raise MotionError(f"{self.step_label(step)} has no joints of {scope_label(scope)}")
            return {"positions": positions}
        if kind == "target":
            c, chain = self.scope_chain(scope)
            xyz, rpy = self.b.target_pose(step["target"], "chain", c["origin"])
            target = pose_from_xyz_rpy(xyz, rpy)
            position_only = bool(step.get("position_only"))
            loop = asyncio.get_event_loop()
            if step.get("move", "joint") == "linear":
                samples = await loop.run_in_executor(
                    None, lambda: linear_path(chain, start, target, position_only))
                return {"path": self._time_path(samples, float(step.get("speed", 0.1)), start)}
            result = await loop.run_in_executor(None, lambda: chain.solve(target, start,
                                                                          position_only=position_only))
            if not result.reachable:
                raise MotionError(f"{self.step_label(step)}: {result.message}")
            return {"positions": result.positions}
        raise MotionError(f"unknown step type {kind!r}")

    def _time_path(self, samples, tcp_speed: float, start: Mapping[str, float]) -> List[Tuple[Dict[str, float], float]]:
        """Timestamps for a linear path: TCP speed (m/s, rotation at 1 rad/s per 0.1 m/s), slowed down wherever a
        joint would exceed its max velocity."""
        tcp_speed = max(1e-3, tcp_speed)
        rot_speed = tcp_speed * 10.0
        prev = dict(start)
        t, out = 0.0, []
        for positions, metres, radians in samples:
            dt = max(metres / tcp_speed, radians / rot_speed, 0.02)
            for name, x in positions.items():
                vmax = self.b.joint(name).max_velocity
                if vmax:
                    dt = max(dt, SPEED_MARGIN * abs(x - prev.get(name, x)) / vmax)
            t += dt
            out.append((positions, round(t, 4)))
            prev.update(positions)
        return out

    # Steps are run in segments: the steps of a segment are planned ahead, each from where the previous one ends,
    # and sent as ONE goal through all their points. The robot's cubic interpolation passes through the points without
    # stopping (it only slows where a joint turns back), so a smooth program does not halt between steps. A segment
    # ends at a step with a wait, at the end of the program, and - when smooth is off - after every step.

    def _segment(self, index: int, single_step: bool) -> List[int]:
        assert self.program is not None
        steps = self.program["steps"]
        seg = [index]
        if single_step or not self.program["blend"]:
            return seg
        while float(steps[seg[-1]].get("wait", 0.0)) <= 0 and seg[-1] + 1 < len(steps):
            seg.append(seg[-1] + 1)
        return seg

    async def _plan_segment(self, seg: List[int], scope: Mapping[str, Any], start: Mapping[str, float]):
        """(joint names, [(positions, time)], index of each step's last point) for one goal through `seg`."""
        assert self.program is not None
        cur = dict(start)
        snapshots: List[Tuple[Dict[str, float], float]] = []
        moved: List[str] = []
        last_point: List[int] = []
        t = 0.0
        for i in seg:
            step = self.program["steps"][i]
            try:
                plan = await self.plan_step(step, scope, cur)
            except MotionError as exc:
                raise MotionError(f"step {i + 1}: {exc}")
            if "positions" in plan:
                speed = float(step.get("speed", 0.5))
                dt = max([self.b._duration(self.b.joint(n), x - cur.get(n, x), speed, 0.3)
                          for n, x in plan["positions"].items()] or [0.3])
                cur.update(plan["positions"])
                t += dt
                snapshots.append((dict(cur), round(t, 4)))
                moved += [n for n in plan["positions"] if n not in moved]
            else:
                for positions, tr in plan["path"]:
                    cur.update(positions)
                    snapshots.append((dict(cur), round(t + tr, 4)))
                t += plan["path"][-1][1]
                moved += [n for n in plan["path"][0][0] if n not in moved]
            last_point.append(len(snapshots) - 1)
        names = [n for n in self.b._require_robot().joint_names if n in moved]
        return names, [([p[n] for n in names], tt) for p, tt in snapshots], last_point

    def _on_point(self, point: int, seg: List[int], last_point: List[int]) -> None:
        """A step of a smooth segment is done when its last point is reached: highlight the next one."""
        for k, last in enumerate(last_point[:-1]):
            if point == last and self.state != "stopping":
                self.step_index = seg[k + 1]
                steps = self.program["steps"] if self.program else []
                self._status(f"step {seg[k + 1] + 1}/{len(steps)}: {self.step_label(steps[seg[k + 1]])}")

    # running programs

    @property
    def running(self) -> bool:
        return self._task is not None and not self._task.done()

    def _status(self, message: str = "") -> None:
        if message:
            self.message = message
        steps = len(self.program["steps"]) if self.program else 0
        self.b._emit(self.on_status, {"state": self.state, "step": self.step_index, "steps": steps,
                                      "loop": self.loop_count, "message": self.message,
                                      "planner": self.planner_status()})

    async def run(self, program: Mapping[str, Any], start: int = 0, single_step: bool = False) -> None:
        """Start a program at step `start` (in the background); single_step runs only that step."""
        if self.running:
            raise MotionError("a program is already running - stop it first")
        steps = list(program.get("steps", []))
        if not steps:
            raise MotionError("the program has no steps")
        self.program = {"scope": dict(program.get("scope") or {"kind": "all"}), "steps": steps,
                        "loop": bool(program.get("loop")) and not single_step,
                        "blend": bool(program.get("blend", True))}
        self.scope_joints(self.program["scope"])           # fail now if the scope is gone
        self._stop = False
        self._resume.set()
        self.loop_count = 0
        self._task = asyncio.ensure_future(self._run(max(0, min(start, len(steps) - 1)), single_step))

    async def _run(self, index: int, single_step: bool) -> None:
        assert self.program is not None
        steps, scope = self.program["steps"], self.program["scope"]
        self.state, self.message = "running", ""
        robot = self.b._require_robot()
        try:
            while not self._stop:
                self.step_index = index
                seg = self._segment(index, single_step)
                label = f"step {index + 1}/{len(steps)}: {self.step_label(steps[index])}"
                self._status(label)
                await self._resume.wait()
                if self._stop:
                    break
                start = await robot.current_positions()
                names, points, last_point = await self._plan_segment(seg, scope, start)
                goal = await robot.execute(names, points, report="points", on_busy=self.b._on_busy)
                self.goal = self.b._watch(goal, f"steps {seg[0] + 1}-{seg[-1] + 1}" if len(seg) > 1 else label, names)
                goal.on("point_reached", lambda p, s=seg, lp=last_point: self._on_point(p.get("point_index", -1), s, lp))
                if not self._resume.is_set():               # paused while this segment was being planned
                    await goal.pause()
                result = await goal.result()
                self.goal = None
                status = result.get("status")
                if status != "succeeded":
                    label = f"step {self.step_index + 1}/{len(steps)}: {self.step_label(steps[self.step_index])}"
                    self._status(f"{label}: {status}" + (f" ({result.get('message')})" if result.get("message") else ""))
                    break
                index = seg[-1]
                await self._wait(float(steps[index].get("wait", 0.0)))
                if single_step:
                    self._status(f"{label}: done")
                    break
                index += 1
                if index >= len(steps):
                    if not self.program["loop"]:
                        self._status("program finished")
                        break
                    index, self.loop_count = 0, self.loop_count + 1
        except MotionError as exc:
            self._status(f"stopped: {exc}")
        except Exception as exc:
            self._status(f"stopped: {exc or type(exc).__name__}")
        finally:
            if self._stop:
                self._status("stopped")
            self.state, self.goal = "idle", None
            self._status()

    async def _wait(self, seconds: float) -> None:
        """The step's wait; time spent paused does not count, stop ends it."""
        left = seconds
        while left > 0 and not self._stop:
            await self._resume.wait()
            t0 = time.monotonic()
            await asyncio.sleep(min(0.05, left))
            left -= time.monotonic() - t0

    async def pause(self) -> None:
        if self.planner_goal is not None and not self.planner_goal.done:
            await self.planner_goal.pause()
        if not self.running:
            self._status()
            return
        self._resume.clear()
        self.state = "paused"
        if self.goal is not None and not self.goal.done:
            await self.goal.pause()
        self._status("paused")

    async def resume(self) -> None:
        if self.planner_goal is not None and not self.planner_goal.done:
            await self.planner_goal.resume()
        if not self.running:
            self._status()
            return
        self.state = "running"
        self._resume.set()
        if self.goal is not None and not self.goal.done:
            await self.goal.resume()
        self._status("running")

    async def stop(self) -> None:
        """Stop the program and the planner stream (each slows down to a halt)."""
        if self.running:
            self._stop = True
            self.state = "stopping"
            self._resume.set()
            if self.goal is not None and not self.goal.done:
                await self.goal.cancel()
            self._status("stopping")
        await self.stop_following()

    # external planner

    def planner_status(self) -> Dict[str, Any]:
        g = self.planner_goal
        return {"url": self.planner_url, "clients": len(self.planner_clients), "poses": self.planner_poses,
                "following": g is not None and not g.done, "scope": list(self.planner_scope)}

    async def start_planner_server(self, host: str = "127.0.0.1", port: int = 8770) -> None:
        """Listen for planners; a busy port is logged, not fatal (the rest of the Toolbox works without it)."""
        try:
            try:
                from websockets.asyncio.server import serve
            except ImportError:                       # websockets < 13
                from websockets.server import serve   # type: ignore
            self._planner_server = await serve(self._planner_client, host, port)
        except OSError as exc:
            self.b.log(f"motion planner endpoint not available on {host}:{port}: {exc}")
            return
        self.planner_url = f"ws://{host}:{port}{PLANNER_PATH}"
        self._planner_task = asyncio.ensure_future(self._planner_state_loop())
        self.b.log(f"motion planners can connect to {self.planner_url}")

    async def stop_planner_server(self) -> None:
        if self._planner_task is not None:
            self._planner_task.cancel()
        if self._planner_server is not None:
            self._planner_server.close()
            await self._planner_server.wait_closed()

    async def follow(self, scope: Mapping[str, Any], speed: float = 1.0) -> None:
        """Start forwarding planner poses to the robot: a stream goal on the scope's joints."""
        await self.stop_following()
        joints = self.scope_joints(scope)
        robot = self.b._require_robot()
        goal = await robot.stream(joints, speed=max(0.05, min(1.0, speed)), on_busy=self.b._on_busy,
                                  report="progress", progress_hz=PLANNER_STATE_HZ)
        self.planner_goal, self.planner_scope, self.planner_poses = goal, joints, 0
        self.b._watch(goal, "planner", joints)
        goal.on("result", lambda p: self._status(f"planner stream: {p.get('status')}"))
        await self._broadcast(self._hello())
        self._status(f"following the planner on {scope_label(scope)}")

    async def stop_following(self) -> None:
        g = self.planner_goal
        if g is not None and not g.done:
            await g.cancel()
        self._status()

    def _hello(self) -> Dict[str, Any]:
        robot = self.b.robot
        return {"type": "hello", "robot": robot.robot_id if robot else None,
                "joints": [j.to_dict() for j in robot.joints] if robot else [],
                "scope": list(self.planner_scope)}

    async def _planner_client(self, ws: Any) -> None:
        path = getattr(getattr(ws, "request", None), "path", None) or getattr(ws, "path", PLANNER_PATH)
        if path.split("?")[0].rstrip("/") != PLANNER_PATH:
            await ws.close(code=4004, reason=f"use {PLANNER_PATH}")
            return
        self.planner_clients.add(ws)
        self._status(f"planner connected ({len(self.planner_clients)})")
        try:
            await ws.send(json.dumps(self._hello()))
            async for text in ws:
                await self._planner_message(text)
        except Exception:
            pass
        finally:
            self.planner_clients.discard(ws)
            self._status(f"planner disconnected ({len(self.planner_clients)} left)")

    async def _planner_message(self, text: str) -> None:
        try:
            msg = json.loads(text)
        except ValueError:
            return
        g = self.planner_goal
        if not isinstance(msg, dict) or g is None or g.done:
            return                                           # not following: poses are ignored
        pos = msg.get("positions")
        if isinstance(pos, list) and isinstance(msg.get("joint_names"), list):
            pos = dict(zip(msg["joint_names"], pos))
        if isinstance(pos, dict):
            scoped = {n: float(x) for n, x in pos.items() if n in self.planner_scope}
            if scoped:
                await g.send(scoped, end=msg.get("end") is True)
                self.planner_poses += 1
        elif msg.get("end") is True:
            await g.end()

    async def _planner_state_loop(self) -> None:
        while True:
            await asyncio.sleep(1.0 / PLANNER_STATE_HZ)
            if not self.planner_clients:
                continue
            positions = dict(self.b.positions)
            g = self.planner_goal
            following = g is not None and not g.done
            if following and g.last_feedback:
                positions.update(zip(g.joint_names, g.last_feedback.get("positions", [])))
            await self._broadcast({"type": "state", "following": following,
                                   "state": (g.last_feedback or {}).get("state") if following else "idle",
                                   "positions": positions})

    async def _broadcast(self, msg: Mapping[str, Any]) -> None:
        text = json.dumps(msg)
        for ws in list(self.planner_clients):
            try:
                await ws.send(text)
            except Exception:
                self.planner_clients.discard(ws)
