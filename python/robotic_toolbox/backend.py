"""
Robotic Toolbox backend: runs the Remote Control MotionController on its own asyncio thread and offers
plain methods to the GUI. No GUI imports here (the GUI passes `post`, which runs a callable on its thread).

    backend = Backend(post=wx.CallAfter)
    backend.start()                      # connector + data_dir from %RC_CONFIG_DIR%\\remote_control.json
    backend.call(backend.jog_step("R_arm1", +0.05), on_done=..., on_error=...)

Every robot operation is a coroutine; `call` schedules it on the controller thread and reports back on the GUI
thread. Events (robot online/offline, goal results, log lines) arrive through the `on_*` callbacks, already on
the GUI thread.
"""
from __future__ import annotations

import asyncio
import math
import threading
import time
from concurrent.futures import Future
from pathlib import Path
from typing import Any, Awaitable, Callable, Dict, List, Mapping, Optional

import numpy as np

from remote_control import (ConfigError, GoalHandle, MotionController, RobotHandle,
                            controller_connector_from_config, controller_connector_from_url,
                            data_dir_from_config, load_system_config, system_config_path)

from .ik import Chain, IkResult, pose_from_xyz_rpy, xyz_rpy_from_pose

Post = Callable[[Callable[[], None]], Any]


def describe_connector(section: Mapping[str, Any]) -> str:
    """One-line description of a connector config section, without secrets."""
    if section.get("url"):
        return str(section["url"])
    opts = ", ".join(f"{k}={v}" for k, v in section.items()
                     if k not in ("type", "password") and v is not None)
    return f"{section.get('type')} ({opts})"


class Backend:
    POLL_PERIOD = 0.2          # s between position reads while a robot is connected

    def __init__(self, post: Post, url: Optional[str] = None, data_dir: Optional[str] = None) -> None:
        self._post = post
        self._url = url
        self._data_dir = data_dir
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.controller: Optional[MotionController] = None
        self.endpoint = ""
        self.config_path: Optional[Path] = None
        self.robot: Optional[RobotHandle] = None
        self.description: Optional[Dict[str, Any]] = None
        self.positions: Dict[str, float] = {}
        self.goal: Optional[GoalHandle] = None          # last goal sent from the toolbox
        self._jog_targets: Dict[str, float] = {}
        self._listening: set = set()
        self._poll_task: Optional[asyncio.Task] = None
        self._thread: Optional[threading.Thread] = None
        self._ready = threading.Event()
        self._start_error: Optional[BaseException] = None
        # GUI callbacks (called on the GUI thread)
        self.on_log: Callable[[str], None] = lambda text: None
        self.on_robots: Callable[[List[str]], None] = lambda ids: None
        self.on_robot_changed: Callable[[], None] = lambda: None
        self.on_positions: Callable[[Dict[str, float]], None] = lambda pos: None
        self.on_state: Callable[[Dict[str, Any]], None] = lambda state: None

    # ── thread plumbing ──────────────────────────────────────────────────────

    def start(self, timeout: float = 10.0) -> None:
        """Start the controller thread; raises ConfigError / OSError if the controller cannot start."""
        self._thread = threading.Thread(target=self._run, name="toolbox-controller", daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout):
            raise TimeoutError("controller did not start")
        if self._start_error is not None:
            raise self._start_error

    def stop(self) -> None:
        if self.loop and self.loop.is_running():
            fut = asyncio.run_coroutine_threadsafe(self._shutdown(), self.loop)
            try:
                fut.result(5)
            except Exception:
                pass
            self.loop.call_soon_threadsafe(self.loop.stop)
        if self._thread:
            self._thread.join(5)

    def _run(self) -> None:
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self._startup())
        except BaseException as exc:   # report to start()
            self._start_error = exc
            self._ready.set()
            return
        self._ready.set()
        self._poll_task = self.loop.create_task(self._poll_positions())
        try:
            self.loop.run_forever()
        finally:
            self.loop.close()

    def _emit(self, fn: Callable[..., None], *args: Any) -> None:
        self._post(lambda: fn(*args))

    def log(self, text: str) -> None:
        self._emit(self.on_log, f"{time.strftime('%H:%M:%S')}  {text}")

    def call(self, coro: Awaitable[Any], on_done: Optional[Callable[[Any], None]] = None,
             on_error: Optional[Callable[[BaseException], None]] = None) -> Future:
        """Run a coroutine on the controller thread; report the outcome on the GUI thread."""
        assert self.loop is not None, "start() first"
        fut = asyncio.run_coroutine_threadsafe(coro, self.loop)  # type: ignore[arg-type]

        def done(f: Future) -> None:
            exc = f.exception()
            if exc is not None:
                if on_error is not None:
                    self._post(lambda: on_error(exc))
                else:
                    self.log(f"error: {exc or type(exc).__name__}")
            elif on_done is not None:
                result = f.result()
                self._post(lambda: on_done(result))

        fut.add_done_callback(done)
        return fut

    # ── controller ───────────────────────────────────────────────────────────

    async def _startup(self) -> None:
        config: Dict[str, Any] = {}
        if self._url:
            connector = controller_connector_from_url(self._url)
            self.endpoint = self._url
        else:
            self.config_path = system_config_path()
            config = load_system_config()
            connector = controller_connector_from_config(config)
            self.endpoint = describe_connector(config["connector"])
        data_dir = self._data_dir
        if data_dir is None and config.get("data_dir"):
            data_dir = str(data_dir_from_config(config, self.config_path))
        hb = config.get("heartbeat", {})
        self.controller = MotionController(connector, heartbeat_interval=float(hb.get("interval", 0.5)),
                                           heartbeat_timeout=float(hb.get("timeout", 2.0)), data_dir=data_dir)
        self.controller.on("robot_online", lambda e: self._robot_event(e["robot_id"], True))
        self.controller.on("robot_offline", lambda e: self._robot_event(e["robot_id"], False))
        await self.controller.start()

    async def _shutdown(self) -> None:
        if self._poll_task is not None:
            self._poll_task.cancel()
            try:
                await self._poll_task
            except asyncio.CancelledError:
                pass
        if self.controller:
            await self.controller.stop()

    @property
    def data_dir(self) -> Optional[Path]:
        return self.controller.data_dir if self.controller else None

    def online_robots(self) -> List[str]:
        if not self.controller:
            return []
        return sorted(r.robot_id for r in self.controller.robots.values() if r.online)

    def _robot_event(self, robot_id: str, online: bool) -> None:
        self.log(f"{'●' if online else '○'} robot {'online' if online else 'offline'}: {robot_id}")
        self._emit(self.on_robots, self.online_robots())
        if self.robot is None and online:
            self.call(self.select(robot_id))   # errors go to the log
        elif self.robot is not None and self.robot.robot_id == robot_id:
            self._emit(self.on_robot_changed)

    async def select(self, robot_id: str) -> None:
        """Make robot_id the robot the toolbox controls; loads its description (kinematic tree)."""
        assert self.controller is not None
        robot = self.controller.robots.get(robot_id)
        if robot is None:
            raise KeyError(robot_id)
        if robot is not self.robot:
            self.robot = robot
            self.description = None
            self._jog_targets.clear()
            if robot.robot_id not in self._listening:
                self._listening.add(robot.robot_id)
                robot.on("state", lambda st, r=robot: r is self.robot and self._emit(self.on_state, dict(st)))
        try:
            self.description = await robot.describe(tree=True)
        except Exception as exc:
            self.description = None
            self.log(f"no kinematic tree from {robot_id}: {exc} (jog and poses still work)")
        self._emit(self.on_robot_changed)

    # ── positions ────────────────────────────────────────────────────────────

    async def _poll_positions(self) -> None:
        while True:
            await asyncio.sleep(self.POLL_PERIOD)
            robot = self.robot
            if robot is None or not robot.online:
                continue
            try:
                pos = await robot.current_positions()
            except Exception:
                continue
            self.positions = pos
            self._emit(self.on_positions, dict(pos))

    def joint(self, name: str) -> Any:
        assert self.robot is not None
        return next(j for j in self.robot.joints if j.name == name)

    # ── motion ───────────────────────────────────────────────────────────────

    def _require_robot(self) -> RobotHandle:
        if self.robot is None or not self.robot.online:
            raise RuntimeError("no robot connected")
        return self.robot

    def _watch(self, goal: GoalHandle, what: str) -> GoalHandle:
        self.goal = goal

        def on_result(p: Dict[str, Any]) -> None:
            status = p.get("status")
            msg = p.get("message") or ""
            if what.startswith("jog") and status == "canceled":
                self.log(f"{what}: stopped")
            else:
                self.log(f"{what}: {status}" + (f" ({msg})" if msg and msg != status else ""))
            if status != "succeeded":
                self._jog_targets.clear()

        goal.on("result", on_result)
        return goal

    @staticmethod
    def _clamp(joint: Any, x: float) -> float:
        if joint.lower is not None:
            x = max(joint.lower, x)
        if joint.upper is not None:
            x = min(joint.upper, x)
        return x

    def _duration(self, joint: Any, distance: float, speed: float, minimum: float = 0.3) -> float:
        """Time for `distance` at `speed` (0..1) of the joint's max velocity; the robot checks 1.5 × the average
        speed against max_velocity, so 1.5 plus a 10 % margin."""
        vmax = joint.max_velocity or 1.0
        return max(minimum, 1.5 * abs(distance) / (vmax * max(0.01, speed)) * 1.1)

    async def jog_step(self, name: str, delta: float, speed: float = 0.5) -> float:
        """Move one joint by delta (rad / m) from its last jog target (queued, so quick clicks add up)."""
        robot = self._require_robot()
        j = self.joint(name)
        active = self.goal is not None and not self.goal.done
        base = self._jog_targets.get(name) if active else None
        if base is None:
            base = (await robot.current_positions()).get(name, 0.0)
        target = self._clamp(j, base + delta)
        self._jog_targets[name] = target
        goal = await robot.execute([name], [([target], round(self._duration(j, target - base, speed), 3))],
                                   report="none", on_busy="queue")
        self._watch(goal, f"jog {name} → {target:.3f}")
        return target

    async def jog_start(self, name: str, direction: int, speed: float = 0.5) -> None:
        """Continuous jog: one goal towards the joint limit at jog speed; jog_stop() ramps it down."""
        robot = self._require_robot()
        j = self.joint(name)
        now = (await robot.current_positions()).get(name, 0.0)
        far = (j.upper if direction > 0 else j.lower)
        if far is None:
            far = now + direction * (2 * math.pi if j.type != "prismatic" else 1.0)
        target = self._clamp(j, far)
        if abs(target - now) < 1e-6:
            self.log(f"{name} is at its {'upper' if direction > 0 else 'lower'} limit")
            return
        self._jog_targets.clear()
        goal = await robot.execute([name], [([target], round(self._duration(j, target - now, speed), 3))],
                                   report="none", on_busy="replace")
        self._watch(goal, f"jog {name}")

    async def jog_stop(self) -> None:
        if self.goal is not None and not self.goal.done:
            await self.goal.cancel()
        self._jog_targets.clear()

    async def move_joints(self, positions: Mapping[str, float], duration: Optional[float] = None,
                          speed: float = 0.5, label: str = "move") -> GoalHandle:
        """Timed move of several joints; without duration, the slowest joint sets it (at `speed` of max)."""
        robot = self._require_robot()
        if duration is None:
            now = await robot.current_positions()
            duration = max([self._duration(self.joint(n), x - now.get(n, x), speed, 1.0)
                            for n, x in positions.items()] or [1.0])
        self._jog_targets.clear()
        goal = await robot.move_to(dict(positions), duration=round(duration, 3), report="points")
        return self._watch(goal, label)

    async def pause(self) -> Dict[str, Any]:
        if self.goal is None:
            raise RuntimeError("no goal to pause")
        return await self.goal.pause()

    async def resume(self) -> Dict[str, Any]:
        if self.goal is None:
            raise RuntimeError("no goal to resume")
        return await self.goal.resume()

    async def cancel(self) -> Dict[str, Any]:
        if self.goal is None:
            raise RuntimeError("no goal to cancel")
        return await self.goal.cancel()

    async def stop_all(self) -> Dict[str, Any]:
        self._jog_targets.clear()
        return await self._require_robot().stop()

    # ── description and poses ───────────────────────────────────────────────

    async def describe_and_save(self) -> Path:
        path, desc = await self._require_robot().save_description()
        self.description = desc
        self._emit(self.on_robot_changed)
        return path

    HOME = "home"

    def home_positions(self) -> Dict[str, float]:
        """The built-in home pose: every joint at 0, or at the limit nearest to 0."""
        robot = self._require_robot()
        return {j.name: self._clamp(j, 0.0) for j in robot.joints}

    def has_saved_pose(self, name: str) -> bool:
        robot = self.robot
        return robot is not None and self.data_dir is not None and name in robot.store.list_poses()

    def list_poses(self) -> List[Dict[str, Any]]:
        """Saved poses, plus the built-in 'home' unless a pose with that name was saved (it then replaces it)."""
        robot = self.robot
        if robot is None:
            return []
        out = []
        if self.data_dir is not None:
            store = robot.store
            for name in store.list_poses():
                p = store.get_pose(name)
                out.append({"name": name, "joints": len(p.get("positions", {})), "saved_at": p.get("saved_at", ""),
                            "builtin": False})
        if not any(p["name"] == self.HOME for p in out):
            out.insert(0, {"name": self.HOME, "joints": len(robot.joints), "saved_at": "built-in: all joints 0",
                           "builtin": True})
        return out

    async def save_pose(self, name: str) -> Path:
        return await self._require_robot().save_pose(name)

    def delete_pose(self, name: str) -> None:
        if name == self.HOME and not self.has_saved_pose(name):
            raise ValueError("the built-in home pose cannot be deleted (save a pose named 'home' to replace it)")
        self._require_robot().delete_pose(name)

    async def go_to_pose(self, name: str, speed: float = 0.5) -> GoalHandle:
        robot = self._require_robot()
        if name == self.HOME and not self.has_saved_pose(name):
            positions = self.home_positions()
        else:
            positions = robot.get_pose(name)
        return await self.move_joints(positions, speed=speed, label=f"pose '{name}'")

    # ── targets (IK) ─────────────────────────────────────────────────────────

    def chain(self, tool: str, base: Optional[str] = None) -> Chain:
        if not self.description or not self.description.get("joints"):
            raise RuntimeError("the robot sent no kinematic tree (Robot → Describe & save to retry)")
        return Chain.from_tree(self.description, tool, base)

    def tools(self) -> List[str]:
        return Chain.tool_candidates(self.description) if self.description else []

    def bases_for(self, tool: str) -> List[str]:
        """Links a target can be relative to: the tool's ancestors (root first)."""
        if not self.description:
            return []
        return list(reversed(Chain.ancestors(self.description, tool)[1:]))

    def tool_pose(self, tool: str, base: Optional[str] = None):
        """(xyz, rpy) of the tool in `base` at the latest measured positions."""
        return xyz_rpy_from_pose(self.chain(tool, base).tool_pose(self.positions))

    def check_target(self, tool: str, base: Optional[str], xyz, rpy, position_only: bool) -> IkResult:
        chain = self.chain(tool, base)
        return chain.solve(pose_from_xyz_rpy(xyz, rpy), self.positions, position_only=position_only)

    async def check_target_async(self, tool: str, base: Optional[str], xyz, rpy, position_only: bool) -> IkResult:
        """check_target on a worker thread (an unreachable target tries several starts)."""
        return await asyncio.get_event_loop().run_in_executor(
            None, lambda: self.check_target(tool, base, xyz, rpy, position_only))

    async def move_to_target(self, tool: str, base: Optional[str], xyz, rpy, position_only: bool,
                             duration: Optional[float] = None, speed: float = 0.5) -> IkResult:
        result = await self.check_target_async(tool, base, xyz, rpy, position_only)
        if not result.reachable:
            return result
        await self.move_joints(result.positions, duration, speed, label=f"target {tool}")
        return result
