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
from typing import Any, Awaitable, Callable, Dict, List, Mapping, Optional, Tuple

import numpy as np

from remote_control import (ConfigError, GoalHandle, MotionController, RobotHandle,
                            controller_connector_from_config, controller_connector_from_url,
                            data_dir_from_config, load_system_config, system_config_path)

from remote_control.kinematics import forward_kinematics

from .ik import Chain, IkResult, pose_from_xyz_rpy, rpy_matrix, xyz_rpy_from_pose
from .mobile import MobileDrive
from .motion import MotionRunner

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

    def __init__(self, post: Post, url: Optional[str] = None, data_dir: Optional[str] = None,
                 planner_port: Optional[int] = None) -> None:
        self._post = post
        self._url = url
        self._data_dir = data_dir
        self._planner_port = planner_port         # None: from the config (default 8770); 0: no planner endpoint
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.controller: Optional[MotionController] = None
        self.endpoint = ""
        self.config_path: Optional[Path] = None
        self.robot: Optional[RobotHandle] = None
        self.description: Optional[Dict[str, Any]] = None
        self.positions: Dict[str, float] = {}
        self.live: Dict[str, Any] = {}    # latest describe(tree=False): base / robot pose, scene targets
        self.goal: Optional[GoalHandle] = None          # last goal sent from the toolbox
        # Per joint: where its pending jog steps end, so quick clicks add up instead of each starting from the
        # measured position. Dropped when a goal on that joint fails or a non-jog move takes the joint over.
        self._jog_targets: Dict[str, float] = {}
        # goal_id -> (goal, its joints) for goals sent from here that are still running: per-group stop /
        # pause / resume, and jog accumulation per joint, look goals up here by joint.
        self.goals: Dict[str, Tuple[GoalHandle, set]] = {}
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
        self.on_selected: Callable[[Optional[str]], None] = lambda item_id: None   # picked in the robot's viewer
        self.on_edited: Callable[[Dict[str, Any]], None] = lambda payload: None    # moved in the robot's viewer
        self.motion = MotionRunner(self)          # Motion tab: programs and the external planner endpoint
        self.drive = MobileDrive(self)            # Drive tab: mobile robots (Unity Mobile Base Drive)

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
        # optional "planner": {"host": "127.0.0.1", "port": 8770} in the config; port 0 turns the endpoint off
        planner = config.get("planner", {})
        port = int(planner.get("port", 8770)) if self._planner_port is None else self._planner_port
        if port:
            await self.motion.start_planner_server(str(planner.get("host", "127.0.0.1")), port)

    async def _shutdown(self) -> None:
        if self._poll_task is not None:
            self._poll_task.cancel()
            try:
                await self._poll_task
            except asyncio.CancelledError:
                pass
        await self.motion.stop_planner_server()
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
                robot.on("selected", lambda p, r=robot: r is self.robot and self._emit(self.on_selected, p.get("id")))
                robot.on("edited", lambda p, r=robot: r is self.robot and self._emit(self.on_edited, dict(p)))
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
                if robot.supports.get("describe"):
                    live = await robot.describe(tree=False)
                    pos = {k: float(v) for k, v in live.get("positions", {}).items() if v is not None}
                    self.live = live
                else:
                    pos = await robot.current_positions()
            except Exception:
                continue
            self.positions = pos
            self._emit(self.on_positions, dict(pos))

    def joint(self, name: str) -> Any:
        assert self.robot is not None
        return next(j for j in self.robot.joints if j.name == name)

    # ── motion ───────────────────────────────────────────────────────────────
    #
    # Every goal the toolbox sends runs in parallel with goals on other joints when the robot supports it
    # (on_busy="parallel"): the left arm can move while the right arm does something else. Goals on the same
    # joints still run in order.

    def _require_robot(self) -> RobotHandle:
        if self.robot is None or not self.robot.online:
            raise RuntimeError("no robot connected")
        return self.robot

    @property
    def _on_busy(self) -> str:
        return self._require_robot().parallel_on_busy

    def _watch(self, goal: GoalHandle, what: str, joints) -> GoalHandle:
        """Track a goal until its result: log it, and forget it (and its joints' jog targets unless it succeeded)."""
        self.goal = goal
        self.goals[goal.goal_id] = (goal, set(joints))

        def on_result(p: Dict[str, Any]) -> None:
            self.goals.pop(goal.goal_id, None)
            status = p.get("status")
            msg = p.get("message") or ""
            if what.startswith("jog") and status == "canceled":
                self.log(f"{what}: stopped")
            else:
                self.log(f"{what}: {status}" + (f" ({msg})" if msg and msg != status else ""))
            if status != "succeeded":
                for j in joints:
                    self._jog_targets.pop(j, None)

        goal.on("result", on_result)
        return goal

    def running_goals(self, joints=None) -> List[GoalHandle]:
        """Goals sent from here that have not ended; only those touching `joints` if given."""
        return [g for g, js in list(self.goals.values())
                if not g.done and (joints is None or js & set(joints))]

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
        """Move one joint by delta (rad / m) from its last jog target (quick clicks add up)."""
        robot = self._require_robot()
        j = self.joint(name)
        # While a jog of this joint is still running, continue from its target; otherwise from the measured position.
        base = self._jog_targets.get(name) if self.running_goals([name]) else None
        if base is None:
            base = (await robot.current_positions()).get(name, 0.0)
        target = self._clamp(j, base + delta)
        self._jog_targets[name] = target
        goal = await robot.execute([name], [([target], round(self._duration(j, target - base, speed), 3))],
                                   report="none", on_busy=self._on_busy)
        self._watch(goal, f"jog {name} → {target:.3f}", [name])
        return target

    async def jog_start(self, name: str, direction: int, speed: float = 0.5) -> None:
        """Continuous jog: one goal towards the joint limit at jog speed; jog_stop() ramps it down."""
        robot = self._require_robot()
        j = self.joint(name)
        for g in self.running_goals([name]):      # this joint only; other joints keep moving
            await g.cancel()
        now = (await robot.current_positions()).get(name, 0.0)
        far = (j.upper if direction > 0 else j.lower)
        if far is None:
            far = now + direction * (2 * math.pi if j.type != "prismatic" else 1.0)
        target = self._clamp(j, far)
        if abs(target - now) < 1e-6:
            self.log(f"{name} is at its {'upper' if direction > 0 else 'lower'} limit")
            return
        self._jog_targets.pop(name, None)
        goal = await robot.execute([name], [([target], round(self._duration(j, target - now, speed), 3))],
                                   report="none", on_busy=self._on_busy)
        self._continuous = goal
        self._watch(goal, f"jog {name}", [name])

    async def jog_stop(self) -> None:
        goal = getattr(self, "_continuous", None)
        if goal is not None and not goal.done:
            await goal.cancel()
        self._continuous = None

    async def move_joints(self, positions: Mapping[str, float], duration: Optional[float] = None,
                          speed: float = 0.5, label: str = "move") -> GoalHandle:
        """Timed move of several joints; without duration, the slowest joint sets it (at `speed` of max)."""
        robot = self._require_robot()
        if duration is None:
            now = await robot.current_positions()
            duration = max([self._duration(self.joint(n), x - now.get(n, x), speed, 1.0)
                            for n, x in positions.items()] or [1.0])
        for n in positions:
            self._jog_targets.pop(n, None)
        goal = await robot.move_to(dict(positions), duration=round(duration, 3), report="points",
                                   on_busy=self._on_busy)
        return self._watch(goal, label, positions)

    async def _each(self, goals: List[GoalHandle], op: str) -> Dict[str, Any]:
        if not goals:
            raise RuntimeError("nothing is moving")
        acks = [await getattr(g, op)() for g in goals]
        failed = [a.get("message") for a in acks if not a.get("ok")]
        return {"ok": not failed, "message": "; ".join(m for m in failed if m) or f"{len(goals)} goal(s)"}

    async def pause(self, joints=None) -> Dict[str, Any]:
        """Pause the running goals (only those moving `joints`, if given)."""
        return await self._each(self.running_goals(joints), "pause")

    async def resume(self, joints=None) -> Dict[str, Any]:
        return await self._each(self.running_goals(joints), "resume")

    async def cancel(self, joints=None) -> Dict[str, Any]:
        return await self._each(self.running_goals(joints), "cancel")

    async def stop_all(self) -> Dict[str, Any]:
        self._jog_targets.clear()
        return await self._require_robot().stop()

    # ── joint groups ─────────────────────────────────────────────────────────

    def groups(self) -> List[Dict[str, Any]]:
        """Joint groups: the saved ones, then suggestions (saved chains' joints, 'L' / 'R' name prefixes and their
        grippers) under names not used yet. Each: {"name", "joints", "builtin"}."""
        robot = self.robot
        if robot is None:
            return []
        names = robot.joint_names
        out: List[Dict[str, Any]] = []
        # Saved groups first; their names win over suggestions with the same name.
        saved = robot.joint_groups() if self.data_dir is not None else {}
        for n, js in saved.items():
            out.append({"name": n, "joints": [j for j in js if j in names], "builtin": False})
        taken = set(saved)

        def suggest(name: str, joints: List[str]) -> None:
            # skip empty groups and "groups" that are the whole robot (that is the All joints entry)
            if name not in taken and joints and len(joints) < len(names):
                taken.add(name)
                out.append({"name": name, "joints": joints, "builtin": True})

        # Short name prefixes (L_, R_, LA_ …) usually mark a side or an arm; finger / grip / jaw joints in it
        # become that side's gripper group, the rest its arm group.
        prefixes = sorted({n.split("_")[0] for n in names if "_" in n and len(n.split("_")[0]) <= 3})
        for pre in prefixes:
            members = [n for n in names if n.startswith(pre + "_")]
            grip = [n for n in members if any(k in n.lower() for k in ("finger", "grip", "jaw"))]
            suggest(f"{pre} arm" if grip else pre, [n for n in members if n not in grip] or members)
            suggest(f"{pre} gripper", grip)
        # Each saved kinematic chain suggests a group of its movable joints.
        try:
            for cname, c in sorted(self.chains().items()):
                suggest(cname, self.chain(c["end"], c["origin"], cname).joint_names)
        except Exception:
            pass
        return out

    def group_joints(self, group: Optional[str]) -> Optional[List[str]]:
        """Joints of a group (None = all joints)."""
        if not group:
            return None
        g = next((g for g in self.groups() if g["name"] == group), None)
        if g is None:
            raise KeyError(f"no group '{group}'")
        return list(g["joints"])

    def save_group(self, name: str, joints) -> Path:
        return self._require_robot().save_joint_group(name, list(joints))

    def delete_group(self, name: str) -> None:
        self._require_robot().delete_joint_group(name)

    # ── description and poses ───────────────────────────────────────────────

    async def describe_and_save(self) -> Path:
        path, desc = await self._require_robot().save_description()
        self.description = desc
        self._emit(self.on_robot_changed)
        return path

    HOME = "home"

    def home_positions(self, group: Optional[str] = None) -> Dict[str, float]:
        """The built-in home pose: every joint (of `group`) at 0, or at the limit nearest to 0."""
        robot = self._require_robot()
        joints = self.group_joints(group)
        return {j.name: self._clamp(j, 0.0) for j in robot.joints if joints is None or j.name in joints}

    async def go_home(self, group: Optional[str] = None, speed: float = 0.5) -> GoalHandle:
        """Home a group (built-in zeros, or the joints of a saved pose "home" that belong to the group)."""
        robot = self._require_robot()
        joints = self.group_joints(group)
        if self.has_saved_pose(self.HOME):
            home = robot.get_pose(self.HOME)
            positions = {n: x for n, x in home.items() if joints is None or n in joints}
        else:
            positions = self.home_positions(group)
        if not positions:
            raise RuntimeError(f"the home pose has no joints of group '{group}'")
        return await self.move_joints(positions, speed=speed, label=f"home {group}" if group else "home")

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
                            "group": p.get("group") or "", "builtin": False})
        if not any(p["name"] == self.HOME for p in out):
            out.insert(0, {"name": self.HOME, "joints": len(robot.joints), "saved_at": "built-in: all joints 0",
                           "group": "", "builtin": True})
        return out

    async def save_pose(self, name: str, group: Optional[str] = None) -> Path:
        """Save the current positions of all joints, or only of `group`'s joints (a group pose)."""
        return await self._require_robot().save_pose(name, joints=self.group_joints(group), group=group)

    def delete_pose(self, name: str) -> None:
        if name == self.HOME and not self.has_saved_pose(name):
            raise ValueError("the built-in home pose cannot be deleted (save a pose named 'home' to replace it)")
        self._require_robot().delete_pose(name)

    async def go_to_pose(self, name: str, speed: float = 0.5) -> GoalHandle:
        """Move to a pose; a group pose moves only its group's joints, in parallel with other groups."""
        robot = self._require_robot()
        if name == self.HOME and not self.has_saved_pose(name):
            positions = self.home_positions()
        else:
            positions = robot.get_pose(name)
        return await self.move_joints(positions, speed=speed, label=f"pose '{name}'")

    # ── chains, TCP, frames (Cartesian) ──────────────────────────────────────

    TCP_FILE = "tcp.json"          # TCP per end link, for chains that were not saved under a name
    FRAMES_FILE = "frames.json"
    CHAINS_FILE = "chains.json"    # named chains: origin, end and their own TCP

    def _tree(self) -> Dict[str, Any]:
        if not self.description or not self.description.get("joints"):
            raise RuntimeError("the robot sent no kinematic tree (Robot → Describe & save to retry)")
        return self.description

    def _doc(self, name: str, key: str) -> Dict[str, Any]:
        robot = self.robot
        if robot is None or self.data_dir is None:
            return {}
        return dict(robot.store.load_doc(name).get(key, {}))

    def _save_doc(self, name: str, key: str, value: Mapping[str, Any]) -> Path:
        robot = self._require_robot()
        if self.data_dir is None:
            raise RuntimeError('no data folder - set "data_dir" in remote_control.json')
        return robot.store.save_doc(name, {key: dict(sorted(value.items()))})

    # tool centre point (per chain end link)

    def tcp(self, end: str):
        """(xyz, rpy) of the TCP in the end link's frame; zero if none was set."""
        t = self._doc(self.TCP_FILE, "tcp").get(end) or {}
        return tuple(t.get("xyz", (0.0, 0.0, 0.0))), tuple(t.get("rpy", (0.0, 0.0, 0.0)))

    def set_tcp(self, end: str, xyz, rpy) -> Path:
        tcps = self._doc(self.TCP_FILE, "tcp")
        if all(abs(v) < 1e-12 for v in list(xyz) + list(rpy)):
            tcps.pop(end, None)
        else:
            tcps[end] = {"xyz": [float(v) for v in xyz], "rpy": [float(v) for v in rpy]}
        return self._save_doc(self.TCP_FILE, "tcp", tcps)

    # named chains

    def chains(self) -> Dict[str, Dict[str, Any]]:
        """Saved chains: name → {"origin", "end", "tcp": {"xyz", "rpy"}, "saved_at"}."""
        return self._doc(self.CHAINS_FILE, "chains")

    def save_chain(self, name: str, origin: str, end: str) -> Path:
        """Save (or replace) chain `name` from `origin` to `end` and give it a TCP: kept if the chain already
        ended at the same link, else taken from an earlier TCP for that end link, else at the end link's origin."""
        name = name.strip()
        if not name:
            raise ValueError("chain name must not be empty")
        tree = self._tree()
        Chain.path(tree, origin, end)          # raises if origin is not above end
        chains = self.chains()
        old = chains.get(name)
        if old is not None and old.get("end") == end and old.get("tcp"):
            tcp = old["tcp"]
        else:
            xyz, rpy = self.tcp(end)
            tcp = {"xyz": list(xyz), "rpy": list(rpy)}
        chains[name] = {"origin": origin, "end": end, "tcp": tcp, "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
        return self._save_doc(self.CHAINS_FILE, "chains", chains)

    def delete_chain(self, name: str) -> None:
        chains = self.chains()
        if name not in chains:
            raise KeyError(f"no chain '{name}'")
        del chains[name]
        self._save_doc(self.CHAINS_FILE, "chains", chains)

    def set_chain_tcp(self, name: str, xyz, rpy) -> Path:
        chains = self.chains()
        if name not in chains:
            raise KeyError(f"no chain '{name}' - save the chain first")
        chains[name]["tcp"] = {"xyz": [float(v) for v in xyz], "rpy": [float(v) for v in rpy]}
        return self._save_doc(self.CHAINS_FILE, "chains", chains)

    def tcp_for(self, end: str, chain_name: Optional[str] = None):
        """TCP (xyz, rpy) in the end link: the named chain's, else the end link's (unsaved chains)."""
        c = self.chains().get(chain_name) if chain_name else None
        if c is not None and c.get("end") == end and c.get("tcp"):
            return tuple(c["tcp"].get("xyz", (0, 0, 0))), tuple(c["tcp"].get("rpy", (0, 0, 0)))
        return self.tcp(end)

    def chain(self, tool: str, base: Optional[str] = None, chain_name: Optional[str] = None) -> Chain:
        """Chain from base to the end link `tool`, ending at its TCP (the named chain's, if given)."""
        xyz, rpy = self.tcp_for(tool, chain_name)
        return Chain.from_tree(self._tree(), tool, base, tcp=pose_from_xyz_rpy(xyz, rpy))

    def tools(self) -> List[str]:
        """End links to offer: chain ends first (last rotating joint of each branch), then every other link."""
        if not self.description:
            return []
        ends = Chain.tool_candidates(self.description)
        root = self.description.get("root")
        rest = [l for l in Chain.links(self.description) if l not in ends and l != root]
        return ends + rest

    def bases_for(self, tool: str) -> List[str]:
        """Links a chain to `tool` can start at: its ancestors (root first)."""
        if not self.description:
            return []
        return list(reversed(Chain.ancestors(self.description, tool)[1:]))

    def chain_links(self, tool: str, base: Optional[str]) -> List[str]:
        tree = self._tree()
        return Chain.path(tree, base or tree.get("root"), tool)

    def tool_pose(self, tool: str, base: Optional[str] = None, chain_name: Optional[str] = None):
        """(xyz, rpy) of the TCP in `base` at the latest measured positions."""
        return xyz_rpy_from_pose(self.chain(tool, base, chain_name).tool_pose(self.positions))

    # link poses (forward kinematics over the whole tree)

    def _link_pose(self, link: str) -> np.ndarray:
        """4x4 pose of a link in the tree root's frame at the latest positions."""
        poses = forward_kinematics(self._tree(), self.positions)
        if link not in poses:
            raise KeyError(f"unknown link '{link}'")
        p = poses[link]
        x, y, z, w = p["orientation"]
        rot = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        t = np.eye(4)
        t[:3, :3] = rot
        t[:3, 3] = p["position"]
        return t

    def express(self, xyz, rpy, parent: str, base: str):
        """A pose given in `parent` expressed in `base` (both links of the robot)."""
        t = np.linalg.inv(self._link_pose(base)) @ self._link_pose(parent) @ pose_from_xyz_rpy(xyz, rpy)
        return xyz_rpy_from_pose(t)

    # named frames

    def frames(self) -> Dict[str, Dict[str, Any]]:
        return self._doc(self.FRAMES_FILE, "frames")

    def save_frame(self, name: str, parent: str, xyz, rpy) -> Path:
        name = name.strip()
        if not name:
            raise ValueError("frame name must not be empty")
        frames = self.frames()
        frames[name] = {"parent": parent, "xyz": [float(v) for v in xyz], "rpy": [float(v) for v in rpy],
                        "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
        return self._save_doc(self.FRAMES_FILE, "frames", frames)

    def frame_from_tcp(self, name: str, tool: str, base: Optional[str], chain_name: Optional[str] = None) -> Path:
        """Save the TCP's current pose as frame `name`, relative to the chain origin `base`."""
        base = base or self._tree().get("root")
        xyz, rpy = self.tool_pose(tool, base, chain_name)
        return self.save_frame(name, base, xyz, rpy)

    def delete_frame(self, name: str) -> None:
        frames = self.frames()
        if name not in frames:
            raise KeyError(f"no frame '{name}'")
        del frames[name]
        self._save_doc(self.FRAMES_FILE, "frames", frames)

    def frame_in(self, name: str, base: str):
        """(xyz, rpy) of frame `name` in `base`."""
        f = self.frames()[name]
        return self.express(f["xyz"], f["rpy"], f["parent"], base)

    # targets

    def check_target(self, tool: str, base: Optional[str], xyz, rpy, position_only: bool,
                     chain_name: Optional[str] = None) -> IkResult:
        chain = self.chain(tool, base, chain_name)
        target = pose_from_xyz_rpy(xyz, rpy)
        result = chain.solve(target, self.positions, position_only=position_only)
        if not result.reachable and not position_only:
            # tell the user which part fails: the position, or only the orientation
            result.position_reachable = chain.solve(target, self.positions, position_only=True, restarts=4).reachable
        return result

    async def check_target_async(self, tool: str, base: Optional[str], xyz, rpy, position_only: bool,
                                 chain_name: Optional[str] = None) -> IkResult:
        """check_target on a worker thread (an unreachable target tries several starts)."""
        return await asyncio.get_event_loop().run_in_executor(
            None, lambda: self.check_target(tool, base, xyz, rpy, position_only, chain_name))

    async def move_to_target(self, tool: str, base: Optional[str], xyz, rpy, position_only: bool,
                             duration: Optional[float] = None, speed: float = 0.5,
                             chain_name: Optional[str] = None) -> IkResult:
        result = await self.check_target_async(tool, base, xyz, rpy, position_only, chain_name)
        if not result.reachable:
            return result
        await self.move_joints(result.positions, duration, speed, label=f"target {chain_name or tool}")
        return result

    # target references and scene-owned targets
    #
    # A target pose can be given in three references: "chain" (the chain origin link), "robot" (the robot
    # instance in its scene) or "scene" (the scene / world). Robots with supports.targets (Unity) own their targets
    # as scene objects and report them in describe replies; other robots use the frames stored here (frames.json).

    REFERENCES = ("chain", "robot", "scene")

    @property
    def uses_scene_targets(self) -> bool:
        return self.robot is not None and bool(self.robot.supports.get("targets"))

    def _live_pose(self, key: str) -> np.ndarray:
        pose = self.live.get(key) or self.live.get("base_pose") or {}
        return pose_from_xyz_rpy(*pose_to_xyz_rpy(pose)) if pose else np.eye(4)

    def to_root(self, xyz, rpy, reference: str, origin: str) -> np.ndarray:
        """4x4 pose in the tree root link's frame, from xyz / rpy given in `reference`."""
        t = pose_from_xyz_rpy(xyz, rpy)
        if reference == "chain":
            return self._link_pose(origin) @ t
        base = self._live_pose("base_pose")
        if reference == "robot":
            return np.linalg.inv(base) @ self._live_pose("robot_pose") @ t
        if reference == "scene":
            return np.linalg.inv(base) @ t
        raise ValueError(f"unknown reference '{reference}'")

    def from_root(self, t: np.ndarray, reference: str, origin: str):
        """(xyz, rpy) in `reference` of a 4x4 pose given in the root link's frame."""
        if reference == "chain":
            return xyz_rpy_from_pose(np.linalg.inv(self._link_pose(origin)) @ t)
        base = self._live_pose("base_pose")
        if reference == "robot":
            return xyz_rpy_from_pose(np.linalg.inv(self._live_pose("robot_pose")) @ base @ t)
        if reference == "scene":
            return xyz_rpy_from_pose(base @ t)
        raise ValueError(f"unknown reference '{reference}'")

    def convert(self, xyz, rpy, src: str, dst: str, origin: str):
        """A pose given in reference `src`, expressed in `dst` (chain references use the chain origin `origin`)."""
        return self.from_root(self.to_root(xyz, rpy, src, origin), dst, origin)

    def scene_targets(self) -> Dict[str, Dict[str, Any]]:
        """Targets the robot's scene reported: id → {"name", "path", "parent", "pose_in_root", …}."""
        return {t["id"]: t for t in self.live.get("targets", []) if t.get("id")}

    def targets_list(self) -> List[Tuple[str, str]]:
        """(id, display name) of every target: the scene's (Unity) or the frames stored here."""
        if self.uses_scene_targets:
            out = []
            for tid, t in self.scene_targets().items():
                path = t.get("path") or t.get("name") or tid
                out.append((tid, path[len("Targets/"):] if path.startswith("Targets/") else path))
            return sorted(out, key=lambda x: x[1].lower())
        return [(f"frame:{name}", name) for name in sorted(self.frames())]

    def target_pose(self, target_id: str, reference: str, origin: str):
        """(xyz, rpy) of a target in `reference`."""
        if target_id.startswith("frame:"):
            xyz, rpy = self.frame_in(target_id[len("frame:"):], origin)
            return self.convert(xyz, rpy, "chain", reference, origin)
        t = self.scene_targets().get(target_id)
        if t is None:
            raise KeyError(f"no target '{target_id}'")
        root = pose_from_xyz_rpy(*pose_to_xyz_rpy(t["pose_in_root"]))
        return self.from_root(root, reference, origin)

    @staticmethod
    def _reference_for_robot(reference: str, origin: str) -> str:
        return f"link:{origin}" if reference == "chain" else reference

    async def create_target(self, name: str, reference: str, xyz, rpy, origin: str,
                            parent: Optional[str] = None) -> str:
        """New target `name` at a pose given in `reference`. Scene-owned (Unity: under "Targets", or `parent`, a
        scene object path) when the robot supports it, else a frame stored here. Returns the target id."""
        if self.uses_scene_targets:
            req: Dict[str, Any] = {"op": "create", "name": name, "pose": _pose(xyz, rpy),
                                   "reference": self._reference_for_robot(reference, origin)}
            if parent:
                req["parent"] = parent
            await self.robot.target(req)  # type: ignore[union-attr]
            path = f"{parent or 'Targets'}/{name}"
            return f"target:{path}"
        cxyz, crpy = self.convert(xyz, rpy, reference, "chain", origin)
        self.save_frame(name, origin, cxyz, crpy)
        return f"frame:{name}"

    async def update_target(self, target_id: str, reference: str, xyz, rpy, origin: str) -> None:
        if target_id.startswith("frame:"):
            name = target_id[len("frame:"):]
            cxyz, crpy = self.convert(xyz, rpy, reference, "chain", origin)
            self.save_frame(name, origin, cxyz, crpy)
            return
        await self.robot.target({"op": "update", "id": target_id, "pose": _pose(xyz, rpy),  # type: ignore[union-attr]
                                 "reference": self._reference_for_robot(reference, origin)})

    async def delete_target(self, target_id: str) -> None:
        if target_id.startswith("frame:"):
            self.delete_frame(target_id[len("frame:"):])
            return
        await self.robot.target({"op": "delete", "id": target_id})  # type: ignore[union-attr]

    async def select_target_in_viewer(self, target_id: str) -> None:
        if self.uses_scene_targets and target_id.startswith("target:"):
            await self.robot.target({"op": "select", "id": target_id})  # type: ignore[union-attr]

    async def set_attach_new_targets(self, attach: bool) -> None:
        if self.uses_scene_targets:
            await self.robot.target({"op": "settings", "attach": bool(attach)})  # type: ignore[union-attr]

    CLICK_ORIENTATIONS = ("approach", "surface", "tcp")

    async def set_click_orientation(self, mode: str) -> None:
        """Orientation of targets the user creates by Ctrl+click: approach (z into the surface, x towards the
        robot), surface (z out of the surface) or tcp (the TCP's current orientation)."""
        if mode not in self.CLICK_ORIENTATIONS:
            raise ValueError(f"unknown click orientation '{mode}'")
        if self.uses_scene_targets:
            await self.robot.target({"op": "settings", "orientation": mode})  # type: ignore[union-attr]

    def suggest_target_name(self, prefix: str) -> str:
        """'<prefix>_<n>' with the lowest n not used by a target: "L_claw_1", "target_3"."""
        base = "".join(c if c.isalnum() or c in "-_" else "_" for c in (prefix or "").strip())
        while "__" in base:
            base = base.replace("__", "_")
        base = base.strip("_") or "target"
        names = {name.split("/")[-1] for _, name in self.targets_list()}
        n = 1
        while f"{base}_{n}" in names:
            n += 1
        return f"{base}_{n}"

    # markers in the robot's viewer (Unity …)

    @property
    def can_visualize(self) -> bool:
        return self.robot is not None and bool(self.robot.supports.get("visualize"))

    def marker_items(self, tool: str, base: Optional[str], selected: Optional[str] = None,
                     target=None, chain_name: Optional[str] = None,
                     focus: Optional[str] = None) -> List[Dict[str, Any]]:
        """Chain line, TCP frame, the saved frames (selectable) and an unsaved target, as `visualize` items."""
        tree = self._tree()
        base = base or tree.get("root")
        tcp_xyz, tcp_rpy = self.tcp_for(tool, chain_name)
        key = chain_name or tool
        tcp_id = f"tcp:{key}"
        items: List[Dict[str, Any]] = [
            {"id": tcp_id, "kind": "frame", "parent": tool, "pose": _pose(tcp_xyz, tcp_rpy),
             "name": object_name("TCP", key), "label": f"TCP {key}", "style": "tcp", "size": 0.08,
             "selectable": False, "editable": True},
            {"id": f"chain:{key}", "kind": "chain", "links": self.chain_links(tool, base), "end": tcp_id,
             "name": object_name("Chain", key), "label": chain_name or f"{base} → {tool}", "style": "chain"},
            {"id": f"origin:{base}", "kind": "frame", "parent": base, "pose": _pose((0, 0, 0), (0, 0, 0)),
             "name": object_name("Origin", base), "label": f"origin {base}", "style": "frame", "size": 0.12,
             "selectable": False},
        ]
        for name, f in ([] if self.uses_scene_targets else sorted(self.frames().items())):
            items.append({"id": f"frame:{name}", "kind": "frame", "parent": f["parent"],
                          "pose": _pose(f["xyz"], f["rpy"]), "name": object_name("Frame", name), "label": name,
                          "style": "target" if selected == name else "frame", "selectable": True,
                          "editable": True, "selected": selected == name})
        if target is not None and selected is None:
            xyz, rpy = target
            items.append({"id": "target", "kind": "frame", "parent": base, "pose": _pose(xyz, rpy),
                          "name": "Target", "label": "target", "style": "target", "selectable": False,
                          "selected": True})
        for item in items:
            if item["id"] == focus:
                item["focus"] = True     # the viewer selects it for editing (Unity: Scene view, Move tool)
        return items

    # edits made in the viewer (the user dragged a TCP / frame)

    def apply_edit(self, payload: Mapping[str, Any], tool: str, chain_name: Optional[str]) -> Optional[str]:
        """Store a pose the user set in the viewer. Returns what changed ("tcp" / "frame:<name>") or None."""
        item_id = str(payload.get("id") or "")
        pose = payload.get("pose") or {}
        xyz, rpy = pose_to_xyz_rpy(pose)
        if item_id == f"tcp:{chain_name or tool}":
            if chain_name and chain_name in self.chains():
                self.set_chain_tcp(chain_name, xyz, rpy)
            else:
                self.set_tcp(tool, xyz, rpy)
            return "tcp"
        if item_id.startswith("frame:"):
            name = item_id[len("frame:"):]
            f = self.frames().get(name)
            if f is None:
                return None
            parent = payload.get("parent") or f["parent"]
            self.save_frame(name, parent, xyz, rpy)
            return item_id
        return None

    async def show_markers(self, tool: str, base: Optional[str], selected: Optional[str] = None,
                           target=None, chain_name: Optional[str] = None, focus: Optional[str] = None) -> bool:
        """Send the markers to the robot's viewer; False if the robot has none. `focus`: item to select for editing."""
        if not self.can_visualize or not tool:
            return False
        # Right after switching robots the tabs may still name the previous robot's links: never send a chain this
        # robot does not have (its viewer would reject it); the tabs redraw once its description has arrived.
        links = set(Chain.links(self.description)) if self.description else set()
        if tool not in links or (base and base not in links):
            return False
        items = self.marker_items(tool, base, selected, target, chain_name, focus)
        await self.robot.visualize(items)  # type: ignore[union-attr]
        return True

    async def clear_markers(self) -> None:
        if self.can_visualize:
            await self.robot.visualize([])  # type: ignore[union-attr]


def object_name(kind: str, name: str) -> str:
    """Name for a marker object in the viewer's scene: "TCP_right_arm", "Frame_pick"."""
    slug = "".join(c if c.isalnum() or c in "-_" else "_" for c in str(name).strip())
    while "__" in slug:
        slug = slug.replace("__", "_")
    return f"{kind}_{slug.strip('_') or 'unnamed'}"


def pose_to_xyz_rpy(pose: Mapping[str, Any]):
    """(xyz, rpy) from {"position", "orientation" (ROS quaternion x y z w)}."""
    x, y, z, w = pose.get("orientation") or (0.0, 0.0, 0.0, 1.0)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    rot = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    t = np.eye(4)
    t[:3, :3] = rot
    t[:3, 3] = pose.get("position") or (0.0, 0.0, 0.0)
    return xyz_rpy_from_pose(t)


def _pose(xyz, rpy) -> Dict[str, List[float]]:
    """{"position", "orientation"} (ROS quaternion) from xyz + URDF rpy."""
    r = rpy_matrix(*rpy)
    w = math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    if w > 1e-6:
        q = [(r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w), w]
    else:   # 180° rotation: use the largest diagonal element
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        v = math.sqrt(max(0.0, 1 + r[i, i] - r[j, j] - r[k, k])) / 2
        q = [0.0, 0.0, 0.0, (r[k, j] - r[j, k]) / (4 * v)]
        q[i] = v
        q[j] = (r[j, i] + r[i, j]) / (4 * v)
        q[k] = (r[k, i] + r[i, k]) / (4 * v)
    return {"position": [float(v) for v in xyz], "orientation": [float(v) for v in q]}
