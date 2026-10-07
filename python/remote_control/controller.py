"""
Controller-side API: the code that decides what robots do uses only this module.

    controller = MotionController(controller_connector_from_url("ws://0.0.0.0:8765/motion"))
    # or: MotionController(controller_connector_from_config(load_config("controller.json")["connector"]))
    await controller.start()
    robot = await controller.wait_for_robot("arm-01")
    goal = await robot.execute(["shoulder", "elbow"],
                               [([0.0, 0.5], 2.0), ([0.3, 0.8], 4.5)], report="all")
    goal.on("point_reached", lambda p: print(p))
    await goal.pause(); await goal.resume()
    print(await goal.result())

Robot data (needs MotionController(..., data_dir=...)), filed under <data_dir>/<project>/<stage>/<robot_id>:

    path, desc = await robot.save_description()      # names, base location, kinematic tree, positions
    await robot.save_pose("ready")                    # current joint positions
    goal = await robot.move_to_pose("ready")          # timed from the joints' max_velocity
    robot.list_poses(); robot.delete_pose("ready")
"""
from __future__ import annotations

import asyncio
import logging
import time
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple, Union

from .protocol import (CONTROL_TYPES, Envelope, GoalStatus, MsgType, Sequencer, SeqTracker,
                       channel_of, new_goal_id)
from .trajectory import Joint
from .connectors.base import ControllerConnector, Link
from .connectors.config import ConfigError
from .data import DEFAULT_NAME, RobotStore

log = logging.getLogger(__name__)

PointLike = Union[Dict[str, Any], Tuple[Sequence[float], float]]
Listener = Callable[[Dict[str, Any]], Any]


class GoalRejected(Exception):
    def __init__(self, goal_id: str, reason: str) -> None:
        super().__init__(f"goal {goal_id} rejected: {reason}")
        self.goal_id, self.reason = goal_id, reason


class RobotOffline(ConnectionError):
    pass


class RobotError(RuntimeError):
    """The robot answered a request with ok=false, or does not support it."""


class _Events:
    def __init__(self) -> None:
        self._listeners: Dict[str, List[Listener]] = {}

    def on(self, event: str, fn: Listener) -> None:
        self._listeners.setdefault(event, []).append(fn)

    def _fire(self, event: str, payload: Dict[str, Any]) -> None:
        calls = [(fn, payload) for fn in self._listeners.get(event, [])]
        calls += [(fn, {"event": event, **payload}) for fn in self._listeners.get("*", [])]
        for fn, arg in calls:
            try:
                r = fn(arg)
                if asyncio.iscoroutine(r):
                    asyncio.ensure_future(r)
            except Exception:
                log.exception("listener for %s failed", event)


class GoalHandle(_Events):
    """events: accepted, rejected, point_reached, feedback, result, and '*' for all."""

    def __init__(self, robot: "RobotHandle", goal_id: str) -> None:
        super().__init__()
        self.robot = robot
        self.goal_id = goal_id
        self.status = "pending"
        self.queue_position: Optional[int] = None
        self.reason: Optional[str] = None
        self.points_reached: List[Dict[str, Any]] = []
        self.last_feedback: Optional[Dict[str, Any]] = None
        self.joint_names: List[str] = []
        self._decided: "asyncio.Future" = asyncio.get_event_loop().create_future()
        self._result: "asyncio.Future" = asyncio.get_event_loop().create_future()

    @property
    def done(self) -> bool:
        return self._result.done()

    async def pause(self) -> Dict[str, Any]:
        return await self.robot._control(MsgType.PAUSE, self.goal_id)

    async def resume(self) -> Dict[str, Any]:
        return await self.robot._control(MsgType.RESUME, self.goal_id)

    async def cancel(self) -> Dict[str, Any]:
        return await self.robot._control(MsgType.CANCEL, self.goal_id)

    async def result(self, timeout: Optional[float] = None) -> Dict[str, Any]:
        return await asyncio.wait_for(asyncio.shield(self._result), timeout)

    async def send(self, positions: Union[Sequence[float], Mapping[str, float]], end: bool = False) -> None:
        """Stream goals: the newest pose (in joint_names order, or by name - names left out keep their last value).
        The robot follows it within max_velocity. end=True: no more poses; the goal succeeds once there."""
        if isinstance(positions, Mapping):
            last = self._last_sent or [0.0] * len(self.joint_names)
            positions = [float(positions.get(n, x)) for n, x in zip(self.joint_names, last)]
        self._last_sent = [float(x) for x in positions]
        await self.robot._send(MsgType.STREAM, {"positions": self._last_sent, **({"end": True} if end else {})},
                               self.goal_id)

    async def end(self) -> None:
        """Stream goals: finish at the last pose sent."""
        await self.robot._send(MsgType.STREAM, {"end": True}, self.goal_id)

    _last_sent: Optional[List[float]] = None

    def _on(self, env: Envelope) -> None:
        p = env.payload
        if env.type == MsgType.ACCEPTED:
            self.status = "accepted"
            self.queue_position = p.get("queue_position")
            if not self._decided.done():
                self._decided.set_result(True)
        elif env.type == MsgType.REJECTED:
            self.status = "rejected"
            self.reason = p.get("reason", "")
            if not self._decided.done():
                self._decided.set_result(False)
            if not self._result.done():
                self._result.set_result({"status": "rejected", "message": self.reason})
        elif env.type == MsgType.POINT_REACHED:
            self.points_reached.append(p)
        elif env.type == MsgType.FEEDBACK:
            self.last_feedback = p
        elif env.type == MsgType.RESULT:
            self.status = p.get("status", GoalStatus.ABORTED)
            if not self._decided.done():  # result for a goal we never saw accepted (reconnect)
                self._decided.set_result(True)
            if not self._result.done():
                self._result.set_result(p)
        self._fire(env.type, p)


class RobotHandle(_Events):
    """events: online, offline, state, selected (the user picked a visualized item), edited (the user moved an
    editable item: {"id", "parent", "pose"}), and '*' for all."""

    def __init__(self, controller: "MotionController", robot_id: str) -> None:
        super().__init__()
        self.controller = controller
        self.robot_id = robot_id
        self.name = robot_id
        self.joints: List[Joint] = []
        self.supports: Dict[str, Any] = {}
        self.state: Dict[str, Any] = {}
        self.project = DEFAULT_NAME
        self.stage = DEFAULT_NAME
        self.instance: Optional[str] = None
        self.online = False
        self._link: Optional[Link] = None
        self._seq: Optional[Sequencer] = None
        self._goals: Dict[str, GoalHandle] = {}
        self._acks: Dict[int, "asyncio.Future"] = {}

    @property
    def joint_names(self) -> List[str]:
        return [j.name for j in self.joints]

    def goal(self, goal_id: str) -> GoalHandle:
        """Handle for any goal id — including goals listed in `state` after a reconnect."""
        if goal_id not in self._goals:
            self._goals[goal_id] = GoalHandle(self, goal_id)
        return self._goals[goal_id]

    async def execute(self, joint_names: Sequence[str], points: Iterable[PointLike], *,
                      report: str = "points", progress_hz: float = 10.0, on_busy: str = "queue",
                      interpolation: str = "cubic", goal_id: Optional[str] = None,
                      timeout: float = 5.0) -> GoalHandle:
        """Send a goal and wait until the robot accepts it. Raises GoalRejected / RobotOffline."""
        pts = []
        for p in points:
            if isinstance(p, dict):
                pts.append({"positions": list(p["positions"]), "time_from_start": p["time_from_start"]})
            else:
                positions, t = p
                pts.append({"positions": list(positions), "time_from_start": t})
        goal = self.goal(goal_id or new_goal_id())
        goal.joint_names = list(joint_names)
        await self._send(MsgType.EXECUTE, {
            "joint_names": list(joint_names), "points": pts, "report": report,
            "progress_hz": progress_hz, "on_busy": on_busy, "interpolation": interpolation,
        }, goal.goal_id)
        accepted = await asyncio.wait_for(asyncio.shield(goal._decided), timeout)
        if not accepted:
            raise GoalRejected(goal.goal_id, goal.reason or "")
        return goal

    async def stream(self, joint_names: Sequence[str], *, speed: float = 1.0, report: str = "progress",
                     progress_hz: float = 10.0, on_busy: str = "queue", goal_id: Optional[str] = None,
                     timeout: float = 5.0) -> GoalHandle:
        """Start a stream goal on `joint_names`: then call goal.send(positions) as often as poses come (e.g. every
        frame of a motion planner) and goal.end() when done; pause / resume / cancel / stop work as for any goal.
        The robot moves towards the newest pose at up to speed × each joint's max_velocity. Needs supports.stream."""
        if not self.supports.get("stream"):
            raise RobotError(f"robot {self.robot_id} cannot follow streamed poses (supports.stream is false)")
        goal = self.goal(goal_id or new_goal_id())
        goal.joint_names = list(joint_names)
        await self._send(MsgType.EXECUTE, {
            "joint_names": list(joint_names), "stream": True, "speed": speed, "report": report,
            "progress_hz": progress_hz, "on_busy": on_busy,
        }, goal.goal_id)
        accepted = await asyncio.wait_for(asyncio.shield(goal._decided), timeout)
        if not accepted:
            raise GoalRejected(goal.goal_id, goal.reason or "")
        return goal

    async def stop(self) -> Dict[str, Any]:
        return await self._control(MsgType.STOP, None)

    async def visualize(self, items: Sequence[Mapping[str, Any]], replace: bool = True) -> Dict[str, Any]:
        """Show markers (frames, chains) in the robot's viewer; see PROTOCOL.md `visualize`. replace=True
        removes everything shown before; otherwise items with the same id are updated, and an item
        {"id": ..., "remove": true} removes one."""
        if not self.supports.get("visualize"):
            raise RobotError(f"robot {self.robot_id} has no viewer (supports.visualize is false)")
        ack = await self._control(MsgType.VISUALIZE, None, payload={"items": [dict(i) for i in items],
                                                                     "replace": replace})
        if not ack.get("ok", False):
            raise RobotError(ack.get("message") or "visualize failed")
        return ack

    async def target(self, request: Mapping[str, Any]) -> Dict[str, Any]:
        """Create / update / delete / select a target owned by the robot's scene (supports.targets), see
        PROTOCOL.md `target`. Raises RobotError if the robot refuses."""
        if not self.supports.get("targets"):
            raise RobotError(f"robot {self.robot_id} has no scene targets (supports.targets is false)")
        ack = await self._control(MsgType.TARGET, None, payload=dict(request))
        if not ack.get("ok", False):
            raise RobotError(ack.get("message") or "target request failed")
        return ack

    # ── description and saved poses ─────────────────────────────────────────

    @property
    def names(self) -> Dict[str, str]:
        """project / stage / robot as reported by the robot (robot = robot_id)."""
        return {"project": self.project, "stage": self.stage, "robot": self.robot_id}

    async def describe(self, tree: bool = True, timeout: float = 5.0) -> Dict[str, Any]:
        """Ask the robot for its names, base location, joint positions and (tree=True) kinematic tree."""
        if not self.supports.get("describe"):
            raise RobotError(f"robot {self.robot_id} does not support describe (older client?)")
        fut = asyncio.get_event_loop().create_future()
        env = await self._send(MsgType.DESCRIBE, {"tree": tree})
        self._acks[env.seq] = fut
        try:
            reply = await asyncio.wait_for(fut, timeout)
        except asyncio.TimeoutError:
            raise RobotError(f"no description from robot {self.robot_id} within {timeout:g} s") from None
        finally:
            self._acks.pop(env.seq, None)
        if not reply.get("ok", False):
            raise RobotError(reply.get("message") or "describe failed")
        self.project = reply.get("project") or self.project
        self.stage = reply.get("stage") or self.stage
        return {k: v for k, v in reply.items() if k not in ("ref_seq", "ok", "message")}

    @property
    def store(self) -> RobotStore:
        data_dir = self.controller.data_dir
        if data_dir is None:
            raise ConfigError('no data directory - set "data_dir" in remote_control.json')
        return RobotStore(data_dir, self.project, self.stage, self.robot_id)

    async def save_description(self, timeout: float = 5.0) -> Tuple[Path, Dict[str, Any]]:
        """describe() and write it to <data_dir>/<project>/<stage>/<robot>/description.json."""
        desc = await self.describe(tree=True, timeout=timeout)
        return self.store.save_description(desc), desc

    async def current_positions(self) -> Dict[str, float]:
        """Measured positions of all joints, fresh from the robot when it supports describe."""
        if self.supports.get("describe"):
            pos = (await self.describe(tree=False))["positions"]
        else:
            pos = self.state.get("positions") or {}
        return {k: float(v) for k, v in pos.items() if v is not None}

    async def save_pose(self, name: str, joints: Optional[Sequence[str]] = None,
                        group: Optional[str] = None) -> Path:
        """Save the current positions of `joints` (default: all, or the joints of `group`) as pose `name`
        (replaces an existing one). A group pose moves only that group when used."""
        if group and not joints:
            joints = self.joint_groups().get(group)
            if not joints:
                raise RobotError(f"no joint group '{group}'")
        pos = await self.current_positions()
        names = list(joints) if joints else self.joint_names
        missing = [n for n in names if n not in pos]
        if missing:
            raise RobotError(f"no position for joint(s) {', '.join(missing)}")
        return self.store.save_pose(name, names, [pos[n] for n in names], group=group)

    # joint groups (saved in groups.json)

    def joint_groups(self) -> Dict[str, List[str]]:
        """Saved joint groups: name -> joint names."""
        return self.store.groups() if self.controller.data_dir is not None else {}

    def save_joint_group(self, name: str, joints: Sequence[str]) -> Path:
        unknown = [j for j in joints if j not in self.joint_names]
        if unknown:
            raise RobotError(f"unknown joint(s): {', '.join(unknown)}")
        return self.store.save_group(name, joints)

    def delete_joint_group(self, name: str) -> None:
        self.store.delete_group(name)

    @property
    def parallel_on_busy(self) -> str:
        """on_busy for independent group motion: "parallel" if the robot supports it, else "queue"."""
        return "parallel" if self.supports.get("parallel_goals") else "queue"

    async def move_group(self, name: str, positions: Mapping[str, float], duration: Optional[float] = None,
                         **execute_kwargs: Any) -> GoalHandle:
        """Move the joints of group `name` (only those listed in positions must belong to it). Runs in parallel
        with goals on other joints when the robot supports it."""
        joints = self.joint_groups().get(name)
        if joints is None:
            raise RobotError(f"no joint group '{name}'")
        outside = [n for n in positions if n not in joints]
        if outside:
            raise RobotError(f"joint(s) not in group '{name}': {', '.join(outside)}")
        execute_kwargs.setdefault("on_busy", self.parallel_on_busy)
        return await self.move_to(positions, duration, **execute_kwargs)

    def list_poses(self) -> List[str]:
        return self.store.list_poses()

    def get_pose(self, name: str) -> Dict[str, float]:
        return {k: float(v) for k, v in self.store.get_pose(name)["positions"].items()}

    def delete_pose(self, name: str) -> None:
        self.store.delete_pose(name)

    async def move_to(self, positions: Mapping[str, float], duration: Optional[float] = None,
                      min_duration: float = 1.0, **execute_kwargs: Any) -> GoalHandle:
        """One-point goal to `positions` (joint name -> rad / m). Without `duration` it is timed so that every
        joint stays within its max_velocity (the robot checks 1.5 x average speed; 10 % margin on top)."""
        names = list(positions)
        target = [float(positions[n]) for n in names]
        if duration is None:
            now = await self.current_positions()
            limits = {j.name: j.max_velocity for j in self.joints}
            duration = min_duration
            for n, x in zip(names, target):
                vmax = limits.get(n)
                if vmax and n in now:
                    duration = max(duration, 1.5 * abs(x - now[n]) / vmax * 1.1)
        return await self.execute(names, [(target, round(duration, 3))], **execute_kwargs)

    async def move_to_pose(self, name: str, duration: Optional[float] = None, **execute_kwargs: Any) -> GoalHandle:
        return await self.move_to(self.get_pose(name), duration, **execute_kwargs)

    # ── internals ────────────────────────────────────────────────────────────

    async def _send(self, msg_type: str, payload: Dict[str, Any], goal_id: Optional[str] = None) -> Envelope:
        if not self.online or self._link is None or self._seq is None:
            raise RobotOffline(f"robot {self.robot_id} is offline")
        env = self._seq.stamp(Envelope(msg_type, self.robot_id, payload, goal_id))
        await self._link.send(env, channel_of(msg_type, from_robot=False))
        return env

    async def _control(self, msg_type: str, goal_id: Optional[str], timeout: float = 5.0,
                       payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        assert msg_type in CONTROL_TYPES
        fut = asyncio.get_event_loop().create_future()
        env = await self._send(msg_type, payload or {}, goal_id)
        self._acks[env.seq] = fut
        try:
            return await asyncio.wait_for(fut, timeout)
        finally:
            self._acks.pop(env.seq, None)

    def _attach(self, link: Link, seq: Sequencer, hello: Dict[str, Any]) -> None:
        self._link, self._seq = link, seq
        self.name = hello.get("name") or self.robot_id
        self.joints = [Joint.from_dict(j) for j in hello.get("joints", [])]
        self.supports = hello.get("supports", {})
        self.state = hello.get("state", {})
        self.project = hello.get("project") or DEFAULT_NAME
        self.stage = hello.get("stage") or DEFAULT_NAME
        self.instance = hello.get("instance")
        was_online, self.online = self.online, True
        if not was_online:
            self._fire("online", {"robot_id": self.robot_id})
        self._fire("state", self.state)

    def _detach(self) -> None:
        if not self.online:
            return
        self.online = False
        self._link = None
        for fut in self._acks.values():
            if not fut.done():
                fut.set_exception(RobotOffline(f"robot {self.robot_id} went offline"))
        self._acks.clear()
        self._fire("offline", {"robot_id": self.robot_id})

    def _on_message(self, env: Envelope) -> None:
        if env.type in (MsgType.ACK, MsgType.DESCRIPTION):
            fut = self._acks.get(int(env.payload.get("ref_seq", -1)))
            if fut and not fut.done():
                fut.set_result(env.payload)
        elif env.type == MsgType.STATE:
            self.state = env.payload
            self._fire("state", env.payload)
        elif env.type == MsgType.SELECTED:
            self._fire("selected", env.payload)
        elif env.type == MsgType.EDITED:
            self._fire("edited", env.payload)
        elif env.goal_id and env.type in (MsgType.ACCEPTED, MsgType.REJECTED, MsgType.POINT_REACHED,
                                          MsgType.FEEDBACK, MsgType.RESULT):
            self.goal(env.goal_id)._on(env)


class _LinkInfo:
    def __init__(self, link: Link) -> None:
        self.link = link
        self.seq = Sequencer()
        self.rx = SeqTracker()
        self.robot: Optional[RobotHandle] = None
        self.last_rx = time.monotonic()


class MotionController(_Events):
    """events: robot_online, robot_offline (payload: {"robot_id"}), robot_renamed ({"robot_id", "new_id"}:
    a second robot announced an id already online and was asked to use new_id).

    data_dir: where robot descriptions and saved poses go (see data.py); None disables those features."""

    def __init__(self, connector: ControllerConnector, heartbeat_interval: float = 0.5,
                 heartbeat_timeout: float = 2.0, data_dir: Optional[Union[str, Path]] = None) -> None:
        super().__init__()
        self.connector = connector
        self.data_dir = Path(data_dir) if data_dir is not None else None
        self.heartbeat_interval = heartbeat_interval
        self.heartbeat_timeout = heartbeat_timeout
        self.robots: Dict[str, RobotHandle] = {}
        self._links: Dict[str, _LinkInfo] = {}
        self._hb_task: Optional["asyncio.Future"] = None
        self._robot_online: Optional[asyncio.Condition] = None  # created in start() (needs a running loop on 3.8)
        connector.on_link_open = self._on_open
        connector.on_link_closed = self._on_closed
        connector.on_message = self._on_message

    @property
    def transport(self) -> ControllerConnector:   # pre-0.3 name
        return self.connector

    async def start(self) -> None:
        self._robot_online = asyncio.Condition()
        await self.connector.start()
        self._hb_task = asyncio.ensure_future(self._heartbeat_loop())

    async def stop(self) -> None:
        if self._hb_task:
            self._hb_task.cancel()
            try:
                await self._hb_task
            except asyncio.CancelledError:
                pass
        await self.connector.stop()

    async def wait_for_robot(self, robot_id: Optional[str] = None,
                             timeout: Optional[float] = None) -> RobotHandle:
        def ready() -> Optional[RobotHandle]:
            for r in self.robots.values():
                if r.online and (robot_id is None or r.robot_id == robot_id):
                    return r
            return None

        async def wait() -> RobotHandle:
            assert self._robot_online is not None, "call start() first"
            async with self._robot_online:
                await self._robot_online.wait_for(lambda: ready() is not None)
                return ready()  # type: ignore[return-value]

        return await asyncio.wait_for(wait(), timeout)

    # ── connector events ─────────────────────────────────────────────────────

    async def _on_open(self, link: Link) -> None:
        self._links[link.id] = _LinkInfo(link)

    async def _on_closed(self, link: Link) -> None:
        info = self._links.pop(link.id, None)
        if info and info.robot and info.robot._link is link:
            info.robot._detach()
            self._fire("robot_offline", {"robot_id": info.robot.robot_id})

    async def _on_message(self, link: Link, env: Envelope) -> None:
        info = self._links.get(link.id)
        if info is None:
            return
        info.last_rx = time.monotonic()
        if env.type == MsgType.HELLO:
            info.rx.reset()
            info.rx.accept(env.seq)
            info.seq.reset()
            robot = self.robots.get(env.robot_id)
            instance = env.payload.get("instance")
            if (robot is not None and robot.online and robot._link is not link and instance and robot.instance
                    and instance != robot.instance and self._link_alive(robot._link)):
                # a different robot announced an id that is in use: give it a free one
                new_id = self._free_id(env.robot_id)
                log.warning("robot_id '%s' is already online - asking the new robot (instance %s) to use '%s'",
                            env.robot_id, instance, new_id)
                welcome = info.seq.stamp(Envelope(MsgType.WELCOME, env.robot_id, {
                    "heartbeat_interval": self.heartbeat_interval, "heartbeat_timeout": self.heartbeat_timeout,
                    "robot_id": new_id, "instance": instance}))
                await link.send(welcome, channel_of(MsgType.WELCOME, from_robot=False))
                self._fire("robot_renamed", {"robot_id": env.robot_id, "new_id": new_id})
                return
            if robot is None:
                robot = self.robots[env.robot_id] = RobotHandle(self, env.robot_id)
            if robot.online and robot._link is not link:
                robot._detach()  # same robot reconnected on a new link before the old one closed
            rejoined = not robot.online
            info.robot = robot
            robot._attach(link, info.seq, env.payload)
            await robot._send(MsgType.WELCOME, {"heartbeat_interval": self.heartbeat_interval,
                                                "heartbeat_timeout": self.heartbeat_timeout})
            if rejoined:
                self._fire("robot_online", {"robot_id": robot.robot_id})
            if self._robot_online is not None:
                async with self._robot_online:
                    self._robot_online.notify_all()
            return
        if not info.rx.accept(env.seq) or info.robot is None or env.robot_id != info.robot.robot_id:
            return
        if env.type != MsgType.HEARTBEAT:
            info.robot._on_message(env)

    def _link_alive(self, link: Optional[Link]) -> bool:
        info = self._links.get(link.id) if link is not None else None
        return info is not None and time.monotonic() - info.last_rx <= self.heartbeat_timeout

    def _free_id(self, robot_id: str) -> str:
        n = 2
        while f"{robot_id}-{n}" in self.robots and self.robots[f"{robot_id}-{n}"].online:
            n += 1
        return f"{robot_id}-{n}"

    async def _heartbeat_loop(self) -> None:
        while True:
            await asyncio.sleep(self.heartbeat_interval)
            now = time.monotonic()
            for info in list(self._links.values()):
                robot = info.robot
                if robot is None or not robot.online:
                    continue
                if now - info.last_rx > self.heartbeat_timeout:
                    log.warning("robot %s silent for %.1fs — closing link", robot.robot_id, now - info.last_rx)
                    await info.link.close()
                    continue
                try:
                    await robot._send(MsgType.HEARTBEAT, {})
                except RobotOffline:
                    pass
