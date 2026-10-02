"""
Controller-side API: the code that decides what robots do uses only this module.

    controller = MotionController(controller_transport_from_url("ws://0.0.0.0:8765/motion"))
    await controller.start()
    robot = await controller.wait_for_robot("arm-01")
    goal = await robot.execute(["shoulder", "elbow"],
                               [([0.0, 0.5], 2.0), ([0.3, 0.8], 4.5)], report="all")
    goal.on("point_reached", lambda p: print(p))
    await goal.pause(); await goal.resume()
    print(await goal.result())
"""
from __future__ import annotations

import asyncio
import logging
import time
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple, Union

from .protocol import (CONTROL_TYPES, Envelope, GoalStatus, MsgType, Sequencer, SeqTracker,
                       channel_of, new_goal_id)
from .trajectory import Joint
from .transports.base import ControllerTransport, Link

log = logging.getLogger(__name__)

PointLike = Union[Dict[str, Any], Tuple[Sequence[float], float]]
Listener = Callable[[Dict[str, Any]], Any]


class GoalRejected(Exception):
    def __init__(self, goal_id: str, reason: str) -> None:
        super().__init__(f"goal {goal_id} rejected: {reason}")
        self.goal_id, self.reason = goal_id, reason


class RobotOffline(ConnectionError):
    pass


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
    """events: online, offline, state, and '*' for all."""

    def __init__(self, controller: "MotionController", robot_id: str) -> None:
        super().__init__()
        self.controller = controller
        self.robot_id = robot_id
        self.name = robot_id
        self.joints: List[Joint] = []
        self.supports: Dict[str, Any] = {}
        self.state: Dict[str, Any] = {}
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
        await self._send(MsgType.EXECUTE, {
            "joint_names": list(joint_names), "points": pts, "report": report,
            "progress_hz": progress_hz, "on_busy": on_busy, "interpolation": interpolation,
        }, goal.goal_id)
        accepted = await asyncio.wait_for(asyncio.shield(goal._decided), timeout)
        if not accepted:
            raise GoalRejected(goal.goal_id, goal.reason or "")
        return goal

    async def stop(self) -> Dict[str, Any]:
        return await self._control(MsgType.STOP, None)

    # ── internals ────────────────────────────────────────────────────────────

    async def _send(self, msg_type: str, payload: Dict[str, Any], goal_id: Optional[str] = None) -> Envelope:
        if not self.online or self._link is None or self._seq is None:
            raise RobotOffline(f"robot {self.robot_id} is offline")
        env = self._seq.stamp(Envelope(msg_type, self.robot_id, payload, goal_id))
        await self._link.send(env, channel_of(msg_type, from_robot=False))
        return env

    async def _control(self, msg_type: str, goal_id: Optional[str], timeout: float = 5.0) -> Dict[str, Any]:
        assert msg_type in CONTROL_TYPES
        fut = asyncio.get_event_loop().create_future()
        env = await self._send(msg_type, {}, goal_id)
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
        if env.type == MsgType.ACK:
            fut = self._acks.get(int(env.payload.get("ref_seq", -1)))
            if fut and not fut.done():
                fut.set_result(env.payload)
        elif env.type == MsgType.STATE:
            self.state = env.payload
            self._fire("state", env.payload)
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
    """events: robot_online, robot_offline (payload: {"robot_id"})."""

    def __init__(self, transport: ControllerTransport, heartbeat_interval: float = 0.5,
                 heartbeat_timeout: float = 2.0) -> None:
        super().__init__()
        self.transport = transport
        self.heartbeat_interval = heartbeat_interval
        self.heartbeat_timeout = heartbeat_timeout
        self.robots: Dict[str, RobotHandle] = {}
        self._links: Dict[str, _LinkInfo] = {}
        self._hb_task: Optional["asyncio.Future"] = None
        self._robot_online: Optional[asyncio.Condition] = None  # created in start() (needs a running loop on 3.8)
        transport.on_link_open = self._on_open
        transport.on_link_closed = self._on_closed
        transport.on_message = self._on_message

    async def start(self) -> None:
        self._robot_online = asyncio.Condition()
        await self.transport.start()
        self._hb_task = asyncio.ensure_future(self._heartbeat_loop())

    async def stop(self) -> None:
        if self._hb_task:
            self._hb_task.cancel()
            try:
                await self._hb_task
            except asyncio.CancelledError:
                pass
        await self.transport.stop()

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

    # ── transport events ─────────────────────────────────────────────────────

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
