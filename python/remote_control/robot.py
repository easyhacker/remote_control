"""
Robot-side runtime: binds a RobotConnector + MotionExecutor + JointDriver and runs the tick loop,
heartbeats and the connection watchdog. Used by the fake robot, tests and (later) the Isaac/ROS bridges.
The Unity client (C#) implements the same behaviour in RemoteControlRobot.cs.
"""
from __future__ import annotations

import asyncio
import logging
import time
import uuid
from typing import Any, Callable, Dict, List, Mapping, Optional, Tuple

from .executor import JointDriver, MotionExecutor
from .kinematics import FRAME, IDENTITY_POSE, forward_kinematics
from .protocol import (PROTOCOL_VERSION, Envelope, MsgType, Sequencer, SeqTracker, channel_of)
from .connectors.base import RobotConnector

log = logging.getLogger(__name__)


class RobotRuntime:
    """
    project / stage: names the robot reports (in `hello` and `description`); the controller files this robot's
    data under <data_dir>/<project>/<stage>/<robot_id>. tree: kinematic tree (kinematics.load_urdf_tree) for
    `describe` replies; base_pose: where the tree's root link is in the world. describer(tree) can replace the
    built-in description entirely (simulators with their own scene graph).
    """

    def __init__(self, connector: RobotConnector, driver: JointDriver, robot_id: str,
                 name: str = "", tick_hz: float = 100.0, decel_time: float = 0.4,
                 software: str = "remote-control-py/0.1", project: str = "", stage: str = "",
                 tree: Optional[Mapping[str, Any]] = None,
                 base_pose: Optional[Mapping[str, Any]] = None,
                 describer: Optional[Callable[[bool], Dict[str, Any]]] = None,
                 visualizer: Optional[Callable[[Dict[str, Any]], None]] = None,
                 target_handler: Optional[Callable[[Dict[str, Any]], Optional[str]]] = None,
                 target_lister: Optional[Callable[[], List[Dict[str, Any]]]] = None) -> None:
        self.connector = connector
        self.driver = driver
        self.robot_id = robot_id
        self.name = name or robot_id
        self.software = software
        self.project = project or "default"
        self.stage = stage or "default"
        self.tree = tree
        self.base_pose = dict(base_pose or IDENTITY_POSE)
        self.describer = describer
        self.visualizer = visualizer
        self.target_handler = target_handler      # `target` requests (scene-owned targets); returns an error or None
        self.target_lister = target_lister        # targets for `describe` replies
        self.instance = uuid.uuid4().hex[:12]     # tells a reconnect of this robot from another robot with the same id
        self.on_renamed: Optional[Callable[[str, str], Any]] = None   # (old_id, new_id)
        self.tick_period = 1.0 / tick_hz
        self.executor = MotionExecutor(driver, self._emit, decel_time)
        self.heartbeat_interval = 0.5
        self.heartbeat_timeout = 2.0
        self._seq = Sequencer()
        self._rx = SeqTracker()
        self._outbox: List[Tuple[str, Optional[str], Dict[str, Any]]] = []
        self._last_rx = time.monotonic()
        self._last_hb = 0.0
        self._last_hello = 0.0
        self.welcomed = False
        self._task: Optional["asyncio.Future"] = None
        connector.on_connected = self._on_connected
        connector.on_disconnected = self._on_disconnected
        connector.on_message = self._on_message
        connector.bind(robot_id)

    @property
    def transport(self) -> RobotConnector:   # pre-0.3 name
        return self.connector

    async def start(self) -> None:
        await self.connector.start()
        self._task = asyncio.ensure_future(self._loop())

    async def stop(self) -> None:
        self.executor.abort_all("robot shutting down")
        await self._flush()
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
        await self.connector.stop()

    # ── outgoing ─────────────────────────────────────────────────────────────

    def _emit(self, msg_type: str, goal_id: Optional[str], payload: Dict[str, Any]) -> None:
        self._outbox.append((msg_type, goal_id, payload))

    async def _flush(self) -> None:
        out, self._outbox = self._outbox, []
        if not self.connector.connected:
            return  # state is re-sent in hello on reconnect; results of finished goals are lost
        for msg_type, goal_id, payload in out:
            env = self._seq.stamp(Envelope(msg_type, self.robot_id, payload, goal_id))
            await self.connector.send(env, channel_of(msg_type, from_robot=True))

    def hello_payload(self) -> Dict[str, Any]:
        return {
            "protocol": PROTOCOL_VERSION,
            "name": self.name,
            "software": self.software,
            "joints": [j.to_dict() for j in self.driver.joints()],
            "supports": {"pause": True, "report_points": True, "report_progress": True,
                         "pose_targets": False, "describe": True, "visualize": self.visualizer is not None,
                         "targets": self.target_handler is not None},
            "instance": self.instance,
            "project": self.project,
            "stage": self.stage,
            "state": self.executor.state_payload(),
        }

    def describe(self, tree: bool = True) -> Dict[str, Any]:
        """Body of a `description` reply (without ref_seq / ok / message)."""
        pos = self.driver.read_positions()
        joints = self.driver.joints()
        d: Dict[str, Any] = {
            "project": self.project, "stage": self.stage, "robot": self.robot_id, "name": self.name,
            "software": self.software, "frame": FRAME, "base_pose": self.base_pose,
            "positions": {j.name: pos.get(j.name) for j in joints},
            "state": self.executor.state,
        }
        if self.target_lister is not None:
            d["targets"] = self.target_lister()
        if self.describer is not None:
            d.update(self.describer(tree))
            return d
        if not tree:
            return d
        if self.tree is not None:
            poses = forward_kinematics(self.tree, pos, self.base_pose)
            d["root"] = self.tree.get("root")
            d["links"] = [{"name": l["name"], "pose": poses.get(l["name"])} for l in self.tree["links"]]
            d["joints"] = [dict(j, position=pos.get(j["command_name"]) if j.get("command_name") else None)
                           for j in self.tree["joints"]]
        else:   # joints only: no geometry known
            d["root"] = None
            d["links"] = []
            d["joints"] = [{"name": j.name, "type": j.type, "parent": None, "child": None, "command_name": j.name,
                            "lower": j.lower, "upper": j.upper, "max_velocity": j.max_velocity,
                            "position": pos.get(j.name)} for j in joints]
        return d

    # ── connector events ─────────────────────────────────────────────────────

    async def _on_connected(self) -> None:
        self._seq.reset()
        self._rx.reset()
        self._last_rx = time.monotonic()
        self._outbox.clear()  # anything queued while offline is superseded by hello.state
        self._send_hello()
        await self._flush()

    def _send_hello(self) -> None:
        self.welcomed = False
        self._last_hello = time.monotonic()
        self._emit(MsgType.HELLO, None, self.hello_payload())

    async def _on_disconnected(self) -> None:
        self.welcomed = False
        self.executor.pause_for("connection_lost")

    async def _on_message(self, env: Envelope) -> None:
        if env.robot_id != self.robot_id:
            return
        if env.type == MsgType.WELCOME:
            self._rx.reset()
        if not self._rx.accept(env.seq):
            return
        self._last_rx = time.monotonic()
        if env.type == MsgType.WELCOME:
            p = env.payload
            new_id = p.get("robot_id")
            if new_id and new_id != self.robot_id and p.get("instance") in (None, self.instance):
                # the controller already has a robot with our id: reconnect under the id it assigned
                asyncio.ensure_future(self.rename(new_id))
                return
            self.welcomed = True
            self.heartbeat_interval = float(p.get("heartbeat_interval", self.heartbeat_interval))
            self.heartbeat_timeout = float(p.get("heartbeat_timeout", self.heartbeat_timeout))
        elif env.type in (MsgType.VISUALIZE, MsgType.TARGET):
            handler = self.visualizer if env.type == MsgType.VISUALIZE else self.target_handler
            ok, message = True, ""
            if handler is None:
                ok, message = False, f"{env.type} not supported"
            else:
                try:
                    error = handler(dict(env.payload))
                    if isinstance(error, str) and error:
                        ok, message = False, error
                except Exception as exc:
                    log.exception("%s failed", env.type)
                    ok, message = False, f"{env.type} failed: {exc}"
            self._emit(MsgType.ACK, None, {"ref_seq": env.seq, "ref_type": env.type, "ok": ok, "message": message})
        elif env.type == MsgType.DESCRIBE:
            try:
                body = {"ok": True, "message": "", **self.describe(bool(env.payload.get("tree", True)))}
            except Exception as exc:  # report, don't drop the connection
                log.exception("describe failed")
                body = {"ok": False, "message": f"describe failed: {exc}"}
            self._emit(MsgType.DESCRIPTION, None, {"ref_seq": env.seq, **body})
        elif env.type != MsgType.HEARTBEAT:
            self.executor.handle(env)
        await self._flush()

    def select(self, item_id: str, source: str = "user") -> None:
        """Tell the controller the user picked a visualized item (e.g. clicked a frame in the viewer)."""
        self._emit(MsgType.SELECTED, None, {"id": item_id, "source": source})

    def edited(self, item_id: str, parent: str, pose: Dict[str, Any], source: str = "user") -> None:
        """Tell the controller the user moved an editable item (pose in the parent link, ROS convention)."""
        self._emit(MsgType.EDITED, None, {"id": item_id, "parent": parent, "pose": pose, "source": source})

    async def rename(self, new_id: str) -> None:
        """Switch to a new robot_id (assigned by the controller) and reconnect under it."""
        old = self.robot_id
        log.warning("robot_id '%s' is taken on this controller - reconnecting as '%s'", old, new_id)
        self.welcomed = False
        await self.connector.stop()
        self.robot_id = new_id
        self.connector.bind(new_id)
        await self.connector.start()
        if self.on_renamed is not None:
            r = self.on_renamed(old, new_id)
            if asyncio.iscoroutine(r):
                await r

    # ── loop ─────────────────────────────────────────────────────────────────

    async def _loop(self) -> None:
        last = time.monotonic()
        while True:
            await asyncio.sleep(self.tick_period)
            now = time.monotonic()
            self.executor.tick(now - last)
            last = now
            if self.connector.connected:
                if now - self._last_rx > self.heartbeat_timeout:
                    self.executor.pause_for("connection_lost")
                    if self.welcomed:  # controller lost us — announce again until welcomed
                        self.welcomed = False
                        self._last_hello = 0.0
                if not self.welcomed and now - self._last_hello >= max(1.0, self.heartbeat_timeout):
                    self._send_hello()
                if now - self._last_hb >= self.heartbeat_interval:
                    self._last_hb = now
                    self._emit(MsgType.HEARTBEAT, None, {})
            await self._flush()
