"""
ROS 2 transport: protocol envelopes as JSON in std_msgs/String messages over DDS.

Configure with a URL —  ros2://<namespace>[?domain=<ROS_DOMAIN_ID>]   e.g. ros2://rc,  ros2://lab/rc?domain=7
or a config section (see connectors/config.py):  {"type": "ros2", "namespace": "lab/rc", "domain_id": 7}
(namespace default "rc"; each segment must be a valid ROS name: letters, digits, underscore)

Topics (std_msgs/String; reliable, keep-last 100 unless noted):
  /<ns>/<robot>/cmd           controller → robot   goals                     (COMMAND channel)
  /<ns>/<robot>/ctrl          controller → robot   pause/resume/cancel/stop/welcome/heartbeat (CONTROL)
  /<ns>/<robot>/status        robot → controller   everything the robot sends
  /<ns>/<robot>/online        "1"/"0", transient-local (latched) robot presence
  /<ns>/_controller/online    "1"/"0", transient-local controller presence
<robot> is the robot_id with characters ROS names don't allow replaced by "_" (the envelope keeps the
real id).

ROS has no last-will, so presence also uses the ROS graph: a robot counts as *connected* only while the
controller's presence is "1" AND the controller is publishing to its cmd/ctrl topics AND subscribed to its
status topic. The controller drops a robot when nothing publishes its status topic any more. No custom
interfaces are needed — only std_msgs — so it works with any ROS 2 install without a colcon build.

Each transport owns its own rclpy Context and spins it on a background thread; callbacks are handed to
asyncio in order.
"""
from __future__ import annotations

import asyncio
import logging
import re
import threading
import uuid
from typing import Any, Awaitable, Callable, Dict, Optional, Tuple, Union
from urllib.parse import parse_qs, urlparse

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from ..protocol import Channel, Envelope, ProtocolError
from .base import ControllerConnector, Link, RobotConnector

log = logging.getLogger(__name__)

CONTROLLER_PRESENCE = "_controller"
DEFAULT_NAMESPACE = "rc"
GRAPH_PERIOD = 0.1

QOS_MSG = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                     reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
QOS_PRESENCE = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                          reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

_TOKEN = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


def topic_token(robot_id: str) -> str:
    """robot_id → a valid ROS name token ("arm-01" → "arm_01", "7x" → "r_7x")."""
    t = re.sub(r"[^A-Za-z0-9_]", "_", robot_id)
    return t if t and t[0].isalpha() else "r_" + t


class Ros2Settings:
    def __init__(self, namespace: str = DEFAULT_NAMESPACE, domain_id: Optional[int] = None) -> None:
        ns = "/".join(p for p in namespace.split("/") if p) or DEFAULT_NAMESPACE
        for part in ns.split("/"):
            if not _TOKEN.match(part):
                raise ValueError(f"invalid ROS namespace segment '{part}' in '{namespace}'")
        self.namespace = ns
        self.domain_id = int(domain_id) if domain_id is not None else None

    @classmethod
    def from_url(cls, url: str) -> "Ros2Settings":
        u = urlparse(url)
        if u.scheme != "ros2":
            raise ValueError(f"not a ros2:// URL: {url}")
        domain = parse_qs(u.query).get("domain")
        return cls(u.netloc + u.path, int(domain[0]) if domain else None)

    @classmethod
    def from_config(cls, cfg: Dict[str, Any]) -> "Ros2Settings":
        if cfg.get("url"):
            s = cls.from_url(cfg["url"])
            if cfg.get("domain_id") is not None:
                s.domain_id = int(cfg["domain_id"])
            return s
        return cls(cfg.get("namespace", DEFAULT_NAMESPACE), cfg.get("domain_id"))

    @classmethod
    def coerce(cls, value: Union[str, "Ros2Settings", Dict[str, Any]]) -> "Ros2Settings":
        if isinstance(value, Ros2Settings):
            return value
        if isinstance(value, dict):
            return cls.from_config(value)
        return cls.from_url(value)

    def topic(self, *parts: str) -> str:
        return "/" + "/".join((self.namespace,) + parts)


Ros2Url = Ros2Settings.from_url   # pre-0.3 name


class _RosNode:
    """A private rclpy context + node, spun on a thread; ROS callbacks become ordered asyncio events."""

    def __init__(self, url: Ros2Settings, name: str, on_event: Callable[[Tuple[Any, ...]], Awaitable[None]]) -> None:
        self._loop = asyncio.get_event_loop()
        self._queue: "asyncio.Queue[Tuple[Any, ...]]" = asyncio.Queue()
        self._on_event = on_event
        self.context = Context()
        if url.domain_id is None:
            rclpy.init(context=self.context)
        else:
            rclpy.init(context=self.context, domain_id=url.domain_id)
        self.node = rclpy.create_node(name, context=self.context)
        self._executor = SingleThreadedExecutor(context=self.context)
        self._executor.add_node(self.node)
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._pump: Optional["asyncio.Future"] = None

    def post(self, *ev: Any) -> None:
        self._loop.call_soon_threadsafe(self._queue.put_nowait, ev)

    def start(self) -> None:
        self._running = True
        self._pump = asyncio.ensure_future(self._run_pump())
        self._thread = threading.Thread(target=self._spin, name=f"rc-ros2-{self.node.get_name()}", daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        while self._running and self.context.ok():
            try:
                self._executor.spin_once(timeout_sec=0.05)
            except Exception:  # never let the spin thread die silently
                log.exception("ROS 2 spin error")

    async def _run_pump(self) -> None:
        while True:
            ev = await self._queue.get()
            try:
                await self._on_event(ev)
            except Exception:
                log.exception("ROS 2 event handler failed")

    async def stop(self) -> None:
        self._running = False
        if self._thread:
            await self._loop.run_in_executor(None, self._thread.join, 2.0)
        try:
            self._executor.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
        finally:
            if self.context.ok():
                rclpy.shutdown(context=self.context)
        if self._pump:
            self._pump.cancel()
            try:
                await self._pump
            except asyncio.CancelledError:
                pass


def _decode(text: str) -> Optional[Envelope]:
    try:
        return Envelope.from_json(text)
    except ProtocolError as exc:
        log.warning("dropping invalid ROS 2 message: %s", exc)
        return None


# ── robot side ───────────────────────────────────────────────────────────────

class Ros2RobotConnector(RobotConnector):
    def __init__(self, settings: Union[str, Ros2Settings, Dict[str, Any]]) -> None:
        super().__init__()
        self.url = self.settings = Ros2Settings.coerce(settings)
        self._ros: Optional[_RosNode] = None
        self._controller_up = False
        self._graph_up = False
        self._linked = False
        self._restart: Optional["asyncio.Future"] = None

    @property
    def connected(self) -> bool:
        return self._linked

    async def start(self) -> None:
        if not self.robot_id:
            raise RuntimeError("Ros2RobotConnector needs bind(robot_id) before start()")
        token = topic_token(self.robot_id)
        if token == CONTROLLER_PRESENCE:
            raise ValueError(f"robot_id '{self.robot_id}' is reserved")
        ros = _RosNode(self.url, f"rc_robot_{token}", self._on_event)
        n = ros.node
        self._pub_status = n.create_publisher(String, self.url.topic(token, "status"), QOS_MSG)
        self._pub_online = n.create_publisher(String, self.url.topic(token, "online"), QOS_PRESENCE)
        self._t_cmd = self.url.topic(token, "cmd")
        self._t_ctrl = self.url.topic(token, "ctrl")
        n.create_subscription(String, self._t_cmd, lambda m: ros.post("message", m.data), QOS_MSG)
        n.create_subscription(String, self._t_ctrl, lambda m: ros.post("message", m.data), QOS_MSG)
        n.create_subscription(String, self.url.topic(CONTROLLER_PRESENCE, "online"),
                              lambda m: ros.post("presence", m.data == "1"), QOS_PRESENCE)
        last = {"graph": None}

        def check_graph() -> None:   # ROS thread
            up = (self._pub_status.get_subscription_count() > 0
                  and n.count_publishers(self._t_cmd) > 0 and n.count_publishers(self._t_ctrl) > 0)
            if up != last["graph"]:
                last["graph"] = up
                ros.post("graph", up)

        n.create_timer(GRAPH_PERIOD, check_graph)
        self._pub_online.publish(String(data="1"))
        self._ros = ros
        ros.start()

    async def stop(self) -> None:
        if self._restart:
            self._restart.cancel()
        await self._shutdown_node()

    async def _shutdown_node(self) -> None:
        ros, self._ros = self._ros, None
        if ros is not None:
            try:
                self._pub_online.publish(String(data="0"))
            except Exception:
                pass
            await ros.stop()
        self._controller_up = self._graph_up = False
        await self._set_link(False)

    async def simulate_drop(self, offline_for: float = 0.5) -> None:
        """Tests: the robot's ROS node disappears and comes back."""
        await self._shutdown_node()

        async def later() -> None:
            await asyncio.sleep(offline_for)
            await self.start()
        self._restart = asyncio.ensure_future(later())

    async def send(self, env: Envelope, channel: Channel) -> None:
        if self._linked and self._ros is not None:
            self._pub_status.publish(String(data=env.to_json()))

    async def _on_event(self, ev: Tuple[Any, ...]) -> None:
        kind = ev[0]
        if kind == "presence":
            self._controller_up = ev[1]
        elif kind == "graph":
            self._graph_up = ev[1]
        elif kind == "message" and self._linked:
            env = _decode(ev[1])
            if env is not None:
                await self._received(env)
        await self._set_link(self._controller_up and self._graph_up)

    async def _set_link(self, up: bool) -> None:
        if up == self._linked:
            return
        self._linked = up
        await (self._connected() if up else self._disconnected())


# ── controller side ──────────────────────────────────────────────────────────

class _Ros2Link(Link):
    def __init__(self, transport: "Ros2ControllerConnector", token: str) -> None:
        self.id = f"ros2-{token}"
        self.token = token
        self._t = transport

    async def send(self, env: Envelope, channel: Channel) -> None:
        pubs = self._t._pubs.get(self.token)
        if pubs is not None:
            pubs[0 if channel == Channel.COMMAND else 1].publish(String(data=env.to_json()))

    async def close(self) -> None:
        # Forget the link; a live robot misses our heartbeats, pauses and re-sends hello.
        await self._t._drop_link(self.token)


class Ros2ControllerConnector(ControllerConnector):
    def __init__(self, settings: Union[str, Ros2Settings, Dict[str, Any]]) -> None:
        super().__init__()
        self.url = self.settings = Ros2Settings.coerce(settings)
        self._ros: Optional[_RosNode] = None
        self._pubs: Dict[str, Tuple[Any, Any]] = {}     # token → (cmd publisher, ctrl publisher)
        self._ready: Dict[str, bool] = {}               # token → robot node alive and matched both ways
        self._links: Dict[str, _Ros2Link] = {}

    async def start(self) -> None:
        ros = _RosNode(self.url, f"rc_controller_{uuid.uuid4().hex[:8]}", self._on_event)
        n = ros.node
        self._pub_presence = n.create_publisher(String, self.url.topic(CONTROLLER_PRESENCE, "online"), QOS_PRESENCE)
        status_re = re.compile("^" + re.escape(self.url.topic("")) + r"([A-Za-z][A-Za-z0-9_]*)/status$")
        known: Dict[str, Tuple[Any, Any, Any]] = {}     # ROS-thread view: token → (cmd pub, ctrl pub, status topic)
        last: Dict[str, bool] = {}

        def discover() -> None:   # ROS thread
            for name, types in n.get_topic_names_and_types():
                m = status_re.match(name)
                if not m or m.group(1) in known or "std_msgs/msg/String" not in types:
                    continue
                token = m.group(1)
                if token == CONTROLLER_PRESENCE:
                    continue
                cmd = n.create_publisher(String, self.url.topic(token, "cmd"), QOS_MSG)
                ctrl = n.create_publisher(String, self.url.topic(token, "ctrl"), QOS_MSG)
                n.create_subscription(String, name, lambda msg, t=token: ros.post("status", t, msg.data), QOS_MSG)
                known[token] = (cmd, ctrl, name)
                ros.post("discovered", token, cmd, ctrl)
            for token, (cmd, ctrl, status_topic) in known.items():
                ready = (n.count_publishers(status_topic) > 0
                         and cmd.get_subscription_count() > 0 and ctrl.get_subscription_count() > 0)
                if ready != last.get(token):
                    last[token] = ready
                    ros.post("ready", token, ready)

        n.create_timer(GRAPH_PERIOD, discover)
        self._pub_presence.publish(String(data="1"))
        self._ros = ros
        ros.start()

    async def stop(self) -> None:
        ros, self._ros = self._ros, None
        if ros is not None:
            try:
                self._pub_presence.publish(String(data="0"))
                await asyncio.sleep(0.05)   # let the "0" go out before the node disappears
            except Exception:
                pass
            await ros.stop()
        for token in list(self._links):
            await self._drop_link(token)

    async def _drop_link(self, token: str) -> None:
        link = self._links.pop(token, None)
        if link is not None:
            await self._closed(link)

    async def _on_event(self, ev: Tuple[Any, ...]) -> None:
        kind, token = ev[0], ev[1]
        if kind == "discovered":
            self._pubs[token] = (ev[2], ev[3])
        elif kind == "ready":
            self._ready[token] = ev[2]
            if not ev[2]:
                await self._drop_link(token)
        elif kind == "status":
            if not self._ready.get(token):
                return   # not matched both ways yet — the robot re-sends hello
            env = _decode(ev[2])
            if env is None or topic_token(env.robot_id) != token:
                return
            link = self._links.get(token)
            if link is None:
                link = self._links[token] = _Ros2Link(self, token)
                await self._opened(link)
            await self._received(link, env)
