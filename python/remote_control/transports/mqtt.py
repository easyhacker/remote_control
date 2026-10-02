"""
MQTT transport: robot and controller both connect out to a broker.

URL:  mqtt://[user:pass@]host[:1883]/<prefix>     mqtts:// for TLS (default port 8883)
      The path is the topic prefix (default "rc"), e.g. mqtt://broker.local/robotmarket/rc

Topics (QoS 1 unless noted):
  <prefix>/<robot_id>/cmd             controller → robot   goals            (COMMAND channel)
  <prefix>/<robot_id>/ctrl            controller → robot   pause/resume/cancel/stop/welcome/heartbeat (CONTROL)
  <prefix>/<robot_id>/status          robot → controller   everything the robot sends
  <prefix>/<robot_id>/online          retained "1" / "0"   robot presence; "0" is the robot's last will
  <prefix>/_controller/online         retained "1" / "0"   controller presence; "0" is its last will

A robot counts as *connected* (RobotTransport.connected) only while it is connected to the broker AND the
controller's presence is "1" — so it pauses and re-announces itself exactly as it would over WebSocket.
Heartbeats use QoS 0. Commands and controls use separate topics, so a pause is never queued behind a goal.

Requires paho-mqtt >= 2.0 (`pip install paho-mqtt`).
"""
from __future__ import annotations

import asyncio
import logging
import ssl
import uuid
from typing import Awaitable, Callable, Dict, List, Optional, Tuple
from urllib.parse import unquote, urlparse

import paho.mqtt.client as mqtt

from ..protocol import Channel, Envelope, MsgType, ProtocolError
from .base import ControllerTransport, Link, RobotTransport

log = logging.getLogger(__name__)

CONTROLLER_PRESENCE = "_controller"
DEFAULT_PREFIX = "rc"
KEEPALIVE = 10


class MqttUrl:
    def __init__(self, url: str) -> None:
        u = urlparse(url)
        if u.scheme not in ("mqtt", "mqtts"):
            raise ValueError(f"not an mqtt:// URL: {url}")
        self.tls = u.scheme == "mqtts"
        self.host = u.hostname or "localhost"
        self.port = u.port or (8883 if self.tls else 1883)
        self.username = unquote(u.username) if u.username else None
        self.password = unquote(u.password) if u.password else None
        self.prefix = u.path.strip("/") or DEFAULT_PREFIX


def _check_level(name: str, what: str) -> None:
    if not name or any(c in name for c in "/+#") or name == CONTROLLER_PRESENCE:
        raise ValueError(f"{what} '{name}' cannot be used as an MQTT topic level")


class _Client:
    """paho client whose thread callbacks become ordered asyncio events."""

    Event = Tuple[str, str, bytes]    # (kind, topic, payload); kind: connected | disconnected | message

    def __init__(self, url: MqttUrl, client_id: str, will_topic: str, subscriptions: List[str],
                 on_event: Callable[["_Client.Event"], Awaitable[None]]) -> None:
        self.url = url
        self.will_topic = will_topic
        self.subscriptions = subscriptions
        self._on_event = on_event
        self._loop = asyncio.get_event_loop()
        self._queue: "asyncio.Queue[_Client.Event]" = asyncio.Queue()
        self._pump: Optional["asyncio.Future"] = None

        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id,
                        protocol=mqtt.MQTTv311, clean_session=True)
        if url.username:
            c.username_pw_set(url.username, url.password)
        if url.tls:
            c.tls_set(cert_reqs=ssl.CERT_REQUIRED)
        c.will_set(will_topic, b"0", qos=1, retain=True)
        c.reconnect_delay_set(min_delay=1, max_delay=5)
        c.on_connect = self._paho_connect
        c.on_disconnect = self._paho_disconnect
        c.on_message = self._paho_message
        self.client = c

    # paho thread ──────────────────────────────────────────────────────────
    def _post(self, ev: "_Client.Event") -> None:
        self._loop.call_soon_threadsafe(self._queue.put_nowait, ev)

    def _paho_connect(self, client, userdata, flags, reason_code, properties) -> None:
        if reason_code.is_failure:
            log.warning("MQTT connect to %s:%d refused: %s", self.url.host, self.url.port, reason_code)
            return
        for topic in self.subscriptions:
            client.subscribe(topic, qos=1)
        client.publish(self.will_topic, b"1", qos=1, retain=True)
        self._post(("connected", "", b""))

    def _paho_disconnect(self, client, userdata, flags, reason_code, properties) -> None:
        self._post(("disconnected", "", b""))

    def _paho_message(self, client, userdata, msg) -> None:
        self._post(("message", msg.topic, bytes(msg.payload)))

    # asyncio side ─────────────────────────────────────────────────────────
    async def _run_pump(self) -> None:
        while True:
            ev = await self._queue.get()
            try:
                await self._on_event(ev)
            except Exception:
                log.exception("MQTT event handler failed")

    def start(self) -> None:
        self._pump = asyncio.ensure_future(self._run_pump())
        self.client.connect_async(self.url.host, self.url.port, keepalive=KEEPALIVE)
        self.client.loop_start()

    async def stop(self) -> None:
        try:
            if self.client.is_connected():
                info = self.client.publish(self.will_topic, b"0", qos=1, retain=True)
                await self._loop.run_in_executor(None, info.wait_for_publish, 1.0)
            self.client.disconnect()
        except Exception:  # broker already gone
            pass
        await self._loop.run_in_executor(None, self.client.loop_stop)
        if self._pump:
            self._pump.cancel()
            try:
                await self._pump
            except asyncio.CancelledError:
                pass

    def publish(self, topic: str, text: str, qos: int = 1) -> None:
        self.client.publish(topic, text.encode("utf-8"), qos=qos)


def _decode(payload: bytes) -> Optional[Envelope]:
    try:
        return Envelope.from_json(payload.decode("utf-8"))
    except (ProtocolError, UnicodeDecodeError) as exc:
        log.warning("dropping invalid MQTT message: %s", exc)
        return None


# ── robot side ───────────────────────────────────────────────────────────────

class MqttRobotTransport(RobotTransport):
    def __init__(self, url: str) -> None:
        super().__init__()
        self.url = MqttUrl(url)
        self._client: Optional[_Client] = None
        self._broker_up = False
        self._controller_up = False
        self._linked = False

    @property
    def connected(self) -> bool:
        return self._linked

    def _topic(self, leaf: str) -> str:
        return f"{self.url.prefix}/{self.robot_id}/{leaf}"

    async def start(self) -> None:
        if not self.robot_id:
            raise RuntimeError("MqttRobotTransport needs bind(robot_id) before start()")
        _check_level(self.robot_id, "robot_id")
        self._presence = f"{self.url.prefix}/{CONTROLLER_PRESENCE}/online"
        self._inbound = {self._topic("cmd"), self._topic("ctrl")}
        self._client = _Client(self.url, f"rc-robot-{self.robot_id}", self._topic("online"),
                               [self._topic("cmd"), self._topic("ctrl"), self._presence], self._on_event)
        self._client.start()

    async def stop(self) -> None:
        if self._client:
            await self._client.stop()
            self._client = None
        await self._set_link(False)

    async def send(self, env: Envelope, channel: Channel) -> None:
        if self._linked and self._client:
            qos = 0 if env.type == MsgType.HEARTBEAT else 1
            self._client.publish(self._topic("status"), env.to_json(), qos)

    async def _on_event(self, ev: "_Client.Event") -> None:
        kind, topic, payload = ev
        if kind == "connected":
            self._broker_up = True
        elif kind == "disconnected":
            self._broker_up = False
            self._controller_up = False   # re-learned from the retained presence on reconnect
        elif topic == self._presence:
            self._controller_up = payload == b"1"
        elif topic in self._inbound and self._linked:
            env = _decode(payload)
            if env is not None:
                await self._received(env)
        await self._set_link(self._broker_up and self._controller_up)

    async def _set_link(self, up: bool) -> None:
        if up == self._linked:
            return
        self._linked = up
        await (self._connected() if up else self._disconnected())


# ── controller side ──────────────────────────────────────────────────────────

class _MqttLink(Link):
    def __init__(self, transport: "MqttControllerTransport", robot_id: str) -> None:
        self.id = f"mqtt-{robot_id}"
        self.robot_id = robot_id
        self._t = transport

    async def send(self, env: Envelope, channel: Channel) -> None:
        client = self._t._client
        if client is None:
            return
        leaf = "cmd" if channel == Channel.COMMAND else "ctrl"
        qos = 0 if env.type == MsgType.HEARTBEAT else 1
        client.publish(f"{self._t.url.prefix}/{self.robot_id}/{leaf}", env.to_json(), qos)

    async def close(self) -> None:
        # MQTT has no per-robot connection to close: forget the link; if the robot is still alive it
        # notices the missing heartbeats, pauses, and re-sends hello (which opens a new link).
        await self._t._drop_link(self.robot_id)


class MqttControllerTransport(ControllerTransport):
    def __init__(self, url: str) -> None:
        super().__init__()
        self.url = MqttUrl(url)
        self._client: Optional[_Client] = None
        self._links: Dict[str, _MqttLink] = {}

    async def start(self) -> None:
        p = self.url.prefix
        self._client = _Client(self.url, f"rc-controller-{uuid.uuid4().hex[:8]}",
                               f"{p}/{CONTROLLER_PRESENCE}/online",
                               [f"{p}/+/status", f"{p}/+/online"], self._on_event)
        self._client.start()

    async def stop(self) -> None:
        if self._client:
            await self._client.stop()
            self._client = None
        for robot_id in list(self._links):
            await self._drop_link(robot_id)

    async def _drop_link(self, robot_id: str) -> None:
        link = self._links.pop(robot_id, None)
        if link is not None:
            await self._closed(link)

    async def _on_event(self, ev: "_Client.Event") -> None:
        kind, topic, payload = ev
        if kind == "disconnected":           # robots' state unknown until they announce again
            for robot_id in list(self._links):
                await self._drop_link(robot_id)
            return
        if kind != "message":
            return
        parts = topic.split("/")
        if len(parts) < 3:
            return
        robot_id, leaf = parts[-2], parts[-1]
        if robot_id == CONTROLLER_PRESENCE:
            return
        if leaf == "online":
            if payload != b"1":
                await self._drop_link(robot_id)
        elif leaf == "status":
            env = _decode(payload)
            if env is None or env.robot_id != robot_id:
                return
            link = self._links.get(robot_id)
            if link is None:
                link = self._links[robot_id] = _MqttLink(self, robot_id)
                await self._opened(link)
            await self._received(link, env)
