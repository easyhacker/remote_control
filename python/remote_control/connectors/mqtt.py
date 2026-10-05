"""
MQTT transport: robot and controller both connect out to a broker.

Configure with a URL —  mqtt://[user:pass@]host[:1883]/<prefix>   (mqtts:// for TLS, default port 8883)
or a config section (see connectors/config.py):
      {"type": "mqtt", "host": "broker.local", "port": 1883, "prefix": "rc",
       "username": "robot", "password_env": "RC_MQTT_PASSWORD", "tls": false, "ca_certs": null,
       "keepalive": 10, "client_id": null}

Topics (QoS 1 unless noted):
  <prefix>/<robot_id>/cmd             controller → robot   goals            (COMMAND channel)
  <prefix>/<robot_id>/ctrl            controller → robot   pause/resume/cancel/stop/welcome/heartbeat (CONTROL)
  <prefix>/<robot_id>/status          robot → controller   everything the robot sends
  <prefix>/<robot_id>/online          retained "1" / "0"   robot presence; "0" is the robot's last will
  <prefix>/_controller/online         retained "1" / "0"   controller presence; "0" is its last will

A robot counts as *connected* (RobotConnector.connected) only while it is connected to the broker AND the
controller's presence is "1" — so it pauses and re-announces itself exactly as it would over WebSocket.
Heartbeats use QoS 0. Commands and controls use separate topics, so a pause is never queued behind a goal.

Requires paho-mqtt >= 2.0 (`pip install paho-mqtt`).
"""
from __future__ import annotations

import asyncio
import logging
import ssl
import uuid
from dataclasses import dataclass
from typing import Any, Awaitable, Callable, Dict, List, Optional, Tuple, Union
from urllib.parse import unquote, urlparse

import paho.mqtt.client as mqtt

from ..protocol import Channel, Envelope, MsgType, ProtocolError
from .base import ControllerConnector, Link, RobotConnector

log = logging.getLogger(__name__)

CONTROLLER_PRESENCE = "_controller"
DEFAULT_PREFIX = "rc"
KEEPALIVE = 10


@dataclass
class MqttSettings:
    host: str = "localhost"
    port: Optional[int] = None          # default 1883, or 8883 with TLS
    prefix: str = DEFAULT_PREFIX
    username: Optional[str] = None
    password: Optional[str] = None
    tls: bool = False
    ca_certs: Optional[str] = None      # CA bundle for TLS; None = system trust store
    keepalive: int = KEEPALIVE
    client_id: Optional[str] = None     # default rc-robot-<robot_id> / rc-controller-<random>

    def __post_init__(self) -> None:
        if self.port is None:
            self.port = 8883 if self.tls else 1883
        self.prefix = self.prefix.strip("/") or DEFAULT_PREFIX

    @classmethod
    def from_url(cls, url: str) -> "MqttSettings":
        u = urlparse(url)
        if u.scheme not in ("mqtt", "mqtts"):
            raise ValueError(f"not an mqtt:// URL: {url}")
        return cls(host=u.hostname or "localhost", port=u.port, prefix=u.path.strip("/") or DEFAULT_PREFIX,
                   username=unquote(u.username) if u.username else None,
                   password=unquote(u.password) if u.password else None, tls=u.scheme == "mqtts")

    @classmethod
    def from_config(cls, cfg: Dict[str, Any]) -> "MqttSettings":
        if cfg.get("url"):
            s = cls.from_url(cfg["url"])
            for k in ("ca_certs", "keepalive", "client_id"):
                if cfg.get(k) is not None:
                    setattr(s, k, cfg[k])
            return s
        known = {"host", "port", "prefix", "username", "password", "tls", "ca_certs", "keepalive", "client_id"}
        return cls(**{k: v for k, v in cfg.items() if k in known})

    @classmethod
    def coerce(cls, value: Union[str, "MqttSettings", Dict[str, Any]]) -> "MqttSettings":
        if isinstance(value, MqttSettings):
            return value
        if isinstance(value, dict):
            return cls.from_config(value)
        return cls.from_url(value)


MqttUrl = MqttSettings.from_url   # pre-0.3 name


def _check_level(name: str, what: str) -> None:
    if not name or any(c in name for c in "/+#") or name == CONTROLLER_PRESENCE:
        raise ValueError(f"{what} '{name}' cannot be used as an MQTT topic level")


class _Client:
    """paho client whose thread callbacks become ordered asyncio events."""

    Event = Tuple[str, str, bytes]    # (kind, topic, payload); kind: connected | disconnected | message

    def __init__(self, url: MqttSettings, client_id: str, will_topic: str, subscriptions: List[str],
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
            c.tls_set(ca_certs=url.ca_certs, cert_reqs=ssl.CERT_REQUIRED)
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
        self.client.connect_async(self.url.host, self.url.port, keepalive=self.url.keepalive)
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

class MqttRobotConnector(RobotConnector):
    def __init__(self, settings: Union[str, MqttSettings, Dict[str, Any]]) -> None:
        super().__init__()
        self.url = self.settings = MqttSettings.coerce(settings)
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
            raise RuntimeError("MqttRobotConnector needs bind(robot_id) before start()")
        _check_level(self.robot_id, "robot_id")
        self._presence = f"{self.url.prefix}/{CONTROLLER_PRESENCE}/online"
        self._inbound = {self._topic("cmd"), self._topic("ctrl")}
        self._client = _Client(self.url, self.url.client_id or f"rc-robot-{self.robot_id}", self._topic("online"),
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
    def __init__(self, transport: "MqttControllerConnector", robot_id: str) -> None:
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


class MqttControllerConnector(ControllerConnector):
    def __init__(self, settings: Union[str, MqttSettings, Dict[str, Any]]) -> None:
        super().__init__()
        self.url = self.settings = MqttSettings.coerce(settings)
        self._client: Optional[_Client] = None
        self._links: Dict[str, _MqttLink] = {}

    async def start(self) -> None:
        p = self.url.prefix
        self._client = _Client(self.url, self.url.client_id or f"rc-controller-{uuid.uuid4().hex[:8]}",
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
