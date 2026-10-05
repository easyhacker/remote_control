"""
Connectors: how envelopes travel. The protocol and executor never depend on a concrete one.

Controller side — one ControllerConnector serves many robots. Each connected peer is a Link;
the controller learns a link's robot_id from its `hello`.

Robot side — one RobotConnector keeps a single (auto-reconnecting) connection to the controller.

To add a connector type (serial, cloud relay…): subclass both classes and register the type in
connectors/__init__.py (name, URL schemes, and how to build it from a config section).
"""
from __future__ import annotations

import abc
from typing import Awaitable, Callable, Optional

from ..protocol import Channel, Envelope


class Link(abc.ABC):
    """One robot connection as seen by the controller."""

    id: str

    @abc.abstractmethod
    async def send(self, env: Envelope, channel: Channel) -> None:
        ...

    @abc.abstractmethod
    async def close(self) -> None:
        ...


LinkHandler = Callable[[Link], Awaitable[None]]
LinkMessageHandler = Callable[[Link, Envelope], Awaitable[None]]
MessageHandler = Callable[[Envelope], Awaitable[None]]
EventHandler = Callable[[], Awaitable[None]]


class ControllerConnector(abc.ABC):
    def __init__(self) -> None:
        self.on_link_open: Optional[LinkHandler] = None
        self.on_link_closed: Optional[LinkHandler] = None
        self.on_message: Optional[LinkMessageHandler] = None

    @abc.abstractmethod
    async def start(self) -> None:
        ...

    @abc.abstractmethod
    async def stop(self) -> None:
        ...

    async def _opened(self, link: Link) -> None:
        if self.on_link_open:
            await self.on_link_open(link)

    async def _closed(self, link: Link) -> None:
        if self.on_link_closed:
            await self.on_link_closed(link)

    async def _received(self, link: Link, env: Envelope) -> None:
        if self.on_message:
            await self.on_message(link, env)


class RobotConnector(abc.ABC):
    def __init__(self) -> None:
        self.on_connected: Optional[EventHandler] = None
        self.on_disconnected: Optional[EventHandler] = None
        self.on_message: Optional[MessageHandler] = None
        self.robot_id: Optional[str] = None

    def bind(self, robot_id: str) -> None:
        """Called by the runtime before start(); transports that address by robot (MQTT topics) need it."""
        self.robot_id = robot_id

    @property
    @abc.abstractmethod
    def connected(self) -> bool:
        """True while messages can reach the controller (for MQTT: broker up AND controller present)."""

    @abc.abstractmethod
    async def start(self) -> None:
        """Begin connecting (and reconnecting after drops) in the background."""

    @abc.abstractmethod
    async def stop(self) -> None:
        ...

    @abc.abstractmethod
    async def send(self, env: Envelope, channel: Channel) -> None:
        """Send if connected; silently drop otherwise (state is re-sent in `hello` on reconnect)."""

    async def _connected(self) -> None:
        if self.on_connected:
            await self.on_connected()

    async def _disconnected(self) -> None:
        if self.on_disconnected:
            await self.on_disconnected()

    async def _received(self, env: Envelope) -> None:
        if self.on_message:
            await self.on_message(env)
