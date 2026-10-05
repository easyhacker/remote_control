"""
In-process transport for tests and demos: loopback://<name>.

Messages still go through JSON encode/decode, are delivered asynchronously in order, and the link
can be dropped on purpose (simulate_drop) to exercise heartbeat / reconnect behaviour.
"""
from __future__ import annotations

import asyncio
import itertools
from typing import Dict, Optional

from ..protocol import Channel, Envelope
from .base import ControllerConnector, Link, RobotConnector

_controllers: Dict[str, "LoopbackControllerConnector"] = {}
_ids = itertools.count(1)


class _Pump:
    """Ordered async delivery of JSON text to a handler coroutine."""

    def __init__(self, deliver) -> None:
        self._q: "asyncio.Queue[Optional[str]]" = asyncio.Queue()
        self._deliver = deliver
        self._task = asyncio.ensure_future(self._run())

    def put(self, text: str) -> None:
        self._q.put_nowait(text)

    async def _run(self) -> None:
        while True:
            text = await self._q.get()
            if text is None:
                return
            await self._deliver(Envelope.from_json(text))

    async def close(self) -> None:
        self._task.cancel()
        try:
            await self._task
        except (asyncio.CancelledError, Exception):
            pass


class _LoopLink(Link):
    def __init__(self, robot: "LoopbackRobotConnector") -> None:
        self.id = f"loop-{next(_ids)}"
        self._robot = robot

    async def send(self, env: Envelope, channel: Channel) -> None:
        self._robot._from_controller(env.to_json())

    async def close(self) -> None:
        self._robot.simulate_drop(0.0)


class LoopbackControllerConnector(ControllerConnector):
    def __init__(self, name: str = "default") -> None:
        super().__init__()
        self.name = name

    async def start(self) -> None:
        _controllers[self.name] = self

    async def stop(self) -> None:
        if _controllers.get(self.name) is self:
            del _controllers[self.name]


class LoopbackRobotConnector(RobotConnector):
    def __init__(self, name: str = "default", reconnect_delay: float = 0.1) -> None:
        super().__init__()
        self.name = name
        self.reconnect_delay = reconnect_delay
        self._link: Optional[_LoopLink] = None
        self._to_ctrl: Optional[_Pump] = None
        self._to_robot: Optional[_Pump] = None
        self._drop: Optional[asyncio.Event] = None   # created in start(): needs a running loop on 3.8
        self._offline_for = 0.0
        self._task: Optional["asyncio.Future"] = None

    @property
    def connected(self) -> bool:
        return self._link is not None

    async def start(self) -> None:
        self._drop = asyncio.Event()
        self._task = asyncio.ensure_future(self._run())

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass

    def simulate_drop(self, offline_for: float = 0.5) -> None:
        """Break the link; reconnect after `offline_for` seconds."""
        self._offline_for = offline_for
        if self._drop is not None:
            self._drop.set()

    async def send(self, env: Envelope, channel: Channel) -> None:
        if self._to_ctrl is not None:
            self._to_ctrl.put(env.to_json())

    def _from_controller(self, text: str) -> None:
        if self._to_robot is not None:
            self._to_robot.put(text)

    async def _run(self) -> None:
        while True:
            ctrl = _controllers.get(self.name)
            if ctrl is None:
                await asyncio.sleep(self.reconnect_delay)
                continue
            link = _LoopLink(self)
            self._drop.clear()
            self._to_ctrl = _Pump(lambda env, l=link, c=ctrl: c._received(l, env))
            self._to_robot = _Pump(self._received)
            self._link = link
            try:
                await ctrl._opened(link)
                await self._connected()
                await self._drop.wait()
            finally:
                self._link = None
                for pump in (self._to_ctrl, self._to_robot):
                    await pump.close()
                self._to_ctrl = self._to_robot = None
                await self._disconnected()
                await ctrl._closed(link)
            await asyncio.sleep(self._offline_for or self.reconnect_delay)
