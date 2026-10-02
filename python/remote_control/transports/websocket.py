"""
WebSocket transport: the robot dials ws://host:port/path, the controller listens.

One JSON text frame per envelope. Outgoing frames go through a two-lane queue so control
messages (pause/cancel/stop/heartbeat) overtake goals waiting to be sent.
"""
from __future__ import annotations

import asyncio
import itertools
import logging
from collections import deque
from typing import Deque, Optional
from urllib.parse import urlparse

try:  # websockets >= 13
    from websockets.asyncio.client import connect as _ws_connect
    from websockets.asyncio.server import serve as _ws_serve

    def _request_path(ws) -> str:
        return ws.request.path
except ImportError:  # websockets 10–12
    from websockets.client import connect as _ws_connect  # type: ignore
    from websockets.server import serve as _ws_serve  # type: ignore

    def _request_path(ws) -> str:
        return ws.path

from websockets.exceptions import ConnectionClosed

from ..protocol import Channel, Envelope, ProtocolError
from .base import ControllerTransport, Link, RobotTransport

log = logging.getLogger(__name__)
MAX_FRAME = 16 * 1024 * 1024
_ids = itertools.count(1)


class _Sender:
    """Two-lane outgoing queue: CONTROL first, then COMMAND/STATUS."""

    def __init__(self, ws) -> None:
        self._ws = ws
        self._control: Deque[str] = deque()
        self._other: Deque[str] = deque()
        self._wake = asyncio.Event()
        self._task = asyncio.ensure_future(self._run())

    def put(self, text: str, channel: Channel) -> None:
        (self._control if channel == Channel.CONTROL else self._other).append(text)
        self._wake.set()

    async def _run(self) -> None:
        try:
            while True:
                await self._wake.wait()
                self._wake.clear()
                while self._control or self._other:
                    text = self._control.popleft() if self._control else self._other.popleft()
                    await self._ws.send(text)
        except ConnectionClosed:
            pass

    async def close(self) -> None:
        self._task.cancel()
        try:
            await self._task
        except (asyncio.CancelledError, Exception):
            pass


def _decode(text) -> Optional[Envelope]:
    if isinstance(text, bytes):
        text = text.decode("utf-8", "replace")
    try:
        return Envelope.from_json(text)
    except ProtocolError as exc:
        log.warning("dropping invalid message: %s", exc)
        return None


# ── controller side ──────────────────────────────────────────────────────────

class _WsLink(Link):
    def __init__(self, ws) -> None:
        self.id = f"ws-{next(_ids)}"
        self.ws = ws
        self.sender = _Sender(ws)

    async def send(self, env: Envelope, channel: Channel) -> None:
        self.sender.put(env.to_json(), channel)

    async def close(self) -> None:
        await self.ws.close()


class WebSocketControllerTransport(ControllerTransport):
    """Listens on ws://host:port/path (host 0.0.0.0 to accept robots from other machines)."""

    def __init__(self, host: str = "0.0.0.0", port: int = 8765, path: str = "/motion") -> None:
        super().__init__()
        self.host, self.port, self.path = host, port, path or "/"
        self._server = None

    async def start(self) -> None:
        self._server = await _ws_serve(self._handle, self.host, self.port, max_size=MAX_FRAME)
        log.info("listening on ws://%s:%d%s", self.host, self.port, self.path)

    @property
    def bound_port(self) -> int:
        """Actual port (useful when constructed with port=0)."""
        return self._server.sockets[0].getsockname()[1] if self._server else self.port

    async def stop(self) -> None:
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
            self._server = None

    async def _handle(self, ws, *_legacy_path) -> None:
        if _request_path(ws).split("?")[0].rstrip("/") != self.path.rstrip("/"):
            await ws.close(code=1008, reason="unknown path")
            return
        link = _WsLink(ws)
        await self._opened(link)
        try:
            async for text in ws:
                env = _decode(text)
                if env is not None:
                    await self._received(link, env)
        except ConnectionClosed:
            pass
        finally:
            await link.sender.close()
            await self._closed(link)


# ── robot side ───────────────────────────────────────────────────────────────

class WebSocketRobotTransport(RobotTransport):
    """Connects to ws://host:port/path and reconnects with backoff (0.5 s → 5 s) after drops."""

    def __init__(self, url: str, min_backoff: float = 0.5, max_backoff: float = 5.0) -> None:
        super().__init__()
        self.url = url
        self.min_backoff, self.max_backoff = min_backoff, max_backoff
        self._ws = None
        self._sender: Optional[_Sender] = None
        self._task: Optional["asyncio.Future"] = None

    @property
    def connected(self) -> bool:
        return self._sender is not None

    async def start(self) -> None:
        self._task = asyncio.ensure_future(self._run())

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass

    async def send(self, env: Envelope, channel: Channel) -> None:
        if self._sender is not None:
            self._sender.put(env.to_json(), channel)

    async def simulate_drop(self) -> None:
        """Close the socket (tests); the run loop reconnects."""
        if self._ws is not None:
            await self._ws.close()

    async def _run(self) -> None:
        backoff = self.min_backoff
        while True:
            try:
                async with _ws_connect(self.url, max_size=MAX_FRAME, open_timeout=5) as ws:
                    backoff = self.min_backoff
                    self._ws = ws
                    self._sender = _Sender(ws)
                    try:
                        await self._connected()
                        async for text in ws:
                            env = _decode(text)
                            if env is not None:
                                await self._received(env)
                    finally:
                        sender, self._sender, self._ws = self._sender, None, None
                        await sender.close()
                        await self._disconnected()
            except asyncio.CancelledError:
                raise
            except (OSError, ConnectionClosed, asyncio.TimeoutError) as exc:
                log.info("connection to %s failed/closed: %s", self.url, exc)
            except Exception:  # never let the reconnect loop die
                log.exception("websocket robot transport error")
            await asyncio.sleep(backoff)
            backoff = min(self.max_backoff, backoff * 2)


def parse_ws_url(url: str):
    u = urlparse(url)
    return u.hostname or "0.0.0.0", u.port or 8765, u.path or "/motion"
