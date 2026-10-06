"""
A tiny MQTT 3.1.1 broker for tests and local demos — NOT for production (use Mosquitto, EMQX, HiveMQ…).

Supports what the Remote Control transport uses: CONNECT (clean session, user/pass ignored), QoS 0/1
publish, retained messages, last-will on ungraceful disconnect, + / # wildcards, keepalive timeout,
PINGREQ, UNSUBSCRIBE. Session takeover on duplicate client ids. No QoS 2, no persistence.

    python tools/mini_mqtt_broker.py --port 1883

In tests: `broker = MiniBroker(port=0); await broker.start(); broker.port; await broker.kick(client_id)`.
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import struct
from typing import Dict, List, Optional, Tuple

log = logging.getLogger("mini_mqtt_broker")

CONNECT, CONNACK, PUBLISH, PUBACK = 1, 2, 3, 4
SUBSCRIBE, SUBACK, UNSUBSCRIBE, UNSUBACK = 8, 9, 10, 11
PINGREQ, PINGRESP, DISCONNECT = 12, 13, 14


def topic_matches(filt: str, topic: str) -> bool:
    if topic.startswith("$") and not filt.startswith("$"):
        return False
    f, t = filt.split("/"), topic.split("/")
    for i, part in enumerate(f):
        if part == "#":
            return True
        if i >= len(t):
            return False
        if part != "+" and part != t[i]:
            return False
    return len(f) == len(t)


def _encode_len(n: int) -> bytes:
    out = bytearray()
    while True:
        b, n = n % 128, n // 128
        out.append(b | (0x80 if n else 0))
        if not n:
            return bytes(out)


def _str(s: bytes) -> bytes:
    return struct.pack("!H", len(s)) + s


def _packet(ptype: int, flags: int, body: bytes) -> bytes:
    return bytes([(ptype << 4) | flags]) + _encode_len(len(body)) + body


class _Session:
    def __init__(self, client_id: str, writer: asyncio.StreamWriter) -> None:
        self.client_id = client_id
        self.writer = writer
        self.subs: Dict[str, int] = {}
        self.will: Optional[Tuple[str, bytes, int, bool]] = None
        self._pid = 0

    def next_pid(self) -> int:
        self._pid = self._pid % 65535 + 1
        return self._pid

    def send(self, data: bytes) -> None:
        if not self.writer.is_closing():
            self.writer.write(data)


class MiniBroker:
    def __init__(self, host: str = "127.0.0.1", port: int = 1883) -> None:
        self.host, self.port = host, port
        self.retained: Dict[str, Tuple[bytes, int]] = {}
        self.sessions: Dict[str, _Session] = {}
        self._server: Optional[asyncio.AbstractServer] = None
        self._tasks: List["asyncio.Task"] = []

    async def start(self) -> None:
        self._server = await asyncio.start_server(self._handle, self.host, self.port)
        self.port = self._server.sockets[0].getsockname()[1]
        log.info("mini MQTT broker on %s:%d", self.host, self.port)

    async def stop(self) -> None:
        for s in list(self.sessions.values()):
            s.will = None
            s.writer.close()
        if self._server:
            self._server.close()
            await self._server.wait_closed()
        for t in self._tasks:
            t.cancel()

    async def kick(self, client_id: str) -> bool:
        """Drop a client's connection as if the network failed (its will is published). A client_id ending
        in "*" kicks every client whose id starts with the rest (robot client ids carry a random suffix)."""
        if client_id.endswith("*"):
            ids = [c for c in self.sessions if c.startswith(client_id[:-1])]
        else:
            ids = [client_id] if client_id in self.sessions else []
        for c in ids:
            self.sessions[c].writer.transport.abort()
        return bool(ids)

    # ── connection handling ─────────────────────────────────────────────────

    async def _handle(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        self._tasks.append(asyncio.current_task())  # type: ignore[arg-type]
        sess: Optional[_Session] = None
        keepalive = 0
        clean = False
        try:
            while True:
                timeout = keepalive * 1.5 if keepalive else None
                first = await asyncio.wait_for(reader.readexactly(1), timeout)
                length, mult = 0, 1
                while True:
                    b = (await reader.readexactly(1))[0]
                    length += (b & 0x7F) * mult
                    mult *= 128
                    if not b & 0x80:
                        break
                body = await reader.readexactly(length)
                ptype, flags = first[0] >> 4, first[0] & 0x0F

                if ptype == CONNECT:
                    sess, keepalive = self._on_connect(body, writer)
                elif sess is None:
                    break
                elif ptype == PUBLISH:
                    self._on_publish(sess, flags, body)
                elif ptype == SUBSCRIBE:
                    self._on_subscribe(sess, body)
                elif ptype == UNSUBSCRIBE:
                    pid = body[:2]
                    pos = 2
                    while pos < len(body):
                        (n,) = struct.unpack("!H", body[pos:pos + 2])
                        sess.subs.pop(body[pos + 2:pos + 2 + n].decode(), None)
                        pos += 2 + n
                    sess.send(_packet(UNSUBACK, 0, pid))
                elif ptype == PINGREQ:
                    sess.send(_packet(PINGRESP, 0, b""))
                elif ptype == DISCONNECT:
                    clean = True
                    break
                # PUBACK and anything else: ignore
        except (asyncio.IncompleteReadError, asyncio.TimeoutError, ConnectionError, OSError):
            pass
        finally:
            if sess is not None and self.sessions.get(sess.client_id) is sess:
                del self.sessions[sess.client_id]
                if not clean and sess.will:
                    topic, payload, qos, retain = sess.will
                    self._route(topic, payload, qos, retain)
            writer.close()

    def _on_connect(self, body: bytes, writer: asyncio.StreamWriter):
        pos = 0
        (n,) = struct.unpack("!H", body[pos:pos + 2])
        pos += 2 + n                                   # protocol name
        pos += 1                                       # protocol level
        cflags = body[pos]
        pos += 1
        (keepalive,) = struct.unpack("!H", body[pos:pos + 2])
        pos += 2

        def read_str() -> bytes:
            nonlocal pos
            (ln,) = struct.unpack("!H", body[pos:pos + 2])
            v = body[pos + 2:pos + 2 + ln]
            pos += 2 + ln
            return v

        client_id = read_str().decode() or f"anon-{id(writer)}"
        sess = _Session(client_id, writer)
        if cflags & 0x04:
            topic = read_str().decode()
            payload = read_str()
            sess.will = (topic, payload, (cflags >> 3) & 0x03, bool(cflags & 0x20))
        old = self.sessions.get(client_id)
        if old is not None:                            # session takeover
            old.will = None
            old.writer.transport.abort()
        self.sessions[client_id] = sess
        sess.send(_packet(CONNACK, 0, b"\x00\x00"))
        return sess, keepalive

    def _on_publish(self, sess: _Session, flags: int, body: bytes) -> None:
        qos, retain = (flags >> 1) & 0x03, bool(flags & 0x01)
        (n,) = struct.unpack("!H", body[:2])
        topic = body[2:2 + n].decode()
        pos = 2 + n
        if qos:
            pid = body[pos:pos + 2]
            pos += 2
            sess.send(_packet(PUBACK, 0, pid))
        self._route(topic, body[pos:], min(qos, 1), retain)

    def _on_subscribe(self, sess: _Session, body: bytes) -> None:
        pid = body[:2]
        pos = 2
        granted = bytearray()
        new: List[str] = []
        while pos < len(body):
            (n,) = struct.unpack("!H", body[pos:pos + 2])
            filt = body[pos + 2:pos + 2 + n].decode()
            qos = min(body[pos + 2 + n], 1)
            pos += 3 + n
            sess.subs[filt] = qos
            granted.append(qos)
            new.append(filt)
        sess.send(_packet(SUBACK, 0, pid + bytes(granted)))
        for topic, (payload, rqos) in list(self.retained.items()):
            for filt in new:
                if topic_matches(filt, topic):
                    self._deliver(sess, topic, payload, min(rqos, sess.subs[filt]), retain=True)
                    break

    def _route(self, topic: str, payload: bytes, qos: int, retain: bool) -> None:
        if retain:
            if payload:
                self.retained[topic] = (payload, qos)
            else:
                self.retained.pop(topic, None)
        for s in list(self.sessions.values()):
            best = -1
            for filt, sq in s.subs.items():
                if topic_matches(filt, topic):
                    best = max(best, min(qos, sq))
            if best >= 0:
                self._deliver(s, topic, payload, best, retain=False)

    def _deliver(self, s: _Session, topic: str, payload: bytes, qos: int, retain: bool) -> None:
        body = _str(topic.encode())
        if qos:
            body += struct.pack("!H", s.next_pid())
        s.send(_packet(PUBLISH, (qos << 1) | (1 if retain else 0), body + payload))


async def _main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1883)
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    broker = MiniBroker(args.host, args.port)
    await broker.start()
    print(f"mini MQTT broker listening on {args.host}:{broker.port} (Ctrl+C to stop)")
    await asyncio.Event().wait()


if __name__ == "__main__":
    try:
        asyncio.run(_main())
    except KeyboardInterrupt:
        pass
