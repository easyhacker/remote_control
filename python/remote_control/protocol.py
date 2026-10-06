"""
Message envelope, message types and channels of the Remote Control Motion Protocol v1.
See PROTOCOL.md at the repository root for the full specification.
"""
from __future__ import annotations

import json
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Optional, Set

PROTOCOL_VERSION = 1


class MsgType:
    # robot → controller
    HELLO = "hello"
    ACCEPTED = "accepted"
    REJECTED = "rejected"
    POINT_REACHED = "point_reached"
    FEEDBACK = "feedback"
    RESULT = "result"
    STATE = "state"
    ACK = "ack"
    DESCRIPTION = "description"
    # controller → robot
    WELCOME = "welcome"
    EXECUTE = "execute"
    PAUSE = "pause"
    RESUME = "resume"
    CANCEL = "cancel"
    STOP = "stop"
    DESCRIBE = "describe"
    # both
    HEARTBEAT = "heartbeat"


CONTROL_TYPES = frozenset({MsgType.PAUSE, MsgType.RESUME, MsgType.CANCEL, MsgType.STOP})


class Channel(Enum):
    COMMAND = "command"  # goals — may be large
    CONTROL = "control"  # pause/resume/cancel/stop/describe/heartbeat/welcome — never waits behind goals
    STATUS = "status"    # everything the robot sends


_CHANNEL_OF = {
    MsgType.EXECUTE: Channel.COMMAND,
    MsgType.PAUSE: Channel.CONTROL,
    MsgType.RESUME: Channel.CONTROL,
    MsgType.CANCEL: Channel.CONTROL,
    MsgType.STOP: Channel.CONTROL,
    MsgType.WELCOME: Channel.CONTROL,
    MsgType.DESCRIBE: Channel.CONTROL,
}


def channel_of(msg_type: str, from_robot: bool) -> Channel:
    if from_robot:
        return Channel.STATUS
    return _CHANNEL_OF.get(msg_type, Channel.CONTROL)


class GoalStatus:
    SUCCEEDED = "succeeded"
    CANCELED = "canceled"
    STOPPED = "stopped"
    ABORTED = "aborted"
    TERMINAL = frozenset({"succeeded", "canceled", "stopped", "aborted"})


class RobotState:
    IDLE = "idle"
    EXECUTING = "executing"
    PAUSING = "pausing"
    PAUSED = "paused"
    RESUMING = "resuming"
    STOPPING = "stopping"


class ProtocolError(ValueError):
    """A message could not be decoded or is not valid protocol v1."""


@dataclass
class Envelope:
    type: str
    robot_id: str
    payload: Dict[str, Any] = field(default_factory=dict)
    goal_id: Optional[str] = None
    seq: int = 0
    ts: float = 0.0
    v: int = PROTOCOL_VERSION

    def to_dict(self) -> Dict[str, Any]:
        d: Dict[str, Any] = {
            "v": self.v, "type": self.type, "robot_id": self.robot_id,
            "seq": self.seq, "ts": self.ts, "payload": self.payload,
        }
        if self.goal_id is not None:
            d["goal_id"] = self.goal_id
        return d

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), separators=(",", ":"))

    @classmethod
    def from_dict(cls, d: Any) -> "Envelope":
        if not isinstance(d, dict):
            raise ProtocolError("message must be a JSON object")
        if d.get("v") != PROTOCOL_VERSION:
            raise ProtocolError(f"unsupported protocol version {d.get('v')!r}")
        msg_type, robot_id = d.get("type"), d.get("robot_id")
        if not isinstance(msg_type, str) or not isinstance(robot_id, str):
            raise ProtocolError("message needs string 'type' and 'robot_id'")
        payload = d.get("payload") or {}
        if not isinstance(payload, dict):
            raise ProtocolError("'payload' must be an object")
        goal_id = d.get("goal_id")
        return cls(
            type=msg_type, robot_id=robot_id, payload=payload,
            goal_id=str(goal_id) if goal_id is not None else None,
            seq=int(d.get("seq") or 0), ts=float(d.get("ts") or 0.0),
        )

    @classmethod
    def from_json(cls, text: str) -> "Envelope":
        try:
            return cls.from_dict(json.loads(text))
        except (ValueError, TypeError) as exc:
            if isinstance(exc, ProtocolError):
                raise
            raise ProtocolError(f"invalid JSON: {exc}") from exc


class Sequencer:
    """Stamps outgoing envelopes with seq/ts. Reset on every (re)connection."""

    def __init__(self) -> None:
        self._next = 1

    def reset(self) -> None:
        self._next = 1

    def stamp(self, env: Envelope) -> Envelope:
        env.seq = self._next
        env.ts = time.time()
        self._next += 1
        return env


class SeqTracker:
    """
    Drops duplicate incoming messages (MQTT QoS 1 may deliver twice).

    Transports with several topics/channels (MQTT cmd + ctrl) do not keep order across them, so this
    accepts out-of-order sequence numbers and only rejects ones already seen, or older than WINDOW.
    """

    WINDOW = 1024

    def __init__(self) -> None:
        self._max = 0
        self._seen: Set[int] = set()

    def reset(self) -> None:
        self._max = 0
        self._seen = set()

    def accept(self, seq: int) -> bool:
        if seq <= 0:  # unsequenced — always accept
            return True
        if seq in self._seen or seq <= self._max - self.WINDOW:
            return False
        self._seen.add(seq)
        if seq > self._max:
            self._max = seq
        if len(self._seen) > 2 * self.WINDOW:
            self._seen = {s for s in self._seen if s > self._max - self.WINDOW}
        return True


def new_goal_id() -> str:
    return uuid.uuid4().hex[:12]
