"""
Pick a transport from an endpoint URL.

    controller_transport_from_url("ws://0.0.0.0:8765/motion")      # listen
    robot_transport_from_url("ws://192.168.1.10:8765/motion")      # dial
    loopback://<name>                                              # in-process (tests)

Planned: mqtt://broker:1883/<prefix>, ros2://<namespace>. A new transport registers its scheme
in the two tables below.
"""
from __future__ import annotations

from urllib.parse import urlparse

from .base import ControllerTransport, Link, RobotTransport
from .loopback import LoopbackControllerTransport, LoopbackRobotTransport


def _ws_controller(url: str) -> ControllerTransport:
    from .websocket import WebSocketControllerTransport, parse_ws_url
    host, port, path = parse_ws_url(url)
    return WebSocketControllerTransport(host, port, path)


def _ws_robot(url: str) -> RobotTransport:
    from .websocket import WebSocketRobotTransport
    return WebSocketRobotTransport(url)


def _loop_controller(url: str) -> ControllerTransport:
    return LoopbackControllerTransport(urlparse(url).netloc or "default")


def _loop_robot(url: str) -> RobotTransport:
    return LoopbackRobotTransport(urlparse(url).netloc or "default")


CONTROLLER_SCHEMES = {"ws": _ws_controller, "loopback": _loop_controller}
ROBOT_SCHEMES = {"ws": _ws_robot, "wss": _ws_robot, "loopback": _loop_robot}


def _lookup(table, url: str):
    scheme = urlparse(url).scheme
    if scheme not in table:
        raise ValueError(f"no transport for '{scheme}://' (known: {', '.join(sorted(table))})")
    return table[scheme](url)


def controller_transport_from_url(url: str) -> ControllerTransport:
    return _lookup(CONTROLLER_SCHEMES, url)


def robot_transport_from_url(url: str) -> RobotTransport:
    return _lookup(ROBOT_SCHEMES, url)


__all__ = [
    "ControllerTransport", "RobotTransport", "Link",
    "controller_transport_from_url", "robot_transport_from_url",
    "LoopbackControllerTransport", "LoopbackRobotTransport",
]
