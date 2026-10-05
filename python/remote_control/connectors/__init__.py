"""
Connectors: how envelopes travel between controller and robots. One base class per side —
ControllerConnector / RobotConnector (connectors/base.py) — and a derived class per type:

    type        controller side                  robot side                  URL schemes
    websocket   WebSocketControllerConnector     WebSocketRobotConnector     ws:// wss://
    mqtt        MqttControllerConnector          MqttRobotConnector          mqtt:// mqtts://
    ros2        Ros2ControllerConnector          Ros2RobotConnector          ros2://        (needs rclpy)
    loopback    LoopbackControllerConnector      LoopbackRobotConnector      loopback://    (tests)

The type is configuration, given either as a URL or as a config section / file:

    robot_connector_from_url("mqtt://broker.local/rc")
    robot_connector_from_config({"type": "mqtt", "host": "broker.local", "prefix": "rc"})
    robot_connector_from_config("robot.json")          # a config file; uses its "connector" section
    robot_connector_from_system_config()               # THE system config: $RC_CONFIG_DIR/remote_control.json

To add a type: subclass both base classes and register it in CONNECTOR_TYPES below.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Dict, Tuple, Union
from urllib.parse import urlparse

from .base import ControllerConnector, Link, RobotConnector
from .config import (CONFIG_DIR_ENV, CONFIG_FILE_NAME, ConfigError, connector_section, load_config,
                     load_system_config, resolve_env, system_config_path)
from .loopback import LoopbackControllerConnector, LoopbackRobotConnector

Builder = Callable[[Dict[str, Any]], Any]


# ── builders: config section → connector instance ────────────────────────────

def _ws_controller(cfg: Dict[str, Any]) -> ControllerConnector:
    from .websocket import WebSocketControllerConnector, parse_ws_url   # needs websockets
    if cfg.get("url"):
        _, port, path = parse_ws_url(cfg["url"])
    else:
        port, path = int(cfg.get("port", 8765)), cfg.get("path", "/motion")
    # `host` is the address robots dial; the controller binds `listen_host` (all interfaces by default)
    return WebSocketControllerConnector(cfg.get("listen_host", "0.0.0.0"), port, path)


def _ws_robot(cfg: Dict[str, Any]) -> RobotConnector:
    from .websocket import WebSocketRobotConnector
    url = cfg.get("url") or "{}://{}:{}{}".format("wss" if cfg.get("tls") else "ws", cfg.get("host", "localhost"),
                                                  int(cfg.get("port", 8765)), cfg.get("path", "/motion"))
    return WebSocketRobotConnector(url, min_backoff=float(cfg.get("min_backoff", 0.5)),
                                   max_backoff=float(cfg.get("max_backoff", 5.0)))


def _mqtt_controller(cfg: Dict[str, Any]) -> ControllerConnector:
    from .mqtt import MqttControllerConnector, MqttSettings   # needs paho-mqtt
    return MqttControllerConnector(MqttSettings.from_config(cfg))


def _mqtt_robot(cfg: Dict[str, Any]) -> RobotConnector:
    from .mqtt import MqttRobotConnector, MqttSettings
    return MqttRobotConnector(MqttSettings.from_config(cfg))


def _ros2_controller(cfg: Dict[str, Any]) -> ControllerConnector:
    from .ros2 import Ros2ControllerConnector, Ros2Settings   # needs rclpy (sourced ROS 2)
    return Ros2ControllerConnector(Ros2Settings.from_config(cfg))


def _ros2_robot(cfg: Dict[str, Any]) -> RobotConnector:
    from .ros2 import Ros2RobotConnector, Ros2Settings
    return Ros2RobotConnector(Ros2Settings.from_config(cfg))


def _loop_name(cfg: Dict[str, Any]) -> str:
    return cfg.get("name") or (urlparse(cfg["url"]).netloc if cfg.get("url") else "") or "default"


@dataclass(frozen=True)
class ConnectorType:
    name: str
    schemes: Tuple[str, ...]
    controller: Builder
    robot: Builder


CONNECTOR_TYPES: Dict[str, ConnectorType] = {t.name: t for t in (
    ConnectorType("websocket", ("ws", "wss"), _ws_controller, _ws_robot),
    ConnectorType("mqtt", ("mqtt", "mqtts"), _mqtt_controller, _mqtt_robot),
    ConnectorType("ros2", ("ros2",), _ros2_controller, _ros2_robot),
    ConnectorType("loopback", ("loopback",),
                  lambda cfg: LoopbackControllerConnector(_loop_name(cfg)),
                  lambda cfg: LoopbackRobotConnector(_loop_name(cfg))),
)}
_ALIASES = {"ws": "websocket", "wss": "websocket", "mqtts": "mqtt", "ros": "ros2"}


def _type_for(cfg: Dict[str, Any]) -> ConnectorType:
    name = cfg.get("type")
    if not name and cfg.get("url"):
        scheme = urlparse(cfg["url"]).scheme
        for t in CONNECTOR_TYPES.values():
            if scheme in t.schemes:
                return t
        raise ConfigError(f"no connector for '{scheme}://' (known: "
                          f"{', '.join(s + '://' for t in CONNECTOR_TYPES.values() for s in t.schemes)})")
    name = _ALIASES.get(str(name).lower(), str(name).lower())
    if name not in CONNECTOR_TYPES:
        raise ConfigError(f"unknown connector type '{cfg.get('type')}' (known: {', '.join(CONNECTOR_TYPES)})")
    return CONNECTOR_TYPES[name]


# ── public factories ─────────────────────────────────────────────────────────

ConfigLike = Union[Dict[str, Any], str]


def _section(config: ConfigLike) -> Dict[str, Any]:
    if isinstance(config, str):          # a path to a config file
        config = load_config(config)
    return connector_section(resolve_env(config))


def controller_connector_from_config(config: ConfigLike) -> ControllerConnector:
    """Build a controller-side connector from a config section, a whole config, or a config file path."""
    cfg = _section(config)
    return _type_for(cfg).controller(cfg)


def robot_connector_from_config(config: ConfigLike) -> RobotConnector:
    """Build a robot-side connector from a config section, a whole config, or a config file path."""
    cfg = _section(config)
    return _type_for(cfg).robot(cfg)


def controller_connector_from_system_config() -> ControllerConnector:
    """From the one system config file, $RC_CONFIG_DIR/remote_control.json."""
    return controller_connector_from_config(load_system_config())


def robot_connector_from_system_config() -> RobotConnector:
    """From the one system config file, $RC_CONFIG_DIR/remote_control.json."""
    return robot_connector_from_config(load_system_config())


def controller_connector_from_url(url: str) -> ControllerConnector:
    return controller_connector_from_config({"url": url})


def robot_connector_from_url(url: str) -> RobotConnector:
    return robot_connector_from_config({"url": url})


# pre-0.3 names
ControllerTransport = ControllerConnector
RobotTransport = RobotConnector
controller_transport_from_url = controller_connector_from_url
robot_transport_from_url = robot_connector_from_url

__all__ = [
    "ControllerConnector", "RobotConnector", "Link", "ConnectorType", "CONNECTOR_TYPES", "ConfigError",
    "controller_connector_from_url", "robot_connector_from_url",
    "controller_connector_from_config", "robot_connector_from_config", "load_config",
    "controller_connector_from_system_config", "robot_connector_from_system_config", "load_system_config",
    "system_config_path", "CONFIG_DIR_ENV", "CONFIG_FILE_NAME",
    "LoopbackControllerConnector", "LoopbackRobotConnector",
]
