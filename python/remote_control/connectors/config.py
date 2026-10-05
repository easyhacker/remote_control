"""
Connector configuration.

The system uses ONE config file, shared by the controller and every robot (Python, ROS 2 bridge, Unity):

    $RC_CONFIG_DIR/remote_control.json

    {
      "connector": { "type": "mqtt", "host": "broker.local", "port": 1883, "prefix": "rc",
                     "username": "robot", "password_env": "RC_MQTT_PASSWORD" },
      "heartbeat": { "interval": 0.5, "timeout": 2.0 }          # controller; robots get it in `welcome`
    }

`load_system_config()` finds and reads it; it raises ConfigError naming RC_CONFIG_DIR / the expected path
when either is missing. The file is JSON so the Unity client (C#) reads the very same file.
(`load_config(path)` can also read YAML / TOML files for programmatic use.)

The `connector` section's `type` picks the connector class; the other keys are that type's options:

    type          options
    ────────────  ─────────────────────────────────────────────────────────────────────────────────
    websocket     host (address robots dial, default localhost), port (8765), path (/motion),
                  listen_host (address the controller binds, default 0.0.0.0), tls (wss://),
                  min_backoff, max_backoff
    mqtt          host, port, prefix (rc), username, password, tls, ca_certs, keepalive, client_id
    ros2          namespace (rc), domain_id
    loopback      name (in-process, for tests)

Every type also accepts "url" (e.g. "mqtt://broker/rc") instead of / in addition to its options.

Secrets stay out of the file: any key ending in `_env` is replaced by the named environment variable
(`password_env: RC_MQTT_PASSWORD` → `password`), and "${VAR}" inside string values is expanded.
"""
from __future__ import annotations

import json
import os
import re
from pathlib import Path
from typing import Any, Dict, Union

_VAR = re.compile(r"\$\{([A-Za-z_][A-Za-z0-9_]*)\}")

CONFIG_DIR_ENV = "RC_CONFIG_DIR"
CONFIG_FILE_NAME = "remote_control.json"


class ConfigError(ValueError):
    pass


def system_config_path() -> Path:
    """$RC_CONFIG_DIR/remote_control.json — the one config file for the whole system."""
    directory = os.environ.get(CONFIG_DIR_ENV)
    if not directory:
        raise ConfigError(f"environment variable {CONFIG_DIR_ENV} is not set - set it to the directory "
                          f"containing {CONFIG_FILE_NAME}")
    path = Path(directory) / CONFIG_FILE_NAME
    if not path.is_file():
        raise ConfigError(f"config file not found: {path} ({CONFIG_DIR_ENV}={directory})")
    return path


def load_system_config() -> Dict[str, Any]:
    """Read $RC_CONFIG_DIR/remote_control.json (JSON) and resolve environment references."""
    return load_config(system_config_path())


def load_config(path: Union[str, Path]) -> Dict[str, Any]:
    """Read a JSON / YAML / TOML config file and resolve environment references."""
    p = Path(path)
    text = p.read_text(encoding="utf-8")
    suffix = p.suffix.lower()
    if suffix in (".yaml", ".yml"):
        try:
            import yaml  # type: ignore
        except ImportError as exc:
            raise ConfigError("YAML config needs PyYAML: pip install pyyaml") from exc
        data = yaml.safe_load(text)
    elif suffix == ".toml":
        try:
            import tomllib  # type: ignore  # Python 3.11+
        except ImportError:
            try:
                import tomli as tomllib  # type: ignore
            except ImportError as exc:
                raise ConfigError("TOML config needs Python 3.11+ or: pip install tomli") from exc
        data = tomllib.loads(text)
    else:
        data = json.loads(text)
    if not isinstance(data, dict):
        raise ConfigError(f"{p}: top level must be an object / mapping")
    return resolve_env(data)


def resolve_env(value: Any) -> Any:
    """Expand ${VAR} in strings and turn `<key>_env: VAR` into `<key>: $VAR` (recursively)."""
    if isinstance(value, dict):
        out: Dict[str, Any] = {}
        for k, v in value.items():
            if isinstance(k, str) and k.endswith("_env") and isinstance(v, str):
                if v not in os.environ:
                    raise ConfigError(f"environment variable {v} (for '{k[:-4]}') is not set")
                out[k[:-4]] = os.environ[v]
            else:
                out[k] = resolve_env(v)
        return out
    if isinstance(value, list):
        return [resolve_env(v) for v in value]
    if isinstance(value, str):
        def sub(m: "re.Match[str]") -> str:
            name = m.group(1)
            if name not in os.environ:
                raise ConfigError(f"environment variable {name} is not set")
            return os.environ[name]
        return _VAR.sub(sub, value)
    return value


def connector_section(config: Dict[str, Any]) -> Dict[str, Any]:
    """The `connector` section of a whole-file config (or the config itself if it already is one)."""
    section = config.get("connector", config)
    if not isinstance(section, dict) or "type" not in section and "url" not in section:
        raise ConfigError("connector config needs a 'type' (websocket, mqtt, ros2, loopback) or a 'url'")
    return section
