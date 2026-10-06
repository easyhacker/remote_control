# Remote Control configuration

Everything (controller, Python robots, the ROS 2 bridge, Unity robots) reads **one** file:

    %RC_CONFIG_DIR%\remote_control.json

Point the environment variable at this folder (or a copy of it elsewhere):

    setx RC_CONFIG_DIR D:\Dev\remote_control\config

`setx` affects programs started *afterwards*. Open a new terminal, and restart Unity Hub and the Editor.

`remote_control.json` holds the communication type (`connector.type`: `websocket`, `mqtt` or `ros2`) and all its
parameters. To switch, copy one of `examples/remote_control.*.json` over it, or edit it.

| Type | Parameter | Default | Meaning |
|---|---|---|---|
| all | `data_dir` | none | where the controller stores robot data: `<data_dir>/<project>/<stage>/<robot>/description.json` and `poses.json`. A relative path is relative to this folder (`../data` = the repository's `data/`) |
| all | `heartbeat.interval` / `heartbeat.timeout` | 0.5 / 2.0 s | how often both sides send heartbeats / silence before a link counts as lost (the controller sends these values to every robot) |
| `websocket` | `host` | `localhost` | address robots connect to (the controller's machine) |
| | `port` | 8765 | TCP port |
| | `path` | `/motion` | URL path |
| | `listen_host` | `0.0.0.0` | interface the controller listens on (`127.0.0.1` = this machine only) |
| | `tls` | false | `wss://` (robots) |
| | `min_backoff` / `max_backoff` | 0.5 / 5.0 s | robot reconnect delay range |
| `mqtt` | `host` / `port` | `localhost` / 1883 (8883 with TLS) | broker |
| | `prefix` | `rc` | topic prefix: `<prefix>/<robot_id>/cmd`, `ctrl`, `status`, `online` |
| | `username` / `password` (or `password_env`) | none | broker login |
| | `tls` / `ca_certs` | false / system CAs | TLS and an optional CA bundle file |
| | `keepalive` | 10 s | MQTT keepalive |
| | `client_id` | `rc-robot-<id>` / `rc-controller-<random>` | override the MQTT client id |
| `ros2` | `namespace` | `rc` | topics `/<namespace>/<robot>/cmd`, `ctrl`, `status`, `online` |
| | `domain_id` | from `ROS_DOMAIN_ID` | ROS 2 domain |

Secrets: write `"password_env": "RC_MQTT_PASSWORD"` instead of a password, and set that environment variable.
Unity robots support `websocket` and `mqtt`, not `ros2`.
