# Remote Control

Send motion goals (timed joint poses) to robots and simulators, get a report as each pose is reached,
and pause, resume, cancel or stop the motion at any time. The behaviour is the same over any connector
(WebSocket, MQTT, ROS 2), and the connector type is configuration.

- **Protocol**: [PROTOCOL.md](PROTOCOL.md). JSON envelopes, the goal lifecycle, timing and interruption rules, heartbeats.
- **Controller (Python)**: [`python/remote_control`](python/remote_control). The API your code uses to drive robots.
- **Robot client (Unity)**: [`unity/Packages/com.robotmarket.remote-control`](unity/Packages/com.robotmarket.remote-control).
  Drives any `ArticulationBody` robot (including URDF-imported ones).

```
 Controller (python)                                   Robot (Unity / Python / later: ROS bridge)
 MotionController ─ RobotHandle ─ GoalHandle            RemoteControlRobot / RobotRuntime
        │  Envelope (JSON)                                      │  Envelope (JSON)
 ControllerConnector  ◄──────── ws:// · mqtt:// · ros2:// ────────►  RobotConnector
                                                               MotionExecutor ─► JointDriver
                                                               (pause = time-scale ramp)   (ArticulationBody, fake, …)
```

The connector, the codec and the joint driver are independent plug-ins. The motion behaviour lives in
`MotionExecutor`, which has a Python reference implementation and a C# port. Both ports pass the same conformance tests.

## Quick start: Unity

0. Point `RC_CONFIG_DIR` at the config folder (once). Then restart Unity Hub and open a new terminal:
   ```bat
   setx RC_CONFIG_DIR D:\Dev\remote_control\config
   ```
   `config/remote_control.json` selects WebSocket on `localhost:8765`. Edit it to change the type or parameters.
1. Open `unity/` in Unity 6 (6000.5.7f1). Open `Assets/Scenes/RemoteControlDemo.unity`, or in any scene use
   **RobotMarket → Remote Control → Create Demo Arm**.
2. Start a controller:
   ```bash
   cd python && python -m venv .venv && .venv/Scripts/pip install websockets
   .venv/Scripts/python examples/controller_demo.py
   ```
3. Press **Play** in Unity. The overlay shows `● connected`, and the controller prints `robot online: unity-arm`.
4. In the controller window, type `g` + Enter (go), then `p` (pause), `r` (resume), `c` (cancel), `s` (stop), `h` (home).

For your own robot, add **RemoteControlRobot** to the root `ArticulationBody`. Each revolute or prismatic
joint is listed under its GameObject name, with limits taken from the drive. Override joint names, limits or max speeds in the inspector.

There is no Unity yet? Run `python examples/fake_robot.py` in place of step 3.

### Over MQTT instead of WebSocket

Both sides connect out to a broker, so robots and the controller can sit behind different routers.

```bash
.venv/Scripts/pip install paho-mqtt
.venv/Scripts/python tools/mini_mqtt_broker.py --port 1883        # or use Mosquitto / EMQX / HiveMQ
.venv/Scripts/python examples/controller_demo.py --url mqtt://127.0.0.1:1883/rc
```

In Unity, set **RemoteControlRobot → Controller Url** to `mqtt://127.0.0.1:1883/rc` (a player build takes
`-controllerUrl mqtt://…`). The fake robot takes `--url mqtt://127.0.0.1:1883/rc`. Brokers with authentication use
`mqtt://user:pass@host/rc`, and TLS uses `mqtts://`. `tools/mini_mqtt_broker.py` is for tests and demos only.

### Real ROS 2 robots

`examples/ros2_robot_bridge.py` makes any ros2_control robot a Remote Control robot. It runs the motion executor
(timing, pause ramps, reports), streams position targets to the robot's forward position controller (or a
`joint_trajectory_controller` topic), and reads `/joint_states`. The controller can reach it over any connector.

```bat
call C:\dev\ros2-windows\local_setup.bat
python examples\fake_ros2_control.py --joints shoulder_yaw,shoulder_pitch,elbow,wrist,gripper
python examples\ros2_robot_bridge.py --url ros2://rc --id demo_arm ^
    --joints shoulder_yaw,shoulder_pitch,elbow,wrist,gripper --urdf examples\demo_arm.urdf
python examples\controller_demo.py --url ros2://rc
```

Joint limits and max velocities come from `--urdf`. `--url` can just as well be `ws://…` or `mqtt://…`. On a real robot,
point `--command-topic` at its controller, for example `/arm_position_controller/commands`. On Windows, rclpy is the
ROS install's Python 3.8, and the package supports 3.8 for that reason.

## Using the controller API

```python
from remote_control import MotionController, controller_connector_from_url

controller = MotionController(controller_connector_from_url("ws://0.0.0.0:8765/motion"))
# or from a config file:  MotionController(controller_connector_from_config("controller.json"))
await controller.start()
robot = await controller.wait_for_robot("unity-arm")

goal = await robot.execute(
    ["shoulder_yaw", "elbow"],
    [([0.5, -0.3], 2.0),          # (positions in rad/m, seconds from goal start)
     ([1.0,  0.2], 4.5)],
    report="all",                 # none | points | progress | all
    on_busy="queue",              # queue | replace | reject
)
goal.on("point_reached", lambda p: print("reached", p["point_index"], p["max_error"]))
await goal.pause();  await goal.resume()       # or goal.cancel(); robot.stop()
print(await goal.result())                     # {"status": "succeeded", "positions": [...]}
```

## Tests

```bash
cd python
.venv/Scripts/python -m unittest discover -s tests -v
```

The same conformance suite (execute, point reports, invalid goals, pause/resume, cancel, stop, replace/reject,
heartbeat loss, disconnect + reconnect) runs against:

| Robot | Connector |
|---|---|
| Python `RobotRuntime` | loopback (in-process) |
| Python `RobotRuntime` | WebSocket |
| Python `RobotRuntime` | MQTT |
| C# core of the Unity package, built with plain .NET (`Tests~/DotnetRobot`) | WebSocket |
| C# core of the Unity package, built with plain .NET | MQTT |
| Python `RobotRuntime` | ROS 2 (DDS) |

Plus `Ros2JointDriver` against a fake ros2_control robot (forward position controller and trajectory topic).

MQTT tests start `tools/mini_mqtt_broker.py` for each test, and the disconnect test kills the robot's broker
connection so that last-will and reconnect are exercised. To run them against a real broker, set
`RC_MQTT_URL=mqtt://localhost:1883`. They need `pip install websockets paho-mqtt` and the .NET SDK, and skip whatever is missing.
The ROS 2 tests need a sourced ROS 2 environment (rclpy, plus numpy for `sensor_msgs`). To run everything at once:
`call C:\dev\ros2-windows\local_setup.bat && .venv38\Scripts\python -m unittest discover -s tests`.

C# self-test of trajectory and parser, matched against the Python numbers:
`dotnet run --project unity/Packages/com.robotmarket.remote-control/Tests~/DotnetRobot -- --selftest`

End-to-end with the real Unity physics in a headless player build:

```bash
python examples/controller_demo.py --url ws://127.0.0.1:8765/motion --script
unity/Builds/RemoteControlDemo/RemoteControlDemo.exe -batchmode -nographics -controllerUrl ws://127.0.0.1:8765/motion
```

The player is built with `Unity -batchmode -projectPath unity -executeMethod RobotMarket.RemoteControl.Editor.DemoArmBuilder.BuildDemoPlayerBatch -quit`.

## Connectors and configuration

A **connector** carries the protocol's messages. Each side has a base class, and each connector type derives
from it:

| Type | Controller side | Robot side | URL |
|---|---|---|---|
| `websocket` | `WebSocketControllerConnector` | `WebSocketRobotConnector` | `ws://host:8765/motion` |
| `mqtt` | `MqttControllerConnector` | `MqttRobotConnector` | `mqtt://[user:pass@]broker:1883/<prefix>` (`mqtts://` for TLS) |
| `ros2` | `Ros2ControllerConnector` | `Ros2RobotConnector` | `ros2://<namespace>[?domain=N]` |
| `loopback` | `LoopbackControllerConnector` | `LoopbackRobotConnector` | `loopback://<name>` (tests) |

Base classes: `ControllerConnector` and `RobotConnector` in `python/remote_control/connectors/base.py`.

**The configuration file.** Everything (the controller, Python robots, the ROS 2 bridge and Unity robots) reads
**one** file, `remote_control.json`, from the directory named by the environment variable **`RC_CONFIG_DIR`**:

```bat
setx RC_CONFIG_DIR D:\Dev\remote_control\config
```

`setx` only affects programs started afterwards, so open a new terminal and restart Unity Hub and the Editor.
The repository's `config/` folder holds the active file plus one example per type (`config/examples/`).
The file sets the communication type and all its parameters:

```json
{
  "connector": {
    "type": "websocket",
    "host": "localhost",
    "port": 8765,
    "path": "/motion",
    "listen_host": "0.0.0.0",
    "tls": false,
    "min_backoff": 0.5,
    "max_backoff": 5.0
  },
  "heartbeat": { "interval": 0.5, "timeout": 2.0 }
}
```

```python
from remote_control import MotionController, controller_connector_from_system_config

controller = MotionController(controller_connector_from_system_config())   # reads %RC_CONFIG_DIR%\remote_control.json
```

- **Example scripts:** `controller_demo.py`, `fake_robot.py` and `ros2_robot_bridge.py` read the file automatically. `--url` overrides it for quick tests.
- **Unity robots:** **Remote Control Robot → Connection Source = Config File** (the default) reads it. **Controller Url** is for quick tests.
- **Errors:** if `RC_CONFIG_DIR` is unset or the file is missing, everything stops with a message naming the variable and the expected path.

Parameters per type (see also [config/README.md](config/README.md)):

| Type | Parameters |
|---|---|
| all | `heartbeat.interval` (0.5 s), `heartbeat.timeout` (2.0 s); the controller sends these to every robot |
| `websocket` | `host` (address robots connect to, `localhost`), `port` (8765), `path` (`/motion`), `listen_host` (controller bind address, `0.0.0.0`), `tls`, `min_backoff` / `max_backoff` (0.5 / 5.0 s) |
| `mqtt` | `host`, `port` (1883, or 8883 with TLS), `prefix` (`rc`), `username`, `password`, `tls`, `ca_certs`, `keepalive` (10 s), `client_id` |
| `ros2` | `namespace` (`rc`), `domain_id` (default `ROS_DOMAIN_ID`). Python only; Unity supports `websocket` and `mqtt` |

Every type also accepts `"url"` as a shorthand. **Secrets stay out of the file:** a key ending in `_env` reads an
environment variable (`"password_env": "RC_MQTT_PASSWORD"` sets `password`), and `${VAR}` is expanded inside strings.
For programmatic use, `robot_connector_from_config(dict_or_path)` also accepts other files (YAML needs `pyyaml`;
TOML needs Python 3.11+ or `tomli`).

The pre-0.3 names (`RobotTransport`, `robot_transport_from_url`, `remote_control.transports`, …) still work as aliases.

## Adding a connector type (serial, cloud relay …)

1. Python: subclass `ControllerConnector` and `RobotConnector` (`python/remote_control/connectors/base.py`), then register the type in `CONNECTOR_TYPES` in `connectors/__init__.py` (name, URL schemes, builder from a config section).
2. C#: implement `IRobotTransport` and register the scheme in `TransportFactory`.
3. Add a test class in `tests/test_motion.py` that runs `_Conformance` over the new connector.

`connectors/mqtt.py`, `connectors/ros2.py` and `Runtime/Core/MqttTransport.cs` are worked examples. Channel mappings are in
[PROTOCOL.md](PROTOCOL.md#transport-mappings).

## Layout

```
PROTOCOL.md
python/
  remote_control/   protocol.py · trajectory.py · executor.py · robot.py · controller.py
                    connectors/  base · loopback · websocket · mqtt · ros2 · config (file loading)
                    drivers/ros2.py (Ros2JointDriver) · urdf.py (limits from URDF)
  examples/         controller_demo.py (interactive / --script) · fake_robot.py
                    ros2_robot_bridge.py · fake_ros2_control.py · demo_arm.urdf
  tools/            mini_mqtt_broker.py (tests / demos)
  tests/            test_motion.py (conformance suite)
unity/              Unity 6 project
  Packages/com.robotmarket.remote-control/
    Runtime/Core/   no UnityEngine: Protocol · Trajectory · MotionExecutor · Transport (WebSocket) · MqttTransport · RobotSession
    Runtime/Plugins/MQTTnet/   MQTTnet 4.3.7 (netstandard2.1, MIT)
    Runtime/Unity/  ArticulationJointDriver · RemoteControlRobot (component + overlay)
    Editor/         DemoArmBuilder (menu + batch scene/player builds)
    Tests~/DotnetRobot/   .NET harness: C# core as a fake robot + self-test
```
