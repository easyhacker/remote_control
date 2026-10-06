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
5. Robot data: `d` describes the robot (project / stage / robot names, base location, kinematic tree) and saves it;
   `save NAME`, `go NAME`, `poses` and `del NAME` manage named joint poses. See [Robot data](#robot-data).

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

## Robot data

The controller keeps files per robot under the config's `data_dir` (`D:\Dev\remote_control\data` here; not in git):

```
<data_dir>/<project>/<stage>/<robot_id>/description.json   names, base location, kinematic tree, joint positions
<data_dir>/<project>/<stage>/<robot_id>/poses.json         named joint poses
```

The robot reports `project` and `stage`. Unity uses the project folder and the scene name; override them under
**RemoteControlRobot → Data names**. `fake_robot.py` takes `--project` / `--stage`. Description poses use the ROS
convention (x forward, y left, z up, metres), so a Unity robot's tree can be compared with its URDF.

| `controller_demo.py` command | API |
|---|---|
| `d` | `path, desc = await robot.save_description()` (or `await robot.describe()` without saving) |
| `save NAME` | `await robot.save_pose("NAME")` (current positions of all joints; `joints=[...]` for a subset) |
| `go NAME` | `goal = await robot.move_to_pose("NAME")` (timed from the joints' max velocities) |
| `poses` | `robot.list_poses()` |
| `del NAME` | `robot.delete_pose("NAME")` |

The controller needs the directory: `MotionController(connector, data_dir=data_dir_from_config(config, system_config_path()))`.

## Robotic Toolbox (desktop app)

A wxPython controller with native widgets on Windows, macOS and Linux:
- **Joint jog:** click a step, or hold to move continuously; drag a slider to go to a value.
- **Poses:** save, go to and delete, in the same `poses.json` as `controller_demo.py`.
- **Targets:** the tool pose (x y z, roll pitch yaw) relative to any link above it, with a reachability check and
  *Move to target*. Inverse kinematics runs in the Toolbox (numpy, damped least squares within the joint limits)
  on the kinematic tree from `describe`, so it works for any robot that supports describe. It does not check
  collisions.

```bash
cd python
.venv/Scripts/pip install -e .[toolbox]          # websockets, paho-mqtt, wxPython, numpy
.venv/Scripts/python -m robotic_toolbox          # connector + data folder from %RC_CONFIG_DIR%\remote_control.json
```

The Toolbox is the controller: start it, then the robot. `--url` and `--data-dir` override the config file.
`robotic_toolbox/backend.py` (controller thread, jog, poses) and `ik.py` have no GUI imports; `app.py` is the window.

**ROS 2 robots** reach the Toolbox through `examples/ros2_robot_bridge.py` with a WebSocket or MQTT `--url`.
The bridge runs on the ROS install's Python (3.8 on Windows Humble) and the Toolbox on 3.12:

```
Robotic Toolbox (3.12) ⇄ WebSocket / MQTT ⇄ ros2_robot_bridge.py (3.8 + rclpy) ⇄ ros2_control robot
```

### Cythonized build

```bash
.venv/Scripts/pip install cython setuptools       # plus a C compiler (Visual Studio Build Tools on Windows)
.venv/Scripts/python tools/build_cython.py --test # → build/cython: compiled remote_control + robotic_toolbox, tests run on it
cd build/cython && ../../.venv/Scripts/python -m robotic_toolbox
```

Each module becomes a native extension (`.pyd` / `.so`). Only the `__init__.py` / `__main__.py` files stay Python.
Build with the same Python version that will run the app.

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
                    kinematics.py (URDF tree, forward kinematics) · data.py (descriptions, saved poses)
  robotic_toolbox/  app.py (wxPython window) · backend.py (controller thread) · ik.py (inverse kinematics)
  examples/         controller_demo.py (interactive / --script) · fake_robot.py
                    ros2_robot_bridge.py · fake_ros2_control.py · demo_arm.urdf
  tools/            mini_mqtt_broker.py (tests / demos) · check_description.py (description vs URDF)
                    build_cython.py (compiled build)
  tests/            test_motion.py (conformance suite) · test_toolbox.py (IK, toolbox backend)
unity/              Unity 6 project
  Packages/com.robotmarket.remote-control/
    Runtime/Core/   no UnityEngine: Protocol · Trajectory · MotionExecutor · Transport (WebSocket) · MqttTransport · RobotSession
    Runtime/Plugins/MQTTnet/   MQTTnet 4.3.7 (netstandard2.1, MIT)
    Runtime/Unity/  ArticulationJointDriver · ArticulationDescriber (tree for describe) · RemoteControlRobot (component + overlay)
    Editor/         DemoArmBuilder (menu + batch scene/player builds) · debug dumps
    Tests~/DotnetRobot/   .NET harness: C# core as a fake robot + self-test
```
