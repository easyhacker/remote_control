# Remote Control

Send motion goals (timed joint poses) to robots and simulators, get a report as each pose is reached,
and pause, resume, cancel or stop the motion at any time. The behaviour is the same over any connector
(WebSocket, MQTT, ROS 2), and the connector type is configuration.

- **Protocol**: [PROTOCOL.md](PROTOCOL.md). JSON envelopes, the goal lifecycle, timing and interruption rules, heartbeats.
- **Controller (Python)**: [`python/remote_control`](python/remote_control). The API your code uses to drive robots.
- **Robot client (Unity)**: [`unity/Packages/com.logixplan.remote-control`](unity/Packages/com.logixplan.remote-control).
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
   **IVI Dynamic → Remote Control → Create Demo Arm**.
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
<data_dir>/<project>/<stage>/<robot_id>/poses.json         named joint poses (a group pose has "group")
<data_dir>/<project>/<stage>/<robot_id>/groups.json        joint groups
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
| | `robot.save_joint_group("left_gripper", ["L_finger1", "L_finger2"])` / `robot.joint_groups()` |
| | `await robot.save_pose("close_left_gripper", group="left_gripper")` (only that group's joints) |
| | `goal = await robot.move_group("left_gripper", {"L_finger1": 0.0, "L_finger2": 0.0}, 1.0)` |

Group moves use `on_busy="parallel"` when the robot supports it, so groups move independently: a gripper can close
while the arm is still moving. Goals on the same joints run one after the other.

The controller needs the directory: `MotionController(connector, data_dir=data_dir_from_config(config, system_config_path()))`.

## Robotic Toolbox (desktop app)

A wxPython controller with native widgets on Windows, macOS and Linux:
- **Joint jog:** click a step, or hold to move continuously; drag a slider to go to a value.
- **Joint groups:** the group selector in Joint jog shows one group's joints; *Home*, *Save pose…* and *Stop group*
  act on that group only. *Groups…* creates and edits groups (saved in `groups.json`). Groups are suggested from saved
  chains and from `L_` / `R_` style name prefixes (arm and gripper). Every move the Toolbox sends runs in parallel with
  moves of other joints, so groups move independently. Pause / Resume / Cancel in the toolbar act on all running moves.
- **Poses:** save, go to and delete, in the same `poses.json` as `controller_demo.py`. A pose saved with a group
  selected (e.g. `close_left_gripper`) stores only that group's joints and moves only them.
- **Tool & Targets:** pick the kinematic chain's start (origin) and end link, from the dropdowns or the kinematic tree view,
  and **Save chain…** under a name. Saving creates the chain's TCP (tool centre point), which you then offset and
  *Apply*. Each saved chain keeps its own TCP. The target can be typed in or be a saved **frame**; *From TCP…* saves where the TCP is
  now. *Check* tests reachability, *Move* goes there. Inverse kinematics runs in the Toolbox (numpy, damped least
  squares within the joint limits) on the kinematic tree from `describe`, so any robot that supports describe works.
  Collisions are not checked.
- **Viewer:** robots with `supports.visualize` (Unity) draw the chain, the TCP and the frames. Clicking a frame in
  Unity's Game view, or selecting it in the Hierarchy, makes it the target in the Toolbox. TCPs and frames are stored
  in the robot's data folder (`chains.json` with each chain's TCP, `frames.json`).

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

### Cythonized build (Windows)

`tools\build_cython.py` compiles every module of `remote_control` and `robotic_toolbox` into a native extension
(`.pyd`). Only the `__init__.py` / `__main__.py` files stay Python. The output goes to `python\build\cython\`, which
mirrors `python\`: the compiled packages, plus `tests\`, `examples\` and `tools\` copied as source. The Toolbox icon
(`robotic_toolbox\resources\`) is copied with them.

Needs Cython, setuptools and the Microsoft C compiler: install **Visual Studio Build Tools** with the
**Desktop development with C++** workload. Then, in a Command Prompt or PowerShell:

```bat
cd D:\Dev\remote_control\python
.venv\Scripts\pip install cython setuptools
```

**Build and test** (about 1 minute to build, 2 more for the tests):

```bat
cd D:\Dev\remote_control\python
.venv\Scripts\python tools\build_cython.py --test
```

Each build replaces `build\cython\`. `--test` first checks that the compiled modules are the ones imported, then runs
the same test suites as the source tree. Other options:

```bat
.venv\Scripts\python tools\build_cython.py
.venv\Scripts\python tools\build_cython.py --keep-c
```

The first one builds only; `--keep-c` keeps the generated `.c` files (debugging).

**Run the compiled Toolbox** (start the robot first, e.g. press Play in Unity):

```bat
cd D:\Dev\remote_control\python\build\cython
..\..\.venv\Scripts\python -m robotic_toolbox
```

To check that the compiled modules are in use (prints a `.pyd` path, not `.py`):

```bat
cd D:\Dev\remote_control\python\build\cython
..\..\.venv\Scripts\python -c "import robotic_toolbox.app as a; print(a.__file__)"
```

**Notes**
- After changing the code, edit the source in `python\` and rebuild. `build\cython\` holds only compiled modules and is
  replaced on every build.
- The `.pyd` files only run on the Python version and bitness they were built with: `app.cp312-win_amd64.pyd` needs
  64-bit Python 3.12 on Windows.
- To hand the app to someone, copy the whole `build\cython\` folder. They need the same Python version with the
  dependencies installed (`pip install websockets paho-mqtt wxPython numpy`).

### Windows release and installer

`tools\build_release.py` makes one zip that installs the Toolbox, the compiled Remote Control Python package and a
Unity demo robot on a Windows PC, without administrator rights or internet access:

```bat
cd D:\Dev\remote_control\python
.venv\Scripts\python tools\build_release.py --test
```

The result is `python\dist\LogixPlan-RoboticToolbox-<version>-win64.zip` (about 80 MB; 47 MB with `--no-unity`). It holds:

| Folder / file | Contents |
|---|---|
| `install.cmd`, `install.ps1` | the installer (`INSTALL.txt` explains it for testers) |
| `uninstall.cmd`, `uninstall.ps1` | copied into the install folder; also listed in Settings > Apps |
| `python\python.<version>.nupkg` | Python itself: python.org's NuGet package, a complete Python with no installer or registry entries |
| `wheels\remote_control-<version>-cp312-cp312-win_amd64.whl` | the compiled `remote_control` + `robotic_toolbox` packages (`.pyd` only) |
| `wheels\*.whl` | websockets, paho-mqtt, wxPython, numpy, at the versions the tests ran with |
| `robot\` | the Unity demo robot, built with IL2CPP |
| `examples\` | `fake_robot.py`, `controller_demo.py`, `mini_mqtt_broker.py` … (source) |

**Unity robot and IL2CPP.** IL2CPP converts the C# of a Unity *player* to C++ and compiles it to a native
`GameAssembly.dll`, so the release holds no C# source and no .NET assemblies. It needs, once:
Unity Hub > Installs > 6000.5.7f1 > Manage > Add modules > **Windows Build Support (IL2CPP)**, plus Visual Studio's
C++ tools (already needed for Cython). Close any Unity editor that has `D:\Dev\remote_control\unity` open: the build
runs Unity in batch mode on that project. To build your own robot project with IL2CPP, use the package's
`PlayerBuilder.BuildBatch` (see the comment at the top of `Editor/PlayerBuilder.cs`). `Runtime/link.xml` keeps
IL2CPP from stripping Newtonsoft.Json and MQTTnet.

Options:

```bat
.venv\Scripts\python tools\build_release.py --no-unity
.venv\Scripts\python tools\build_release.py --skip-cython --skip-unity-build
.venv\Scripts\python tools\build_release.py --no-python
.venv\Scripts\python tools\build_release.py --unity-source
```

- `--no-unity`: leave out the Unity robot, so Unity isn't needed.
- `--skip-cython --skip-unity-build`: reuse the existing `build\cython` and Unity player.
- `--no-python`: a smaller zip; `install.ps1` then downloads Python from nuget.org.
- `--unity-source`: also ship the Unity package as C# source, for Unity users.

The builder stops if the wheel contains Python sources or lacks a compiled module, if the Unity player is not an
IL2CPP build, and it never copies IL2CPP's `*_BackUpThisFolder_ButDontShipItWithYourGame` folder (the generated C++).

**What install.cmd does** (per user; see `installer\install.ps1`):

1. It unpacks Python into `%LOCALAPPDATA%\Programs\LogixPlan\RoboticToolbox\python` and checks its version.
2. It installs the wheels with `pip --no-index`, so it is offline.
3. It copies `robot\`, `examples\` and the docs.
4. It sets up the config:
   - If `RC_CONFIG_DIR` is already set, the config goes in that folder. Otherwise it goes in
     `%LOCALAPPDATA%\LogixPlan\config`, and `RC_CONFIG_DIR` is set to it.
   - A new `remote_control.json` uses WebSocket on port 8765, with data in `Documents\LogixPlan\data`. An existing
     file keeps its settings.
   - Either way, the installer writes a `"toolbox"` section saying where the Toolbox is:
     `{ "command": "<install>\python\pythonw.exe", "args": "-m robotic_toolbox" }`. Unity's
     **IVI Dynamic › Robotic Toolbox › Open Robotic Toolbox** starts what it names, passing the Editor's
     `RC_CONFIG_DIR` on.
   - The Toolbox runs once per config, because it is the controller: a second start brings the running window to
     the front.
5. It adds Start menu shortcuts (Robotic Toolbox, Demo robot, Fake robot, Uninstall), a desktop shortcut and a
   Settings > Apps entry.
6. It checks that the installed modules are the compiled ones.

Options: `-InstallDir`, `-ConfigDir`, `-DataDir`, `-Port` and `-Online`. `-NoShortcuts -NoRegister` installs without
touching shortcuts, Settings or `RC_CONFIG_DIR`, for trying it on a development PC.

The uninstaller refuses while the Toolbox or a robot from the install folder is running. It keeps the config and data
unless given `-RemoveData`.

**Testing on a clean VM** (Windows 10 / 11 x64):

1. Copy the zip to the VM, unzip it and run `install.cmd`.
2. Start the Toolbox, then *Demo robot (Unity)* or *Fake robot (demo)* from the Start menu. The robot should appear
   as connected.
3. On first start, Windows Firewall asks about Python listening on the network. *Allow* is needed only for robots
   on other computers.

### Minimal copy of a Unity robot project

To move a Unity project to another PC (e.g. a test VM) without its gigabytes of `Library` and unused assets, copy
only what some scenes need:

```bat
cd D:\Dev\remote_control\python
.venv\Scripts\python tools\unity_min_copy.py "D:\Data\unity\project1\My project" Assets/Scenes/robot0625.unity --out "D:\Temp\min\My project" --zip
```

`tools\unity_min_copy.py` writes a self-contained copy:

- **Scene assets:** the scenes and every asset they reference, found by following GUIDs through prefabs, materials and
  so on, each with its `.meta`.
- **Settings assets:** the assets the project settings reference.
- **Code:** all C# scripts and assembly definitions.
- **ProjectSettings:** with the build scene list reduced to the copied scenes.
- **Packages:** the local `file:` packages (Remote Control, Bridge) embedded in `Packages\`, so the copy doesn't need
  the original `D:\Dev\…` paths.

For `robot0625` that is 129 MB (25 MB zipped) instead of 11 GB. Keep the project folder's name (`My project`): the
robot reports it as its project, and the Toolbox files the robot data under it, so data copied from
`data\My project\` matches. The first open in Unity (same version) rebuilds `Library` (about 2 minutes here).

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
`dotnet run --project unity/Packages/com.logixplan.remote-control/Tests~/DotnetRobot -- --selftest`

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

## Implementation details: joint groups and parallel goals

### Robot side: the executor

`python/remote_control/executor.py` and its C# port `Runtime/Core/MotionExecutor.cs` hold the goal logic. The two
files mirror each other: change both, and the conformance tests in `tests/test_motion.py` run against both.

- **Goals, actives and the queue.** Every accepted goal goes into the queue. Running goals are in `actives`
  (`_actives` in C#), and no two running goals share a joint. Each goal has its own timeline: `time` advances by
  `dt × rate`, and `rate` ramps towards `target_rate` over `decel_time`. Pause, cancel and stop set `target_rate`
  to 0, so the goal slows down along its path. Each goal ramps on its own, so pausing one goal does not slow the
  others.
- **Starting goals (`_start_ready` / `StartReady`).** This runs after every new goal and whenever a goal ends. It
  walks the queue in order, keeping a set of busy joints: the joints of the running goals, plus those of the
  queued goals it has skipped so far.
  - A `parallel` goal starts if none of its joints is busy.
  - A `queue` (sequential) goal starts only when nothing is running and it is first in the queue. Everything
    behind it then waits.

  Adding the joints of skipped goals to the busy set keeps goals on the same joint in arrival order. A goal on
  free joints may still overtake them.
- **Starting check.** A goal starts from the joints' measured positions. The speed of the move to its first point
  is checked against `max_velocity` only at that moment, because the start position is not known earlier. If the
  check fails, the goal is aborted.
- **Tick.** Each running goal writes only its own joints. A finished goal frees its joints, and queued goals are
  then started.
- **State.** The robot is `paused` only when every running goal is paused. One moving goal makes it `executing`
  (or `resuming`, `pausing`, `stopping`). `goal_id` and `pause_reason` describe the first running goal, as they
  did before parallel goals, so older controllers still work. `active` and `goals` list every running goal.
- **Accepted queue_position.** This is 0 if the goal started right away. Otherwise it is the number of goals it
  waits for: for a parallel goal, only the running and queued goals that share its joints.
- **Capability.** The robot announces `supports.parallel_goals` in `hello`. A robot without it never receives
  `parallel`: `RobotHandle.parallel_on_busy` falls back to `queue`.

### Controller side: groups and group poses

- **groups.json.** `RobotStore.groups()`, `save_group()` and `delete_group()` in `data.py` read and write
  `groups.json` in the robot's data folder. `RobotHandle.save_joint_group()` first checks that the joint names
  exist.
- **Group poses.** `RobotHandle.save_pose(name, group=…)` stores only the group's joints, plus `"group": name`, in
  `poses.json`. Going to a pose moves exactly the joints stored in it, so a group pose never touches other joints.
- **move_group.** `RobotHandle.move_group(name, positions)` rejects joints outside the group. It then calls
  `move_to` with `on_busy=parallel_on_busy`.

### Toolbox

- **Every move runs in parallel.** `backend.py` sends every move (jog, slider, pose, home, target) with
  `on_busy=robot.parallel_on_busy`.
- **Goal tracking.** `_watch()` records each goal with its joints in `Backend.goals` until the goal ends.
  `running_goals(joints)` finds the goals that touch some joints. Per-group *Stop group*, Pause, Resume and Cancel
  use it. With no joints given, they act on every running goal.
- **Jog steps add up per joint.** `_jog_targets` keeps where each joint's pending jog steps end. While a jog of
  that joint is still running, a new click continues from that target, not from the measured position. A failed
  goal, or any other move of that joint, drops the entry. Continuous jog (*Hold*) cancels only the earlier goals
  of that joint.
- **Suggested groups.** `Backend.groups()` returns the saved groups first, then suggestions under names not yet
  used:
  - Short name prefixes (`L_`, `R_` …): finger, grip or jaw joints become `<prefix> gripper`, the rest
    `<prefix> arm`.
  - Each saved chain suggests its movable joints.

  Groups containing every joint are skipped, since *All joints* covers them. Suggestions are computed each time
  and are never stored until the user saves one in *Groups…*.
- **Group home.** `go_home(group)` uses the group's joints from a saved `home` pose if one exists. Otherwise it
  uses 0, clamped to the joint limits.
- **UI** (`app.py`).
  - `JogTab.group_name` is the selected group. `apply_group()` shows only its rows.
  - `GroupsDialog` edits groups.
  - `save_pose_dialog(group)` saves a group pose.
  - The joint list is a `ScrolledWindow` that always shows its scrollbar. `wheel_scrolls_parent(slider,
    always=True)` makes the mouse wheel over a joint slider scroll the list instead of moving the joint.

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
                    kinematics.py (URDF tree, forward kinematics) · data.py (descriptions, poses, chains, groups)
  robotic_toolbox/  app.py (wxPython window) · backend.py (controller thread) · ik.py (inverse kinematics)
  examples/         controller_demo.py (interactive / --script) · fake_robot.py
                    ros2_robot_bridge.py · fake_ros2_control.py · demo_arm.urdf
  tools/            mini_mqtt_broker.py (tests / demos) · check_description.py (description vs URDF)
                    build_cython.py (compiled build)
  tests/            test_motion.py (conformance suite) · test_toolbox.py (IK, toolbox backend)
unity/              Unity 6 project
  Packages/com.logixplan.remote-control/
    Runtime/Core/   no UnityEngine: Protocol · Trajectory · MotionExecutor · Transport (WebSocket) · MqttTransport · RobotSession
    Runtime/Plugins/MQTTnet/   MQTTnet 4.3.7 (netstandard2.1, MIT)
    Runtime/Unity/  ArticulationJointDriver · ArticulationDescriber (tree for describe) · RemoteControlRobot (component + overlay)
    Editor/         DemoArmBuilder (menu + batch scene/player builds) · debug dumps
    Tests~/DotnetRobot/   .NET harness: C# core as a fake robot + self-test
```
