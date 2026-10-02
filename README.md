# Remote Control

Send motion goals (timed joint poses) to robots and simulators, get a report as each pose is reached,
and pause, resume, cancel or stop the motion at any time. The behaviour is the same over any transport.

- **Protocol**: [PROTOCOL.md](PROTOCOL.md). JSON envelopes, the goal lifecycle, timing and interruption rules, heartbeats.
- **Controller (Python)**: [`python/remote_control`](python/remote_control). The API your code uses to drive robots.
- **Robot client (Unity)**: [`unity/Packages/com.robotmarket.remote-control`](unity/Packages/com.robotmarket.remote-control).
  Drives any `ArticulationBody` robot (including URDF-imported ones).

```
 Controller (python)                                   Robot (Unity / Python / later: ROS bridge)
 MotionController ─ RobotHandle ─ GoalHandle            RemoteControlRobot / RobotRuntime
        │  Envelope (JSON)                                      │  Envelope (JSON)
 ControllerTransport  ◄──── ws:// (now) · mqtt:// · ros2:// ────►  RobotTransport
                                                               MotionExecutor ─► JointDriver
                                                               (pause = time-scale ramp)   (ArticulationBody, fake, …)
```

The transport, the codec and the joint driver are independent plug-ins. The motion behaviour lives in
`MotionExecutor`, which has a Python reference implementation and a C# port. Both ports pass the same conformance tests.

## Quick start: Unity

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

## Using the controller API

```python
from remote_control import MotionController, controller_transport_from_url

controller = MotionController(controller_transport_from_url("ws://0.0.0.0:8765/motion"))
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

| Robot | Transport |
|---|---|
| Python `RobotRuntime` | loopback (in-process) |
| Python `RobotRuntime` | WebSocket |
| C# core of the Unity package, built with plain .NET (`Tests~/DotnetRobot`) | WebSocket |

C# self-test of trajectory and parser, matched against the Python numbers:
`dotnet run --project unity/Packages/com.robotmarket.remote-control/Tests~/DotnetRobot -- --selftest`

End-to-end with the real Unity physics in a headless player build:

```bash
python examples/controller_demo.py --url ws://127.0.0.1:8765/motion --script
unity/Builds/RemoteControlDemo/RemoteControlDemo.exe -batchmode -nographics -controllerUrl ws://127.0.0.1:8765/motion
```

The player is built with `Unity -batchmode -projectPath unity -executeMethod RobotMarket.RemoteControl.Editor.DemoArmBuilder.BuildDemoPlayerBatch -quit`.

## Adding a transport (MQTT, ROS 2, serial …)

1. Python: subclass `ControllerTransport` and `RobotTransport` (`python/remote_control/transports/base.py`), then register the URL scheme in `transports/__init__.py`.
2. C#: implement `IRobotTransport` and register the scheme in `TransportFactory`.
3. Add a test class in `tests/test_motion.py` that runs `_Conformance` over the new transport.

Channel mapping for MQTT and ROS 2 is specified in [PROTOCOL.md](PROTOCOL.md#transport-mappings).

## Layout

```
PROTOCOL.md
python/
  remote_control/   protocol.py · trajectory.py · executor.py · robot.py · controller.py · transports/
  examples/         controller_demo.py (interactive / --script) · fake_robot.py
  tests/            test_motion.py (conformance suite)
unity/              Unity 6 project
  Packages/com.robotmarket.remote-control/
    Runtime/Core/   no UnityEngine: Protocol · Trajectory · MotionExecutor · Transport (WebSocket) · RobotSession
    Runtime/Unity/  ArticulationJointDriver · RemoteControlRobot (component + overlay)
    Editor/         DemoArmBuilder (menu + batch scene/player builds)
    Tests~/DotnetRobot/   .NET harness: C# core as a fake robot + self-test
```
