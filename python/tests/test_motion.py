"""
Conformance tests: controller ↔ robot behaviour, run identically over every connector and robot
implementation:
  - LoopbackConnectorTests   Python robot, in-process connector
  - WebSocketConnectorTests  Python robot over WebSocket
  - CSharpRobotTests         the Unity package's C# core (built with plain .NET) over WebSocket
  - MqttConnectorTests       Python robot over MQTT
  - CSharpMqttRobotTests     the C# core over MQTT
  - Ros2ConnectorTests       Python robot over ROS 2 (DDS)            — needs a sourced ROS 2 env (rclpy)
  - Ros2DriverTests          Ros2JointDriver against a fake ros2_control robot

MQTT tests use tools/mini_mqtt_broker.py, or a real broker if RC_MQTT_URL is set
(e.g. RC_MQTT_URL=mqtt://localhost:1883 for Mosquitto).

Run from the python/ folder:   python -m unittest discover -s tests -v
The C# tests need the .NET SDK; they build unity/Packages/com.logixplan.remote-control/Tests~/DotnetRobot.
"""
import asyncio
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
import uuid
from pathlib import Path

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "tools"))

from remote_control import (FakeDriver, GoalRejected, Joint, MotionController, PoseNotFound,  # noqa: E402
                            RobotRuntime, RobotStore, Trajectory, data_dir_from_config, forward_kinematics,
                            load_urdf_tree, safe_name)
from remote_control.kinematics import quat_from_rpy, quat_rotate  # noqa: E402
from remote_control.protocol import SeqTracker  # noqa: E402
from remote_control.connectors.loopback import (LoopbackControllerConnector,  # noqa: E402
                                                LoopbackRobotConnector)

JOINTS = [   # the C# harness (Tests~/DotnetRobot/Program.cs) uses the same joints
    Joint("shoulder", "revolute", -3.0, 3.0, 4.0),
    Joint("elbow", "revolute", -2.0, 2.0, 4.0),
    Joint("slide", "prismatic", 0.0, 0.5, 1.0),
]
ROBOT_ID = "arm-test"
PROJECT, STAGE = "Test project", "stage:1"   # reported by every test robot; ':' is not allowed in file names
DEMO_URDF = os.path.join(HERE, "..", "examples", "demo_arm.urdf")
# repository root (tools/build_cython.py --test runs these tests from a build folder and sets RC_REPO_ROOT)
REPO_ROOT = os.environ.get("RC_REPO_ROOT") or os.path.normpath(os.path.join(HERE, "..", ".."))
DOTNET_PROJECT = os.path.normpath(os.path.join(
    REPO_ROOT, "unity", "Packages", "com.logixplan.remote-control", "Tests~", "DotnetRobot"))


class TrajectoryTests(unittest.TestCase):
    def test_cubic_passes_points_with_zero_end_velocity(self):
        tr = Trajectory([0.0], [1.0, 2.0, 3.0], [[1.0], [3.0], [2.0]])
        for t, x in [(0, 0.0), (1, 1.0), (2, 3.0), (3, 2.0)]:
            self.assertAlmostEqual(tr.sample(t)[0], x, places=9)
        eps = 1e-4
        self.assertAlmostEqual((tr.sample(eps)[0] - tr.sample(0)[0]) / eps, 0.0, places=2)
        self.assertAlmostEqual((tr.sample(3)[0] - tr.sample(3 - eps)[0]) / eps, 0.0, places=2)

    def test_peak_speed_within_factor(self):
        times, pos = [0.5, 0.6, 2.0, 2.2], [[1.0], [1.5], [1.6], [3.0]]
        tr = Trajectory([0.0], times, pos)
        t = [0.0] + times
        x = [0.0] + [p[0] for p in pos]
        for i in range(1, len(t)):
            avg = abs(x[i] - x[i - 1]) / (t[i] - t[i - 1])
            n = 400
            h = (t[i] - t[i - 1]) / n
            peak = max(abs(tr.sample(t[i - 1] + (k + 1) * h)[0] - tr.sample(t[i - 1] + k * h)[0]) / h
                       for k in range(n))
            self.assertLessEqual(peak, 1.5 * avg + 1e-6, f"segment {i}")

    def test_seq_tracker_drops_duplicates(self):
        s = SeqTracker()
        self.assertEqual([s.accept(n) for n in (1, 2, 2, 1, 3)], [True, True, False, False, True])

    def test_seq_tracker_accepts_out_of_order(self):
        s = SeqTracker()   # MQTT cmd and ctrl topics are not ordered relative to each other
        self.assertEqual([s.accept(n) for n in (1, 3, 2, 3, 2)], [True, True, True, False, False])
        for n in range(4, 3000):
            s.accept(n)
        self.assertFalse(s.accept(5))  # outside the window


class UrdfTests(unittest.TestCase):
    def test_urdf_limits(self):
        import tempfile
        from remote_control.urdf import load_urdf_joints
        urdf = """<robot name="r">
          <joint name="a" type="revolute"><limit lower="-1.5" upper="1.5" velocity="2.0" effort="1"/></joint>
          <joint name="b" type="prismatic"><limit lower="0" upper="0.03" velocity="0.1" effort="1"/></joint>
          <joint name="c" type="continuous"><limit velocity="3" effort="1"/></joint>
          <joint name="f" type="fixed"/></robot>"""
        with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as f:
            f.write(urdf)
        try:
            j = {x.name: x for x in load_urdf_joints(f.name)}
            self.assertEqual(sorted(j), ["a", "b", "c"])
            self.assertEqual((j["a"].lower, j["a"].upper, j["a"].max_velocity), (-1.5, 1.5, 2.0))
            self.assertEqual(j["b"].type, "prismatic")
            self.assertIsNone(j["c"].lower)
            with self.assertRaises(ValueError):
                load_urdf_joints(f.name, ["a", "zz"])
        finally:
            os.unlink(f.name)


class KinematicsTests(unittest.TestCase):
    def assertVec(self, a, b, places=6):
        for x, y in zip(a, b):
            self.assertAlmostEqual(x, y, places=places)

    def test_rpy_matches_urdf_convention(self):
        self.assertVec(quat_rotate(quat_from_rpy(0, 0, math.pi / 2), (1, 0, 0)), (0, 1, 0))
        self.assertVec(quat_rotate(quat_from_rpy(math.pi / 2, 0, 0), (0, 1, 0)), (0, 0, 1))
        # fixed axes: roll first, then yaw — x stays x under roll, then yaw turns it to y
        self.assertVec(quat_rotate(quat_from_rpy(math.pi / 2, 0, math.pi / 2), (1, 0, 0)), (0, 1, 0))

    def test_tree_and_forward_kinematics(self):
        tree = load_urdf_tree(DEMO_URDF)
        self.assertEqual(tree["root"], "base_link")
        self.assertEqual([j["command_name"] for j in tree["joints"]],
                         ["shoulder_yaw", "shoulder_pitch", "elbow", "wrist", "gripper"])
        self.assertEqual(tree["joints"][4]["type"], "prismatic")
        zero = forward_kinematics(tree, {})
        self.assertVec(zero["finger"]["position"], (0, 0, 1.1))
        bent = forward_kinematics(tree, {"shoulder_pitch": math.pi / 2, "gripper": 0.02})
        self.assertVec(bent["forearm"]["position"], (0.5, 0, 0.15))
        self.assertVec(bent["finger"]["position"], (0.95, 0, 0.13))   # gripper x axis now points down
        base = {"position": [1, 2, 0], "orientation": list(quat_from_rpy(0, 0, math.pi / 2))}
        moved = forward_kinematics(tree, {"shoulder_pitch": math.pi / 2}, base)
        self.assertVec(moved["forearm"]["position"], (1, 2.5, 0.15))


class DataStoreTests(unittest.TestCase):
    def test_safe_names(self):
        self.assertEqual(safe_name("My project"), "My project")
        self.assertEqual(safe_name("a/b\\c:d"), "a_b_c_d")
        self.assertEqual(safe_name(""), "default")
        self.assertEqual(safe_name(".."), "_")
        self.assertEqual(safe_name("scene. "), "scene")

    def test_data_dir_relative_to_config_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            cfg = Path(tmp) / "config" / "remote_control.json"
            self.assertEqual(data_dir_from_config({"data_dir": "../data"}, cfg), (Path(tmp) / "data").resolve())
            self.assertEqual(data_dir_from_config({"data_dir": tmp}), Path(tmp).resolve())
            with self.assertRaises(ValueError):
                data_dir_from_config({}, cfg)

    def test_poses_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            store = RobotStore(tmp, "P", "S/1", "r")
            self.assertEqual(store.dir, Path(tmp) / "P" / "S_1" / "r")
            self.assertEqual(store.list_poses(), [])
            store.save_pose("b", ["j1", "j2"], [1, 2])
            store.save_pose("a", ["j1"], [0.5])
            self.assertEqual(store.list_poses(), ["a", "b"])
            self.assertEqual(store.get_pose("b")["positions"], {"j1": 1.0, "j2": 2.0})
            data = json.loads((store.dir / "poses.json").read_text(encoding="utf-8"))
            self.assertEqual((data["project"], data["stage"], data["robot"]), ("P", "S/1", "r"))
            store.delete_pose("b")
            self.assertEqual(store.list_poses(), ["a"])
            with self.assertRaises(PoseNotFound):
                store.get_pose("b")
            with self.assertRaises(ValueError):
                store.save_pose(" ", ["j1"], [0])


class ConnectorConfigTests(unittest.TestCase):
    """Connector type and options come from configuration (URL, dict, or file)."""

    def test_type_selects_connector_class(self):
        from remote_control.connectors import controller_connector_from_config as ctrl
        from remote_control.connectors import robot_connector_from_config as rob
        self.assertEqual(type(rob({"type": "loopback", "name": "x"})).__name__, "LoopbackRobotConnector")
        self.assertEqual(type(ctrl({"type": "loopback"})).__name__, "LoopbackControllerConnector")
        if HAVE_WS:
            r = rob({"type": "websocket", "host": "10.0.0.5", "port": 9000})
            self.assertEqual((type(r).__name__, r.url), ("WebSocketRobotConnector", "ws://10.0.0.5:9000/motion"))
            c = ctrl({"type": "ws", "port": 9001})
            self.assertEqual((type(c).__name__, c.host, c.port), ("WebSocketControllerConnector", "0.0.0.0", 9001))
        if HAVE_MQTT:
            m = rob({"type": "mqtt", "host": "b", "prefix": "lab/rc", "tls": True, "client_id": "me"})
            self.assertEqual(type(m).__name__, "MqttRobotConnector")
            self.assertEqual((m.settings.host, m.settings.port, m.settings.prefix, m.settings.client_id),
                             ("b", 8883, "lab/rc", "me"))
        if HAVE_ROS2:
            r2 = rob({"type": "ros2", "namespace": "lab/rc", "domain_id": 7})
            self.assertEqual((r2.settings.namespace, r2.settings.domain_id), ("lab/rc", 7))

    def test_url_or_type_and_errors(self):
        from remote_control.connectors import ConfigError, robot_connector_from_config, robot_connector_from_url
        self.assertEqual(type(robot_connector_from_config({"url": "loopback://abc"})).__name__, "LoopbackRobotConnector")
        self.assertEqual(type(robot_connector_from_url("loopback://abc")).__name__, "LoopbackRobotConnector")
        with self.assertRaises(ConfigError):
            robot_connector_from_config({"type": "carrier-pigeon"})
        with self.assertRaises(ConfigError):
            robot_connector_from_config({"host": "x"})          # neither type nor url
        if HAVE_MQTT:
            m = robot_connector_from_config({"url": "mqtts://u:p@h:1234/a", "keepalive": 30})
            self.assertEqual((m.settings.username, m.settings.password, m.settings.port, m.settings.keepalive),
                             ("u", "p", 1234, 30))

    def test_secrets_from_environment(self):
        from remote_control.connectors.config import ConfigError, resolve_env
        os.environ["RC_TEST_SECRET"] = "s3cret"
        os.environ["RC_TEST_HOST"] = "broker.lan"
        try:
            cfg = resolve_env({"connector": {"type": "mqtt", "host": "${RC_TEST_HOST}", "password_env": "RC_TEST_SECRET"}})
            self.assertEqual(cfg["connector"], {"type": "mqtt", "host": "broker.lan", "password": "s3cret"})
            with self.assertRaises(ConfigError):
                resolve_env({"password_env": "RC_TEST_DOES_NOT_EXIST"})
        finally:
            del os.environ["RC_TEST_SECRET"], os.environ["RC_TEST_HOST"]

    def test_config_files(self):
        import json, tempfile
        from remote_control import load_config, robot_connector_from_config
        d = tempfile.mkdtemp()
        jp = os.path.join(d, "robot.json")
        with open(jp, "w") as f:
            json.dump({"robot_id": "r1", "connector": {"type": "loopback", "name": "n"}}, f)
        self.assertEqual(load_config(jp)["robot_id"], "r1")
        self.assertEqual(type(robot_connector_from_config(jp)).__name__, "LoopbackRobotConnector")   # path works too
        try:
            try:
                import tomllib  # noqa: F401
            except ImportError:
                import tomli  # noqa: F401
            tp = os.path.join(d, "robot.toml")
            with open(tp, "w") as f:
                f.write('robot_id = "r2"\n[connector]\ntype = "loopback"\nname = "t"\n')
            self.assertEqual(load_config(tp)["connector"]["name"], "t")
        except ImportError:
            pass
        config_dir = os.path.join(REPO_ROOT, "config")
        for folder in (config_dir, os.path.join(config_dir, "examples")):
            for name in os.listdir(folder):
                if name.endswith(".json"):
                    with open(os.path.join(folder, name)) as f:
                        self.assertIn("type", json.load(f)["connector"], name)

    def test_system_config_comes_from_rc_config_dir(self):
        import json, tempfile
        from remote_control import (ConfigError, controller_connector_from_system_config, load_system_config,
                                    robot_connector_from_system_config, system_config_path)
        saved = os.environ.pop("RC_CONFIG_DIR", None)
        try:
            with self.assertRaises(ConfigError) as cm:
                system_config_path()
            self.assertIn("RC_CONFIG_DIR", str(cm.exception))
            d = tempfile.mkdtemp()
            os.environ["RC_CONFIG_DIR"] = d
            with self.assertRaises(ConfigError) as cm:          # directory set, file missing
                load_system_config()
            self.assertIn("remote_control.json", str(cm.exception))
            with open(os.path.join(d, "remote_control.json"), "w") as f:
                json.dump({"connector": {"type": "loopback", "name": "sys"},
                           "heartbeat": {"interval": 0.2, "timeout": 1.0}}, f)
            self.assertEqual(load_system_config()["heartbeat"]["timeout"], 1.0)
            self.assertEqual(robot_connector_from_system_config().name, "sys")
            self.assertEqual(controller_connector_from_system_config().name, "sys")
            if HAVE_WS:   # one file serves both sides: robots dial `host`, the controller binds `listen_host`
                from remote_control import controller_connector_from_config, robot_connector_from_config
                ws = {"type": "websocket", "host": "10.1.2.3", "port": 9100, "listen_host": "127.0.0.1"}
                self.assertEqual(robot_connector_from_config(ws).url, "ws://10.1.2.3:9100/motion")
                c = controller_connector_from_config(ws)
                self.assertEqual((c.host, c.port), ("127.0.0.1", 9100))
        finally:
            if saved is None:
                os.environ.pop("RC_CONFIG_DIR", None)
            else:
                os.environ["RC_CONFIG_DIR"] = saved


class _Conformance:
    """Mixed into a TestCase per connector / robot implementation.

    Positions are read from protocol messages (results, feedback, state) only, so the tests work
    for robots running in another process or language.
    """

    async def start_robot(self, controller):
        """Start the robot side; return an object with an async stop()."""
        raise NotImplementedError

    async def make_controller_connector(self):
        raise NotImplementedError

    async def drop_link(self):
        """Break the connection; the robot must reconnect by itself."""
        await self.robot._link.close()

    async def asyncSetUp(self):
        self.controller = MotionController(await self.make_controller_connector(),
                                           heartbeat_interval=0.1, heartbeat_timeout=0.5)
        await self.controller.start()
        self.robot_side = await self.start_robot(self.controller)
        self.robot = await self.controller.wait_for_robot(ROBOT_ID, timeout=20)

    async def asyncTearDown(self):
        await self.robot_side.stop()
        await self.controller.stop()

    async def wait_state(self, state, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.robot.state.get("state") == state:
                return
            await asyncio.sleep(0.01)
        self.fail(f"robot never reached state {state!r} (is {self.robot.state.get('state')!r})")

    def pos(self, joint):
        return self.robot.state["positions"][joint]

    async def wait_offline(self, timeout=3.0):
        deadline = time.monotonic() + timeout
        while self.robot.online and time.monotonic() < deadline:
            await asyncio.sleep(0.01)
        self.assertFalse(self.robot.online, "controller never noticed the robot went away")

    # ── tests ────────────────────────────────────────────────────────────────

    async def test_hello_describes_robot(self):
        self.assertEqual(self.robot.joint_names, ["shoulder", "elbow", "slide"])
        self.assertEqual(self.robot.joints[2].type, "prismatic")
        self.assertEqual(self.robot.joints[0].upper, 3.0)
        self.assertTrue(self.robot.supports["pause"])
        self.assertEqual(self.robot.state["state"], "idle")

    async def test_describe_reports_names_positions_and_joints(self):
        self.assertEqual(self.robot.names, {"project": PROJECT, "stage": STAGE, "robot": ROBOT_ID})  # from hello
        self.assertTrue(self.robot.supports.get("describe"))
        d = await self.robot.describe(tree=False)
        self.assertEqual((d["project"], d["stage"], d["robot"]), (PROJECT, STAGE, ROBOT_ID))
        self.assertEqual(sorted(d["positions"]), sorted(self.robot.joint_names))
        self.assertEqual(len(d["base_pose"]["orientation"]), 4)
        self.assertNotIn("joints", d)
        full = await self.robot.describe()
        self.assertEqual([j["command_name"] for j in full["joints"] if j.get("command_name")],
                         self.robot.joint_names)

    async def test_description_and_poses_are_saved_per_project_stage_robot(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.controller.data_dir = Path(tmp)
            robot_dir = Path(tmp) / "Test project" / "stage_1" / ROBOT_ID
            path, desc = await self.robot.save_description()
            self.assertEqual(path, robot_dir / "description.json")
            saved = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual((saved["project"], saved["stage"], saved["robot"]), (PROJECT, STAGE, ROBOT_ID))

            goal = await self.robot.execute(["shoulder", "slide"], [([1.0, 0.2], 0.5)])
            self.assertEqual((await goal.result(timeout=3))["status"], "succeeded")
            await self.robot.save_pose("ready")
            goal = await self.robot.move_to({"shoulder": 0.0, "slide": 0.0}, duration=0.5)
            self.assertEqual((await goal.result(timeout=3))["status"], "succeeded")
            await self.robot.save_pose("zero", joints=["shoulder"])
            self.assertEqual(self.robot.list_poses(), ["ready", "zero"])
            self.assertEqual(self.robot.get_pose("zero"), {"shoulder": 0.0})

            goal = await self.robot.move_to_pose("ready")
            self.assertEqual((await goal.result(timeout=5))["status"], "succeeded")
            pos = await self.robot.current_positions()
            self.assertAlmostEqual(pos["shoulder"], 1.0, places=3)
            self.assertAlmostEqual(pos["slide"], 0.2, places=3)

            self.robot.delete_pose("ready")
            self.assertEqual(self.robot.list_poses(), ["zero"])
            with self.assertRaises(PoseNotFound):
                await self.robot.move_to_pose("ready")

    async def test_parallel_goals_on_different_joints(self):
        self.assertTrue(self.robot.supports.get("parallel_goals"))
        t0 = time.monotonic()
        a = await self.robot.execute(["shoulder"], [([1.0], 0.6)], on_busy="parallel")
        b = await self.robot.execute(["elbow"], [([0.8], 0.6)], on_busy="parallel")
        self.assertEqual((a.queue_position, b.queue_position), (0, 0))       # both start at once
        await asyncio.sleep(0.15)
        self.assertEqual(sorted(self.robot.state.get("active", [])), sorted([a.goal_id, b.goal_id]))
        self.assertEqual((await a.result(timeout=3))["status"], "succeeded")
        self.assertEqual((await b.result(timeout=3))["status"], "succeeded")
        self.assertLess(time.monotonic() - t0, 1.1)                          # not one after the other
        await self.wait_state("idle")
        self.assertAlmostEqual(self.pos("shoulder"), 1.0, places=3)
        self.assertAlmostEqual(self.pos("elbow"), 0.8, places=3)

    async def test_parallel_goal_on_busy_joints_waits_for_them(self):
        a = await self.robot.execute(["shoulder", "elbow"], [([1.0, 0.5], 0.5)], on_busy="parallel")
        b = await self.robot.execute(["elbow"], [([-0.5], 0.4)], on_busy="parallel")
        c = await self.robot.execute(["slide"], [([0.2], 0.4)], on_busy="parallel")
        self.assertEqual(a.queue_position, 0)
        self.assertGreaterEqual(b.queue_position, 1)                         # elbow is busy: waits for a
        self.assertEqual(c.queue_position, 0)                                # slide is free: runs now
        self.assertEqual((await b.result(timeout=5))["status"], "succeeded")
        await self.wait_state("idle")
        self.assertAlmostEqual(self.pos("elbow"), -0.5, places=3)
        self.assertAlmostEqual(self.pos("shoulder"), 1.0, places=3)
        self.assertAlmostEqual(self.pos("slide"), 0.2, places=3)

    async def test_parallel_goals_pause_and_cancel_independently(self):
        a = await self.robot.execute(["shoulder"], [([2.0], 1.0)], on_busy="parallel", report="all",
                                     progress_hz=50)
        b = await self.robot.execute(["elbow"], [([1.0], 0.6)], on_busy="parallel")
        await asyncio.sleep(0.2)
        self.assertTrue((await a.pause())["ok"])
        self.assertEqual((await b.result(timeout=3))["status"], "succeeded")    # b keeps going
        self.assertLess(self.pos("shoulder"), 2.0)
        self.assertTrue((await a.cancel())["ok"])
        self.assertEqual((await a.result(timeout=3))["status"], "canceled")
        c = await self.robot.execute(["shoulder"], [([0.5], 0.4)], on_busy="parallel")
        d = await self.robot.execute(["slide"], [([0.3], 0.8)], on_busy="parallel")
        await asyncio.sleep(0.1)
        self.assertTrue((await self.robot.stop())["ok"])                       # stop ends everything
        self.assertEqual((await c.result(timeout=3))["status"], "stopped")
        self.assertEqual((await d.result(timeout=3))["status"], "stopped")

    async def test_execute_reports_each_point_and_succeeds(self):
        goal = await self.robot.execute(["shoulder", "elbow"],
                                        [([0.5, -0.3], 0.3), ([1.0, 0.2], 0.6)], report="all", progress_hz=20)
        result = await goal.result(timeout=3)
        self.assertEqual(result["status"], "succeeded")
        self.assertEqual([p["point_index"] for p in goal.points_reached], [0, 1])
        self.assertLess(goal.points_reached[0]["max_error"], 0.1)   # measured on the tick that passes the point (Windows timers ~15 ms)
        self.assertIsNotNone(goal.last_feedback)
        self.assertAlmostEqual(result["positions"][0], 1.0)
        self.assertAlmostEqual(result["positions"][1], 0.2)
        await self.wait_state("idle")
        self.assertAlmostEqual(self.pos("slide"), 0.0)  # joint not in the goal holds

    async def test_invalid_goals_are_rejected(self):
        cases = [
            (["shoulder"], [([3.5], 1.0)], "above upper limit"),
            (["wrist"], [([0.0], 1.0)], "unknown joint"),
            (["shoulder"], [([0.1], 1.0), ([0.2], 1.0)], "time_from_start"),
            (["shoulder"], [([0.0, 1.0], 1.0)], "positions for"),
            (["slide"], [([0.1], 1.0), ([0.5], 1.1)], "max_velocity"),
        ]
        for names, pts, fragment in cases:
            with self.assertRaises(GoalRejected) as cm:
                await self.robot.execute(names, pts)
            self.assertIn(fragment, cm.exception.reason)

    async def test_pause_holds_position_then_resume_finishes(self):
        goal = await self.robot.execute(["shoulder"], [([2.0], 0.8)], report="all", progress_hz=50)
        await asyncio.sleep(0.3)
        ack = await goal.pause()
        self.assertTrue(ack["ok"])
        await self.wait_state("paused")
        await asyncio.sleep(0.1)
        held = goal.last_feedback["positions"][0]
        self.assertEqual(goal.last_feedback["rate"], 0)
        self.assertGreater(held, 0.05)
        self.assertLess(held, 2.0)
        await asyncio.sleep(0.3)
        self.assertEqual(goal.last_feedback["positions"][0], held)
        self.assertEqual((await goal.pause())["message"], "already paused")
        t0 = time.monotonic()
        self.assertTrue((await goal.resume())["ok"])
        result = await goal.result(timeout=3)
        self.assertEqual(result["status"], "succeeded")
        self.assertGreater(time.monotonic() - t0, 0.2)  # the rest of the timeline still ran
        self.assertAlmostEqual(result["positions"][0], 2.0)

    async def test_cancel_active_goal_then_queued_goal_runs(self):
        first = await self.robot.execute(["shoulder"], [([2.0], 1.0)])
        second = await self.robot.execute(["elbow"], [([0.5], 0.2)])
        self.assertEqual(second.queue_position, 1)
        await asyncio.sleep(0.2)
        self.assertTrue((await first.cancel())["ok"])
        r1 = await first.result(timeout=3)
        self.assertEqual(r1["status"], "canceled")
        self.assertLess(r1["positions"][0], 2.0)
        self.assertEqual((await second.result(timeout=3))["status"], "succeeded")

    async def test_cancel_queued_goal(self):
        first = await self.robot.execute(["shoulder"], [([1.0], 0.4)])
        second = await self.robot.execute(["elbow"], [([0.5], 0.2)])
        await second.cancel()
        self.assertEqual((await second.result(timeout=1))["status"], "canceled")
        self.assertEqual((await first.result(timeout=3))["status"], "succeeded")
        await self.wait_state("idle")
        self.assertEqual(self.pos("elbow"), 0.0)

    async def test_stop_ends_active_and_queued(self):
        first = await self.robot.execute(["shoulder"], [([2.0], 1.0)])
        second = await self.robot.execute(["elbow"], [([0.5], 0.2)])
        await asyncio.sleep(0.2)
        self.assertTrue((await self.robot.stop())["ok"])
        self.assertEqual((await first.result(timeout=3))["status"], "stopped")
        self.assertEqual((await second.result(timeout=3))["status"], "stopped")
        await self.wait_state("idle")

    async def test_on_busy_replace_and_reject(self):
        first = await self.robot.execute(["shoulder"], [([2.0], 1.0)])
        with self.assertRaises(GoalRejected):
            await self.robot.execute(["elbow"], [([0.5], 0.2)], on_busy="reject")
        await asyncio.sleep(0.1)
        third = await self.robot.execute(["shoulder"], [([-1.0], 1.2)], on_busy="replace")
        r1 = await first.result(timeout=3)
        self.assertEqual(r1["status"], "canceled")
        self.assertIn("replaced", r1["message"])
        r3 = await third.result(timeout=3)
        self.assertEqual(r3["status"], "succeeded")
        self.assertAlmostEqual(r3["positions"][0], -1.0)

    async def test_control_for_unknown_goal_is_acked_not_ok(self):
        ack = await self.robot.goal("nope").pause()
        self.assertFalse(ack["ok"])
        self.assertEqual(ack["message"], "no such goal")

    async def test_heartbeat_loss_pauses_goal(self):
        goal = await self.robot.execute(["shoulder"], [([2.0], 1.0)])
        await asyncio.sleep(0.1)
        self.controller._hb_task.cancel()  # controller goes silent, link stays up
        await self.wait_state("paused", timeout=2)
        self.assertEqual(self.robot.state["pause_reason"], "connection_lost")
        self.controller._hb_task = asyncio.ensure_future(self.controller._heartbeat_loop())
        await goal.resume()
        self.assertEqual((await goal.result(timeout=3))["status"], "succeeded")

    async def test_disconnect_pauses_and_goal_resumes_after_reconnect(self):
        goal = await self.robot.execute(["shoulder"], [([2.0], 1.0)])
        await asyncio.sleep(0.2)
        await self.drop_link()
        await self.wait_offline()
        robot = await self.controller.wait_for_robot(ROBOT_ID, timeout=5)
        self.assertIs(robot, self.robot)
        await self.wait_state("paused")
        self.assertEqual(robot.state["goal_id"], goal.goal_id)
        self.assertEqual(robot.state["pause_reason"], "connection_lost")
        self.assertLess(self.pos("shoulder"), 2.0)
        await goal.resume()
        self.assertEqual((await goal.result(timeout=3))["status"], "succeeded")


class RobotIdAndViewerTests(unittest.IsolatedAsyncioTestCase):
    """Duplicate robot ids are renamed by the controller; visualize / selected round trip (Python robot)."""

    async def asyncSetUp(self):
        self.name = f"ids{id(self)}"
        self.controller = MotionController(LoopbackControllerConnector(self.name), heartbeat_interval=0.1,
                                           heartbeat_timeout=0.5)
        self.renamed = []
        self.controller.on("robot_renamed", self.renamed.append)
        await self.controller.start()
        self.runtimes = []

    async def asyncTearDown(self):
        for r in self.runtimes:
            await r.stop()
        await self.controller.stop()

    async def robot(self, robot_id, **kw):
        r = RobotRuntime(LoopbackRobotConnector(self.name, reconnect_delay=0.05), FakeDriver(JOINTS), robot_id,
                         tick_hz=100, decel_time=0.1, **kw)
        self.runtimes.append(r)
        await r.start()
        return r

    async def wait_online(self, robot_id, timeout=5.0):
        return await self.controller.wait_for_robot(robot_id, timeout=timeout)

    async def test_second_robot_with_same_id_is_renamed(self):
        first = await self.robot("arm")
        await self.wait_online("arm")
        second = await self.robot("arm")
        await self.wait_online("arm-2")
        self.assertEqual(second.robot_id, "arm-2")
        self.assertEqual(first.robot_id, "arm")
        self.assertTrue(self.controller.robots["arm"].online)
        self.assertEqual(self.renamed, [{"robot_id": "arm", "new_id": "arm-2"}])
        third = await self.robot("arm")
        await self.wait_online("arm-3")
        self.assertEqual(third.robot_id, "arm-3")
        goal = await self.controller.robots["arm-2"].execute(["shoulder"], [([0.5], 0.2)])
        self.assertEqual((await goal.result(timeout=3))["status"], "succeeded")
        self.assertAlmostEqual(second.driver.positions["shoulder"], 0.5)
        self.assertEqual(first.driver.positions["shoulder"], 0.0)

    async def test_reconnect_of_same_robot_keeps_its_id(self):
        r = await self.robot("arm")
        handle = await self.wait_online("arm")
        r.connector.simulate_drop(offline_for=0.1)
        await asyncio.sleep(0.6)
        self.assertIs(await self.wait_online("arm"), handle)
        self.assertEqual(r.robot_id, "arm")
        self.assertEqual(self.renamed, [])

    async def test_visualize_and_selected(self):
        shown = []
        await self.robot("viewer-arm", visualizer=shown.append)
        robot = await self.wait_online("viewer-arm")
        self.assertTrue(robot.supports["visualize"])
        items = [{"id": "frame:pick", "kind": "frame", "parent": "base_link",
                  "pose": {"position": [0.3, 0, 0.2], "orientation": [0, 0, 0, 1]}, "selectable": True}]
        ack = await robot.visualize(items)
        self.assertTrue(ack["ok"])
        self.assertEqual(shown, [{"items": items, "replace": True}])
        picked = []
        robot.on("selected", picked.append)
        self.runtimes[0].select("frame:pick", source="click")
        await asyncio.sleep(0.2)
        self.assertEqual(picked, [{"id": "frame:pick", "source": "click"}])

    async def test_visualize_without_viewer_is_refused(self):
        await self.robot("plain-arm")
        robot = await self.wait_online("plain-arm")
        self.assertFalse(robot.supports.get("visualize"))
        with self.assertRaises(Exception):
            await robot.visualize([])


class _PythonRobot:
    def __init__(self, connector):
        self.connector = connector
        self.runtime = RobotRuntime(connector, FakeDriver(JOINTS), ROBOT_ID, tick_hz=200, decel_time=0.1,
                                    project=PROJECT, stage=STAGE)

    async def start(self):
        await self.runtime.start()
        return self

    async def stop(self):
        await self.runtime.stop()


class LoopbackConnectorTests(_Conformance, unittest.IsolatedAsyncioTestCase):
    async def make_controller_connector(self):
        self.name = f"t{id(self)}"
        return LoopbackControllerConnector(self.name)

    async def start_robot(self, controller):
        return await _PythonRobot(LoopbackRobotConnector(self.name, reconnect_delay=0.05)).start()

    async def drop_link(self):
        self.robot_side.connector.simulate_drop(offline_for=0.2)


try:
    import websockets  # noqa: F401
    HAVE_WS = True
except ImportError:
    HAVE_WS = False


class _WebSocketController:
    async def make_controller_connector(self):
        from remote_control.connectors.websocket import WebSocketControllerConnector
        ctrl = WebSocketControllerConnector("127.0.0.1", 0, "/motion")
        await ctrl.start()          # bind now to learn the port …
        ctrl.start = _noop          # … so MotionController.start() doesn't bind again
        self.url = f"ws://127.0.0.1:{ctrl.bound_port}/motion"
        return ctrl


@unittest.skipUnless(HAVE_WS, "websockets not installed")
class WebSocketConnectorTests(_WebSocketController, _Conformance, unittest.IsolatedAsyncioTestCase):
    async def start_robot(self, controller):
        from remote_control.connectors.websocket import WebSocketRobotConnector
        return await _PythonRobot(WebSocketRobotConnector(self.url, min_backoff=0.1)).start()


class _DotnetRobot:
    """The Unity package's C# core, built with plain .NET, as a separate process."""

    dll = None

    @classmethod
    def build(cls):
        if cls.dll is None:
            subprocess.run(["dotnet", "build", "-nologo", "-v", "q", "-c", "Release"], cwd=DOTNET_PROJECT,
                           check=True, stdout=subprocess.DEVNULL)
            cls.dll = os.path.join(DOTNET_PROJECT, "bin", "Release", "net8.0", "DotnetRobot.dll")
        return cls.dll

    def __init__(self, url):
        self.proc = subprocess.Popen(["dotnet", self.build(), url, ROBOT_ID, PROJECT, STAGE], stdin=subprocess.PIPE,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

    def stall(self, seconds):
        """Block the harness's update loop (like a slow frame in Unity)."""
        self.proc.stdin.write(f"stall {seconds}\n".encode())
        self.proc.stdin.flush()

    async def stop(self):
        self.proc.stdin.close()   # harness exits when stdin closes
        try:
            await asyncio.get_event_loop().run_in_executor(None, lambda: self.proc.wait(timeout=5))
        except subprocess.TimeoutExpired:
            self.proc.kill()


@unittest.skipUnless(HAVE_WS and shutil.which("dotnet") and os.path.isdir(DOTNET_PROJECT),
                     "needs websockets and the .NET SDK")
class _StallTests:
    """C# robot only: a blocked update loop (slow render frame) must not drop the link."""

    async def test_slow_update_loop_keeps_link(self):
        goal = await self.robot.execute(["shoulder"], [([1.0], 0.5)])
        dropped = []
        self.robot.on("offline", lambda e: dropped.append(e))
        self.robot_side.stall(3 * self.controller.heartbeat_timeout)   # well past the timeout
        await asyncio.sleep(3 * self.controller.heartbeat_timeout + 0.5)
        self.assertEqual(dropped, [], "controller dropped a robot whose update loop was only slow")
        self.assertTrue(self.robot.online)
        self.assertEqual((await goal.result(timeout=5))["status"], "succeeded")


class CSharpRobotTests(_StallTests, _WebSocketController, _Conformance, unittest.IsolatedAsyncioTestCase):
    @classmethod
    def setUpClass(cls):
        _DotnetRobot.build()   # once, outside the event loop

    async def start_robot(self, controller):
        return _DotnetRobot(self.url)

    async def test_second_robot_with_same_id_is_renamed(self):
        second = _DotnetRobot(self.url)
        try:
            renamed = await self.controller.wait_for_robot(ROBOT_ID + "-2", timeout=20)
            self.assertTrue(renamed.online)
            self.assertTrue(self.robot.online)          # the first keeps its id
            goal = await renamed.execute(["elbow"], [([0.4], 0.2)])
            self.assertEqual((await goal.result(timeout=5))["status"], "succeeded")
        finally:
            await second.stop()


try:
    import paho.mqtt  # noqa: F401
    HAVE_MQTT = True
except ImportError:
    HAVE_MQTT = False


class _MqttBroker:
    """Each test gets its own topic prefix; the mini broker is started per test unless RC_MQTT_URL is set."""

    async def make_controller_connector(self):
        from remote_control.connectors.mqtt import MqttControllerConnector
        prefix = f"rctest{uuid.uuid4().hex[:8]}"
        base = os.environ.get("RC_MQTT_URL")
        if base:
            self.broker = None
            self.url = f"{base.rstrip('/')}/{prefix}"
        else:
            from mini_mqtt_broker import MiniBroker
            self.broker = MiniBroker(port=0)
            await self.broker.start()
            self.url = f"mqtt://127.0.0.1:{self.broker.port}/{prefix}"
        return MqttControllerConnector(self.url)

    async def asyncTearDown(self):
        await super().asyncTearDown()
        if self.broker:
            await self.broker.stop()

    async def drop_link(self):
        if self.broker:   # the robot's network fails: broker publishes its last will
            self.assertTrue(await self.broker.kick(f"rc-robot-{ROBOT_ID}-*"))
        else:
            await super().drop_link()


@unittest.skipUnless(HAVE_MQTT, "paho-mqtt not installed")
class MqttConnectorTests(_MqttBroker, _Conformance, unittest.IsolatedAsyncioTestCase):
    async def start_robot(self, controller):
        from remote_control.connectors.mqtt import MqttRobotConnector
        return await _PythonRobot(MqttRobotConnector(self.url)).start()


@unittest.skipUnless(HAVE_MQTT, "paho-mqtt not installed")
class MqttFromConfigTests(MqttConnectorTests):
    """Same conformance suite, but both sides built from config sections instead of URLs."""

    async def make_controller_connector(self):
        from remote_control import controller_connector_from_config
        await super().make_controller_connector()            # starts the broker, sets self.url
        from urllib.parse import urlparse
        u = urlparse(self.url)
        self.cfg = {"type": "mqtt", "host": u.hostname, "port": u.port, "prefix": u.path.strip("/")}
        return controller_connector_from_config({"connector": dict(self.cfg)})

    async def start_robot(self, controller):
        from remote_control import robot_connector_from_config
        return await _PythonRobot(robot_connector_from_config(dict(self.cfg))).start()


@unittest.skipUnless(HAVE_MQTT and shutil.which("dotnet") and os.path.isdir(DOTNET_PROJECT),
                     "needs paho-mqtt and the .NET SDK")
class CSharpMqttRobotTests(_StallTests, _MqttBroker, _Conformance, unittest.IsolatedAsyncioTestCase):
    @classmethod
    def setUpClass(cls):
        _DotnetRobot.build()

    async def start_robot(self, controller):
        return _DotnetRobot(self.url)


try:
    import rclpy  # noqa: F401
    HAVE_ROS2 = True
except ImportError:
    HAVE_ROS2 = False


@unittest.skipUnless(HAVE_ROS2, "needs rclpy (source ROS 2's local_setup first)")
class Ros2ConnectorTests(_Conformance, unittest.IsolatedAsyncioTestCase):
    async def make_controller_connector(self):
        from remote_control.connectors.ros2 import Ros2ControllerConnector
        self.url = f"ros2://rctest{uuid.uuid4().hex[:8]}"
        return Ros2ControllerConnector(self.url)

    async def start_robot(self, controller):
        from remote_control.connectors.ros2 import Ros2RobotConnector
        return await _PythonRobot(Ros2RobotConnector(self.url)).start()

    async def drop_link(self):   # the robot's ROS node disappears (crash / network) and comes back
        await self.robot_side.connector.simulate_drop(offline_for=0.3)


class _FakeRos2Robot:
    """A pretend ros2_control robot: tracks commands instantly and publishes /joint_states at 100 Hz."""

    def __init__(self, ns, initial, command_type):
        import threading
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray
        from trajectory_msgs.msg import JointTrajectory
        self.positions = dict(initial)
        self.commands = 0
        self.ctx = Context()
        rclpy.init(context=self.ctx)
        self.node = rclpy.create_node("fake_ros2_control", context=self.ctx)
        names = list(initial)
        if command_type == "position":
            def on_cmd(msg):
                self.commands += 1
                self.positions.update(zip(names, msg.data))
            self.node.create_subscription(Float64MultiArray, f"/{ns}/arm_position_controller/commands", on_cmd, 10)
        else:
            def on_traj(msg):
                self.commands += 1
                self.positions.update(zip(msg.joint_names, msg.points[-1].positions))
            self.node.create_subscription(JointTrajectory, f"/{ns}/arm_controller/joint_trajectory", on_traj, 10)
        pub = self.node.create_publisher(JointState, f"/{ns}/joint_states", 10)

        def publish():
            msg = JointState(name=list(self.positions), position=list(self.positions.values()))
            msg.header.stamp = self.node.get_clock().now().to_msg()
            pub.publish(msg)
        self.node.create_timer(0.01, publish)
        self._ex = SingleThreadedExecutor(context=self.ctx)
        self._ex.add_node(self.node)
        self._running = True
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        while self._running and self.ctx.ok():
            self._ex.spin_once(timeout_sec=0.02)

    def close(self):
        import rclpy
        self._running = False
        self._thread.join(2)
        self._ex.shutdown(timeout_sec=1)
        self.node.destroy_node()
        rclpy.shutdown(context=self.ctx)


@unittest.skipUnless(HAVE_ROS2, "needs rclpy (source ROS 2's local_setup first)")
class Ros2DriverTests(unittest.IsolatedAsyncioTestCase):
    async def _run(self, command_type):
        from remote_control.drivers.ros2 import Ros2JointDriver
        ns = f"rcdrv{uuid.uuid4().hex[:8]}"
        hw = _FakeRos2Robot(ns, {"shoulder": 0.0, "elbow": 0.0, "slide": 0.2}, command_type)
        topic = (f"/{ns}/arm_position_controller/commands" if command_type == "position"
                 else f"/{ns}/arm_controller/joint_trajectory")
        driver = Ros2JointDriver(JOINTS, topic, command_type=command_type, joint_state_topic=f"/{ns}/joint_states")
        name = f"drv{id(self)}"
        controller = MotionController(LoopbackControllerConnector(name), heartbeat_interval=0.1, heartbeat_timeout=0.5)
        runtime = RobotRuntime(LoopbackRobotConnector(name), driver, ROBOT_ID, tick_hz=100, decel_time=0.1)
        try:
            self.assertTrue(await asyncio.get_event_loop().run_in_executor(None, driver.wait_ready, 10))
            await controller.start()
            await runtime.start()
            robot = await controller.wait_for_robot(ROBOT_ID, timeout=5)
            self.assertAlmostEqual(robot.state["positions"]["slide"], 0.2)
            goal = await robot.execute(["shoulder", "elbow"], [([1.0, -0.5], 0.5)], report="points")
            result = await goal.result(timeout=5)
            self.assertEqual(result["status"], "succeeded")
            await asyncio.sleep(0.1)
            self.assertAlmostEqual(hw.positions["shoulder"], 1.0, places=3)
            self.assertAlmostEqual(hw.positions["elbow"], -0.5, places=3)
            self.assertAlmostEqual(hw.positions["slide"], 0.2, places=6)   # held at its measured position
            self.assertGreater(hw.commands, 20)                             # streamed every tick
            self.assertLess(goal.points_reached[0]["max_error"], 0.1)
        finally:
            await runtime.stop()
            await controller.stop()
            driver.close()
            hw.close()

    async def test_forward_position_controller(self):
        await self._run("position")

    async def test_joint_trajectory_topic(self):
        await self._run("trajectory")


async def _noop():
    return None


if __name__ == "__main__":
    unittest.main()
