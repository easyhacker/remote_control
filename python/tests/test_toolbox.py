"""
Robotic Toolbox tests without a GUI: inverse kinematics, and the backend (controller thread) against a Python
fake robot that carries the demo arm's URDF tree, over WebSocket.

    python -m unittest tests.test_toolbox -v
"""
import asyncio
import math
import os
import socket
import sys
import tempfile
import threading
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

try:
    import numpy as np
except ImportError:   # the toolbox needs numpy (pip install -e .[toolbox])
    raise unittest.SkipTest("numpy not installed")

from remote_control import FakeDriver, Joint, RobotRuntime, load_urdf_tree, robot_connector_from_url  # noqa: E402
from robotic_toolbox.backend import Backend  # noqa: E402
from robotic_toolbox.ik import Chain, matrix_rpy, pose_from_xyz_rpy, rpy_matrix  # noqa: E402

DEMO_URDF = os.path.join(HERE, "..", "examples", "demo_arm.urdf")
JOINTS = [
    Joint("shoulder_yaw", "revolute", -2.97, 2.97, 2.0),
    Joint("shoulder_pitch", "revolute", -1.75, 1.75, 2.0),
    Joint("elbow", "revolute", -2.44, 2.44, 2.0),
    Joint("wrist", "revolute", -3.14, 3.14, 2.0),
    Joint("gripper", "prismatic", 0.0, 0.05, 0.1),
]

try:
    import websockets  # noqa: F401
    HAVE_WS = True
except ImportError:
    HAVE_WS = False


class IkTests(unittest.TestCase):
    def setUp(self):
        self.tree = load_urdf_tree(DEMO_URDF)

    def test_rpy_round_trip_including_gimbal_lock(self):
        for rpy in [(0.1, 0.2, 0.3), (-2.0, 1.0, 3.0), (1.2, math.pi / 2, 0.0), (0.4, -math.pi / 2, 0.0)]:
            r = rpy_matrix(*rpy)
            np.testing.assert_allclose(rpy_matrix(*matrix_rpy(r)), r, atol=1e-9)

    def test_tools_and_bases(self):
        self.assertEqual(Chain.tool_candidates(self.tree), ["wrist_link"])
        self.assertEqual(Chain.ancestors(self.tree, "forearm"), ["forearm", "upper_arm", "yaw_link", "base_link"])
        with self.assertRaises(ValueError):
            Chain.from_tree(self.tree, "forearm", base="finger")

    def test_solves_reachable_poses(self):
        chain = Chain.from_tree(self.tree, "wrist_link")
        self.assertEqual(chain.joint_names, ["shoulder_yaw", "shoulder_pitch", "elbow", "wrist"])
        rng = np.random.default_rng(3)
        lo, hi = chain._limits()
        for _ in range(10):
            q = rng.uniform(lo, hi)
            goal = chain.tool_pose(dict(zip(chain.joint_names, q)))
            r = chain.solve(goal, {})
            self.assertTrue(r.reachable, r.message)
            np.testing.assert_allclose(chain.tool_pose(r.positions), goal, atol=2e-3)

    def test_reports_unreachable_and_position_only(self):
        chain = Chain.from_tree(self.tree, "wrist_link")
        far = pose_from_xyz_rpy((2.0, 0, 0.5), (0, 0, 0))
        r = chain.solve(far, {}, restarts=3)
        self.assertFalse(r.reachable)
        self.assertGreater(r.position_error, 0.5)
        self.assertIn("not reachable", r.message)
        # a point the arm reaches, with an orientation it cannot have (wrist pointing down while upright)
        p = pose_from_xyz_rpy((0.3, 0.0, 0.6), (0, 0, 0))
        self.assertTrue(chain.solve(p, {}, position_only=True).reachable)


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class _RobotThread:
    """Python fake robot (demo arm tree) on its own thread and event loop."""

    def __init__(self, url):
        self.url = url
        self.loop = asyncio.new_event_loop()
        self.ready = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        self.ready.wait(5)

    def _run(self):
        asyncio.set_event_loop(self.loop)
        connector = robot_connector_from_url(self.url)
        connector.min_backoff = 0.1
        self.runtime = RobotRuntime(connector, FakeDriver(JOINTS), "toolbox-arm", tick_hz=200, decel_time=0.1,
                                    project="Toolbox test", stage="bench", tree=load_urdf_tree(DEMO_URDF))
        self.loop.run_until_complete(self.runtime.start())
        self.ready.set()
        try:
            self.loop.run_forever()
        finally:
            self.loop.close()

    def positions(self):
        return dict(self.runtime.driver.positions)

    def stop(self):
        fut = asyncio.run_coroutine_threadsafe(self.runtime.stop(), self.loop)
        try:
            fut.result(5)
        finally:
            self.loop.call_soon_threadsafe(self.loop.stop)
            self.thread.join(5)


@unittest.skipUnless(HAVE_WS, "websockets not installed")
class BackendTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        url = f"ws://127.0.0.1:{free_port()}/motion"
        self.logs = []
        self.backend = Backend(post=lambda fn: fn(), url=url, data_dir=self.tmp.name)
        self.backend.on_log = self.logs.append
        self.backend.start()
        self.robot = _RobotThread(url)
        deadline = time.monotonic() + 10
        while (self.backend.description is None or not self.backend.positions) and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertIsNotNone(self.backend.description, f"robot never selected; log: {self.logs}")

    def tearDown(self):
        self.robot.stop()
        self.backend.stop()
        self.tmp.cleanup()

    def run_(self, coro, timeout=10):
        return self.backend.call(coro).result(timeout)

    def wait_goal(self, timeout=10):
        return asyncio.run_coroutine_threadsafe(self.backend.goal.result(timeout), self.backend.loop).result(timeout + 1)

    def test_selects_robot_and_loads_tree(self):
        self.assertEqual(self.backend.robot.names, {"project": "Toolbox test", "stage": "bench", "robot": "toolbox-arm"})
        self.assertEqual(self.backend.tools(), ["wrist_link"])
        self.assertEqual(self.backend.bases_for("wrist_link"), ["base_link", "yaw_link", "upper_arm", "forearm"])

    def test_jog_steps_add_up_and_respect_limits(self):
        self.run_(self.backend.jog_step("elbow", 0.2, speed=1.0))
        self.run_(self.backend.jog_step("elbow", 0.2, speed=1.0))      # queued: starts from the first target
        self.wait_goal()
        time.sleep(0.1)
        self.assertAlmostEqual(self.robot.positions()["elbow"], 0.4, places=3)
        self.run_(self.backend.jog_step("gripper", 1.0, speed=1.0))     # clamped to the 0.05 m limit
        self.wait_goal()
        self.assertAlmostEqual(self.robot.positions()["gripper"], 0.05, places=4)

    def test_continuous_jog_stops_on_release(self):
        self.run_(self.backend.jog_start("shoulder_yaw", +1, speed=0.5))
        time.sleep(0.6)
        self.run_(self.backend.jog_stop())
        self.assertEqual(self.wait_goal()["status"], "canceled")
        x = self.robot.positions()["shoulder_yaw"]
        self.assertGreater(x, 0.05)
        self.assertLess(x, 2.97)

    def test_poses(self):
        self.run_(self.backend.move_joints({"shoulder_pitch": 0.5, "elbow": -0.4}, duration=0.5))
        self.wait_goal()
        self.run_(self.backend.save_pose("ready"))
        self.assertEqual([p["name"] for p in self.backend.list_poses()], ["home", "ready"])
        self.run_(self.backend.move_joints({"shoulder_pitch": 0.0, "elbow": 0.0}, duration=0.5))
        self.wait_goal()
        self.run_(self.backend.go_to_pose("ready", speed=1.0))
        self.assertEqual(self.wait_goal()["status"], "succeeded")
        pos = self.robot.positions()
        self.assertAlmostEqual(pos["shoulder_pitch"], 0.5, places=3)
        self.assertAlmostEqual(pos["elbow"], -0.4, places=3)
        self.backend.delete_pose("ready")
        self.assertEqual([p["name"] for p in self.backend.list_poses()], ["home"])

    def test_builtin_home_pose(self):
        poses = self.backend.list_poses()
        self.assertEqual(poses[0]["name"], "home")
        self.assertTrue(poses[0]["builtin"])
        with self.assertRaises(ValueError):
            self.backend.delete_pose("home")
        self.run_(self.backend.move_joints({"elbow": 0.8, "gripper": 0.03}, duration=0.5))
        self.wait_goal()
        self.run_(self.backend.go_to_pose("home", speed=1.0))
        self.assertEqual(self.wait_goal()["status"], "succeeded")
        pos = self.robot.positions()
        self.assertAlmostEqual(pos["elbow"], 0.0, places=3)
        self.assertAlmostEqual(pos["gripper"], 0.0, places=4)          # 0 is its lower limit
        # a saved 'home' replaces the built-in one
        self.run_(self.backend.move_joints({"elbow": 0.3}, duration=0.5))
        self.wait_goal()
        self.run_(self.backend.save_pose("home"))
        self.assertFalse(self.backend.list_poses()[0]["builtin"])
        self.run_(self.backend.move_joints({"elbow": 0.0}, duration=0.5))
        self.wait_goal()
        self.run_(self.backend.go_to_pose("home", speed=1.0))
        self.wait_goal()
        self.assertAlmostEqual(self.robot.positions()["elbow"], 0.3, places=3)
        self.backend.delete_pose("home")                               # back to the built-in one
        self.assertTrue(self.backend.list_poses()[0]["builtin"])

    def test_check_and_move_to_target(self):
        chain = self.backend.chain("wrist_link")
        goal_q = {"shoulder_yaw": 0.6, "shoulder_pitch": 0.7, "elbow": 0.5, "wrist": -0.3}
        from robotic_toolbox.ik import xyz_rpy_from_pose
        xyz, rpy = xyz_rpy_from_pose(chain.tool_pose(goal_q))
        r = self.run_(self.backend.check_target_async("wrist_link", "base_link", xyz, rpy, False))
        self.assertTrue(r.reachable, r.message)
        r = self.run_(self.backend.move_to_target("wrist_link", "base_link", xyz, rpy, False, speed=1.0), timeout=20)
        self.assertTrue(r.reachable)
        self.assertEqual(self.wait_goal()["status"], "succeeded")
        time.sleep(0.3)   # next position poll
        now_xyz, _ = self.backend.tool_pose("wrist_link", "base_link")
        np.testing.assert_allclose(now_xyz, xyz, atol=2e-3)
        far = self.run_(self.backend.move_to_target("wrist_link", None, (3.0, 0, 0), (0, 0, 0), True), timeout=30)
        self.assertFalse(far.reachable)


if __name__ == "__main__":
    unittest.main()
