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

from remote_control import (FakeDriver, Joint, RobotRuntime, forward_kinematics, load_urdf_tree,  # noqa: E402
                            robot_connector_from_url)
from robotic_toolbox.backend import Backend  # noqa: E402
from robotic_toolbox.ik import Chain, matrix_rpy, pose_from_xyz_rpy, rpy_matrix, xyz_rpy_from_pose  # noqa: E402

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


class _SceneTargets:
    """A robot-side target store like Unity's: targets are scene objects with a world pose."""

    def __init__(self, runtime_getter, base):
        self.runtime_getter = runtime_getter
        self.base = base          # 4x4 pose of the robot's root link in the scene
        self.targets = {}         # id -> 4x4 world pose

    def handle(self, req):
        from robotic_toolbox.backend import pose_to_xyz_rpy
        op = req.get("op")
        if op in ("settings", "select"):
            return None
        if op == "delete":
            return None if self.targets.pop(req["id"], None) is not None else f"no target '{req['id']}'"
        t = pose_from_xyz_rpy(*pose_to_xyz_rpy(req["pose"]))
        ref = req.get("reference", "scene")
        if ref.startswith("link:"):
            rt = self.runtime_getter()
            poses = forward_kinematics(rt.tree, rt.driver.read_positions())
            link = poses[ref[5:]]
            t = self.base @ _matrix(link) @ t
        elif ref in ("robot",):
            t = self.base @ t           # this fake robot instance sits at its root link
        if op == "create":
            self.targets[f"target:{req.get('parent') or 'Targets'}/{req['name']}"] = t
        elif op == "update":
            if req["id"] not in self.targets:
                return f"no target '{req['id']}'"
            self.targets[req["id"]] = t
        return None

    def list(self):
        from robotic_toolbox.backend import _pose
        out = []
        for tid, t in self.targets.items():
            path = tid[len("target:"):]
            root = np.linalg.inv(self.base) @ t
            out.append({"id": tid, "name": path.split("/")[-1], "path": path,
                        "pose_in_scene": _pose(*xyz_rpy_from_pose(t)),
                        "pose_in_root": _pose(*xyz_rpy_from_pose(root))})
        return out


def _matrix(pose):
    from robotic_toolbox.backend import pose_to_xyz_rpy
    return pose_from_xyz_rpy(*pose_to_xyz_rpy(pose))


class _RobotThread:
    """Python fake robot (demo arm tree) on its own thread and event loop."""

    def __init__(self, url, base=None, scene_targets=False):
        self.url = url
        self.scene_targets = scene_targets     # True: owns its targets like Unity (supports.targets)
        self.base = base if base is not None else np.eye(4)
        self.scene = _SceneTargets(lambda: self.runtime, self.base)
        self.loop = asyncio.new_event_loop()
        self.ready = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        self.ready.wait(5)

    def _run(self):
        asyncio.set_event_loop(self.loop)
        connector = robot_connector_from_url(self.url)
        connector.min_backoff = 0.1
        self.shown = []
        from robotic_toolbox.backend import _pose
        self.runtime = RobotRuntime(connector, FakeDriver(JOINTS), "toolbox-arm", tick_hz=200, decel_time=0.1,
                                    project="Toolbox test", stage="bench", tree=load_urdf_tree(DEMO_URDF),
                                    visualizer=self.shown.append, base_pose=_pose(*xyz_rpy_from_pose(self.base)),
                                    target_handler=self.scene.handle if self.scene_targets else None,
                                    target_lister=self.scene.list if self.scene_targets else None)
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
        tools = self.backend.tools()
        self.assertEqual(tools[0], "wrist_link")               # chain ends first …
        self.assertIn("finger", tools)                          # … then every other link
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

    def test_joint_groups_and_group_poses(self):
        b = self.backend
        b.save_group("arm", ["shoulder_yaw", "shoulder_pitch", "elbow", "wrist"])
        b.save_group("gripper", ["gripper"])
        saved = [g for g in b.groups() if not g["builtin"]]
        self.assertEqual([(g["name"], g["joints"]) for g in saved],
                         [("arm", ["shoulder_yaw", "shoulder_pitch", "elbow", "wrist"]), ("gripper", ["gripper"])])
        with self.assertRaises(Exception):
            b.save_group("bad", ["no_such_joint"])
        # a group pose stores only the group's joints, and moves only them
        self.run_(b.move_joints({"gripper": 0.04, "elbow": 0.2}, duration=0.8))
        self.wait_goal()
        self.run_(b.save_pose("close_gripper", group="gripper"))
        pose = next(p for p in b.list_poses() if p["name"] == "close_gripper")
        self.assertEqual((pose["group"], pose["joints"]), ("gripper", 1))
        self.run_(b.move_joints({"gripper": 0.0}, duration=0.8))
        self.wait_goal()
        # the arm moves slowly while the gripper pose runs in parallel and finishes first
        arm = self.run_(b.move_joints({"elbow": 1.0}, duration=1.5))
        t0 = time.monotonic()
        grip = self.run_(b.go_to_pose("close_gripper", speed=1.0))
        self.assertEqual(self.run_(grip.result(5))["status"], "succeeded")
        self.assertLess(time.monotonic() - t0, 1.2)                          # did not wait for the arm
        self.assertFalse(arm.done)
        self.assertAlmostEqual(self.robot.positions()["gripper"], 0.04, places=4)
        # stopping one group leaves the other moving
        self.run_(b.move_joints({"gripper": 0.0}, duration=1.0))
        self.run_(b.cancel(b.group_joints("gripper")))
        self.assertFalse(arm.done)
        self.assertEqual(self.run_(arm.result(5))["status"], "succeeded")
        self.assertAlmostEqual(self.robot.positions()["elbow"], 1.0, places=3)
        # home for one group only
        self.run_(b.go_home("arm", speed=1.0))
        self.wait_goal()
        pos = self.robot.positions()
        self.assertAlmostEqual(pos["elbow"], 0.0, places=3)
        self.assertGreater(pos["gripper"], 0.0)                              # untouched
        b.delete_group("gripper")
        self.assertNotIn("gripper", [g["name"] for g in b.groups() if not g["builtin"]])

    def test_suggested_groups_from_chains(self):
        self.backend.save_chain("arm chain", "base_link", "wrist_link")
        g = next(g for g in self.backend.groups() if g["name"] == "arm chain")
        self.assertTrue(g["builtin"])
        self.assertEqual(g["joints"], ["shoulder_yaw", "shoulder_pitch", "elbow", "wrist"])

    def test_tcp_moves_the_tool_point(self):
        xyz0, _ = self.backend.tool_pose("wrist_link", "base_link")
        self.backend.set_tcp("wrist_link", (0, 0, 0.1), (0, 0, 0))
        xyz1, _ = self.backend.tool_pose("wrist_link", "base_link")
        np.testing.assert_allclose(np.subtract(xyz1, xyz0), (0, 0, 0.1), atol=1e-9)   # arm points up at zero
        self.assertEqual(self.backend.tcp("wrist_link"), ((0.0, 0.0, 0.1), (0.0, 0.0, 0.0)))
        self.backend.set_tcp("wrist_link", (0, 0, 0), (0, 0, 0))                         # back to the link origin
        self.assertNotIn("wrist_link", self.backend._doc("tcp.json", "tcp"))

    def test_named_chains_get_their_own_tcp(self):
        self.backend.set_tcp("wrist_link", (0, 0, 0.05), (0, 0, 0))          # earlier, unsaved TCP for that end
        self.backend.save_chain("arm", "base_link", "wrist_link")
        chain = self.backend.chains()["arm"]
        self.assertEqual((chain["origin"], chain["end"]), ("base_link", "wrist_link"))
        self.assertEqual(chain["tcp"]["xyz"], [0, 0, 0.05])                  # TCP created from it
        self.backend.save_chain("forearm only", "upper_arm", "wrist_link")
        self.backend.set_chain_tcp("forearm only", (0.1, 0, 0), (0, 0, 0))
        xyz_arm, _ = self.backend.tool_pose("wrist_link", "base_link", "arm")
        xyz_fa, _ = self.backend.tool_pose("wrist_link", "base_link", "forearm only")
        np.testing.assert_allclose(np.subtract(xyz_fa, xyz_arm), (0.1, 0, -0.05), atol=1e-9)   # each chain its TCP
        self.backend.save_chain("arm", "yaw_link", "wrist_link")             # replace: same end keeps the TCP
        self.assertEqual(self.backend.chains()["arm"]["tcp"]["xyz"], [0, 0, 0.05])
        self.assertEqual(self.backend.chains()["arm"]["origin"], "yaw_link")
        with self.assertRaises(ValueError):
            self.backend.save_chain("bad", "finger", "wrist_link")            # origin not above the end
        items = {i["id"]: i for i in self.backend.marker_items("wrist_link", "yaw_link", chain_name="arm")}
        self.assertEqual(items["tcp:arm"]["label"], "TCP arm")
        self.assertEqual(items["chain:arm"]["links"], ["yaw_link", "upper_arm", "forearm", "wrist_link"])
        self.backend.delete_chain("arm")
        self.assertEqual(list(self.backend.chains()), ["forearm only"])

    def test_tcp_and_frames_moved_in_the_viewer(self):
        from robotic_toolbox.backend import object_name
        self.assertEqual(object_name("TCP", "right arm"), "TCP_right_arm")
        self.backend.save_chain("right arm", "base_link", "wrist_link")
        self.backend.save_frame("pick", "base_link", (0.3, 0, 0.4), (0, 0, 0))
        items = {i["id"]: i for i in self.backend.marker_items("wrist_link", "base_link", chain_name="right arm")}
        self.assertEqual(items["tcp:right arm"]["name"], "TCP_right_arm")
        self.assertTrue(items["tcp:right arm"]["editable"])
        self.assertEqual(items["frame:pick"]["name"], "Frame_pick")
        self.assertEqual(items["chain:right arm"]["name"], "Chain_right_arm")
        focused = [i for i in self.backend.marker_items("wrist_link", "base_link", chain_name="right arm",
                                                        focus="tcp:right arm") if i.get("focus")]
        self.assertEqual([i["id"] for i in focused], ["tcp:right arm"])      # "Edit in Unity"

        edits = []
        self.backend.on_edited = edits.append
        # the user drags the TCP 4 cm along the link's z and turns it 90° about x (quaternion of rx = 90°)
        pose = {"position": [0.0, 0.0, 0.04], "orientation": [0.7071068, 0.0, 0.0, 0.7071068]}
        self.robot.loop.call_soon_threadsafe(self.robot.runtime.edited, "tcp:right arm", "wrist_link", pose, "editor")
        deadline = time.monotonic() + 3
        while not edits and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertEqual(edits[0]["id"], "tcp:right arm")
        self.assertEqual(self.backend.apply_edit(edits[0], "wrist_link", "right arm"), "tcp")
        tcp = self.backend.chains()["right arm"]["tcp"]
        np.testing.assert_allclose(tcp["xyz"], (0, 0, 0.04), atol=1e-6)
        np.testing.assert_allclose(tcp["rpy"], (math.pi / 2, 0, 0), atol=1e-5)
        # a frame moved in the viewer is saved too
        moved = {"id": "frame:pick", "parent": "base_link",
                 "pose": {"position": [0.25, 0.1, 0.4], "orientation": [0, 0, 0, 1]}}
        self.assertEqual(self.backend.apply_edit(moved, "wrist_link", "right arm"), "frame:pick")
        np.testing.assert_allclose(self.backend.frames()["pick"]["xyz"], (0.25, 0.1, 0.4))
        self.assertIsNone(self.backend.apply_edit({"id": "origin:base_link", "pose": {}}, "wrist_link", "right arm"))

    def test_frames_from_tcp_and_as_targets(self):
        self.backend.set_tcp("wrist_link", (0.05, 0, 0), (0, 0, 0))
        self.run_(self.backend.move_joints({"shoulder_yaw": 0.4, "shoulder_pitch": 0.6, "elbow": 0.4}, duration=0.5))
        self.wait_goal()
        time.sleep(0.3)   # position poll
        self.backend.frame_from_tcp("pick", "wrist_link", "base_link")
        self.assertEqual(list(self.backend.frames()), ["pick"])
        self.assertEqual(self.backend.frames()["pick"]["parent"], "base_link")
        # the frame expressed in another link of the chain is the same point
        xyz_b, rpy_b = self.backend.frame_in("pick", "base_link")
        np.testing.assert_allclose(xyz_b, self.backend.tool_pose("wrist_link", "base_link")[0], atol=1e-9)
        self.run_(self.backend.move_joints({"shoulder_yaw": 0.0, "shoulder_pitch": 0.0, "elbow": 0.0}, duration=0.5))
        self.wait_goal()
        time.sleep(0.3)
        r = self.run_(self.backend.move_to_target("wrist_link", "base_link", xyz_b, rpy_b, False, speed=1.0),
                      timeout=20)
        self.assertTrue(r.reachable, r.message)
        self.wait_goal()
        time.sleep(0.3)
        np.testing.assert_allclose(self.backend.tool_pose("wrist_link", "base_link")[0], xyz_b, atol=2e-3)
        self.backend.delete_frame("pick")
        self.assertEqual(self.backend.frames(), {})

    def test_markers_and_selection_from_viewer(self):
        self.assertTrue(self.backend.can_visualize)
        self.backend.save_frame("drop", "base_link", (0.3, 0.1, 0.4), (0, 0, 0))
        self.assertTrue(self.run_(self.backend.show_markers("wrist_link", "base_link", selected="drop")))
        items = {i["id"]: i for i in self.robot.shown[-1]["items"]}
        self.assertEqual(items["chain:wrist_link"]["links"],
                         ["base_link", "yaw_link", "upper_arm", "forearm", "wrist_link"])
        self.assertEqual(items["tcp:wrist_link"]["parent"], "wrist_link")
        self.assertTrue(items["frame:drop"]["selectable"])
        self.assertEqual(items["frame:drop"]["style"], "target")
        np.testing.assert_allclose(items["frame:drop"]["pose"]["position"], (0.3, 0.1, 0.4))
        picked = []
        self.backend.on_selected = picked.append
        self.robot.loop.call_soon_threadsafe(self.robot.runtime.select, "frame:drop", "click")
        deadline = time.monotonic() + 3
        while not picked and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertEqual(picked, ["frame:drop"])

    def test_scene_targets_and_references(self):
        # the robot stands at (1, 2, 0) in the scene, turned 90° left: scene, robot and chain coordinates differ
        self.robot.stop()
        self.robot = _RobotThread(self.backend._url, base=pose_from_xyz_rpy((1, 2, 0), (0, 0, math.pi / 2)),
                                  scene_targets=True)
        deadline = time.monotonic() + 10
        while not (self.backend.uses_scene_targets
                   and self.backend.live.get("base_pose", {}).get("position", [0])[0]) and time.monotonic() < deadline:
            time.sleep(0.05)
        b = self.backend
        self.assertTrue(b.uses_scene_targets)
        np.testing.assert_allclose(b.convert((1.3, 2.0, 0.5), (0, 0, 0), "scene", "chain", "base_link")[0],
                                   (0.0, -0.3, 0.5), atol=1e-9)        # 0.3 m along scene x = robot's right
        tid = self.run_(b.create_target("pick", "scene", (1.3, 2.0, 0.5), (0, 0, 0), "base_link"))
        self.assertEqual(tid, "target:Targets/pick")
        deadline = time.monotonic() + 3
        while tid not in b.scene_targets() and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertEqual(b.targets_list(), [(tid, "pick")])
        np.testing.assert_allclose(b.target_pose(tid, "scene", "base_link")[0], (1.3, 2.0, 0.5), atol=1e-6)
        np.testing.assert_allclose(b.target_pose(tid, "chain", "base_link")[0], (0.0, -0.3, 0.5), atol=1e-6)
        np.testing.assert_allclose(b.target_pose(tid, "chain", "upper_arm")[0], (0.0, -0.3, 0.35), atol=1e-6)
        # created in chain coordinates (origin upper_arm, 0.15 m above the base) -> same scene point
        tid2 = self.run_(b.create_target("place", "chain", (0.0, -0.3, 0.35), (0, 0, 0), "upper_arm"))
        deadline = time.monotonic() + 3
        while tid2 not in b.scene_targets() and time.monotonic() < deadline:
            time.sleep(0.05)
        np.testing.assert_allclose(b.target_pose(tid2, "scene", "base_link")[0], (1.3, 2.0, 0.5), atol=1e-6)
        # reachable (position only), and moving there puts the wrist on it
        xyz, rpy = b.target_pose(tid, "chain", "base_link")
        r = self.run_(b.move_to_target("wrist_link", "base_link", xyz, rpy, True, speed=1.0), timeout=20)
        self.assertTrue(r.reachable, r.message)
        self.wait_goal()
        time.sleep(0.3)
        np.testing.assert_allclose(b.tool_pose("wrist_link", "base_link")[0], xyz, atol=2e-3)
        self.run_(b.delete_target(tid))
        deadline = time.monotonic() + 3
        while tid in b.scene_targets() and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertEqual([t for t, _ in b.targets_list()], [tid2])

    def test_check_says_whether_position_or_orientation_fails(self):
        # the wrist cannot face straight down while the 4-joint demo arm keeps it above the base: orientation fails
        r = self.backend.check_target("wrist_link", "base_link", (0.3, 0.0, 0.6), (math.pi, 0, 0), False)
        self.assertFalse(r.reachable)
        self.assertTrue(r.position_reachable)
        far = self.backend.check_target("wrist_link", "base_link", (3.0, 0.0, 0.6), (0, 0, 0), False)
        self.assertFalse(far.position_reachable)
        self.assertIsNone(self.backend.check_target("wrist_link", "base_link", (3.0, 0, 0.6), (0, 0, 0), True)
                          .position_reachable)                            # position-only checks don't add it

    def test_suggested_target_names(self):
        b = self.backend
        self.assertEqual(b.suggest_target_name("L claw"), "L_claw_1")
        self.assertEqual(b.suggest_target_name(""), "target_1")
        b.save_frame("L_claw_1", "base_link", (0.3, 0, 0.4), (0, 0, 0))
        b.save_frame("L_claw_2", "base_link", (0.3, 0, 0.5), (0, 0, 0))
        self.assertEqual(b.suggest_target_name("L_claw"), "L_claw_3")
        b.delete_frame("L_claw_1")
        self.assertEqual(b.suggest_target_name("L_claw"), "L_claw_1")   # lowest free number

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
