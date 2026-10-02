"""
Conformance tests: controller ↔ robot behaviour, run identically over every transport and robot
implementation:
  - LoopbackTransportTests   Python robot, in-process transport
  - WebSocketTransportTests  Python robot over WebSocket
  - CSharpRobotTests         the Unity package's C# core (built with plain .NET) over WebSocket

Run from the python/ folder:   python -m unittest discover -s tests -v
The C# tests need the .NET SDK; they build unity/Packages/com.robotmarket.remote-control/Tests~/DotnetRobot.
"""
import asyncio
import os
import shutil
import subprocess
import sys
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from remote_control import (FakeDriver, GoalRejected, Joint, MotionController, RobotRuntime,  # noqa: E402
                            Trajectory)
from remote_control.protocol import SeqTracker  # noqa: E402
from remote_control.transports.loopback import (LoopbackControllerTransport,  # noqa: E402
                                                LoopbackRobotTransport)

JOINTS = [   # the C# harness (Tests~/DotnetRobot/Program.cs) uses the same joints
    Joint("shoulder", "revolute", -3.0, 3.0, 4.0),
    Joint("elbow", "revolute", -2.0, 2.0, 4.0),
    Joint("slide", "prismatic", 0.0, 0.5, 1.0),
]
ROBOT_ID = "arm-test"
DOTNET_PROJECT = os.path.normpath(os.path.join(
    HERE, "..", "..", "unity", "Packages", "com.robotmarket.remote-control", "Tests~", "DotnetRobot"))


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


class _Conformance:
    """Mixed into a TestCase per transport / robot implementation.

    Positions are read from protocol messages (results, feedback, state) only, so the tests work
    for robots running in another process or language.
    """

    async def start_robot(self, controller):
        """Start the robot side; return an object with an async stop()."""
        raise NotImplementedError

    async def make_controller_transport(self):
        raise NotImplementedError

    async def drop_link(self):
        """Break the connection; the robot must reconnect by itself."""
        await self.robot._link.close()

    async def asyncSetUp(self):
        self.controller = MotionController(await self.make_controller_transport(),
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

    # ── tests ────────────────────────────────────────────────────────────────

    async def test_hello_describes_robot(self):
        self.assertEqual(self.robot.joint_names, ["shoulder", "elbow", "slide"])
        self.assertEqual(self.robot.joints[2].type, "prismatic")
        self.assertEqual(self.robot.joints[0].upper, 3.0)
        self.assertTrue(self.robot.supports["pause"])
        self.assertEqual(self.robot.state["state"], "idle")

    async def test_execute_reports_each_point_and_succeeds(self):
        goal = await self.robot.execute(["shoulder", "elbow"],
                                        [([0.5, -0.3], 0.3), ([1.0, 0.2], 0.6)], report="all", progress_hz=20)
        result = await goal.result(timeout=3)
        self.assertEqual(result["status"], "succeeded")
        self.assertEqual([p["point_index"] for p in goal.points_reached], [0, 1])
        self.assertLess(goal.points_reached[0]["max_error"], 0.05)  # measured on the tick that passes the point
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
        await asyncio.sleep(0.05)
        self.assertFalse(self.robot.online)
        robot = await self.controller.wait_for_robot(ROBOT_ID, timeout=5)
        self.assertIs(robot, self.robot)
        await self.wait_state("paused")
        self.assertEqual(robot.state["goal_id"], goal.goal_id)
        self.assertEqual(robot.state["pause_reason"], "connection_lost")
        self.assertLess(self.pos("shoulder"), 2.0)
        await goal.resume()
        self.assertEqual((await goal.result(timeout=3))["status"], "succeeded")


class _PythonRobot:
    def __init__(self, transport):
        self.transport = transport
        self.runtime = RobotRuntime(transport, FakeDriver(JOINTS), ROBOT_ID, tick_hz=200, decel_time=0.1)

    async def start(self):
        await self.runtime.start()
        return self

    async def stop(self):
        await self.runtime.stop()


class LoopbackTransportTests(_Conformance, unittest.IsolatedAsyncioTestCase):
    async def make_controller_transport(self):
        self.name = f"t{id(self)}"
        return LoopbackControllerTransport(self.name)

    async def start_robot(self, controller):
        return await _PythonRobot(LoopbackRobotTransport(self.name, reconnect_delay=0.05)).start()

    async def drop_link(self):
        self.robot_side.transport.simulate_drop(offline_for=0.2)


try:
    import websockets  # noqa: F401
    HAVE_WS = True
except ImportError:
    HAVE_WS = False


class _WebSocketController:
    async def make_controller_transport(self):
        from remote_control.transports.websocket import WebSocketControllerTransport
        ctrl = WebSocketControllerTransport("127.0.0.1", 0, "/motion")
        await ctrl.start()          # bind now to learn the port …
        ctrl.start = _noop          # … so MotionController.start() doesn't bind again
        self.url = f"ws://127.0.0.1:{ctrl.bound_port}/motion"
        return ctrl


@unittest.skipUnless(HAVE_WS, "websockets not installed")
class WebSocketTransportTests(_WebSocketController, _Conformance, unittest.IsolatedAsyncioTestCase):
    async def start_robot(self, controller):
        from remote_control.transports.websocket import WebSocketRobotTransport
        return await _PythonRobot(WebSocketRobotTransport(self.url, min_backoff=0.1)).start()


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
        self.proc = subprocess.Popen(["dotnet", self.build(), url, ROBOT_ID], stdin=subprocess.PIPE,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

    async def stop(self):
        self.proc.stdin.close()   # harness exits when stdin closes
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()


@unittest.skipUnless(HAVE_WS and shutil.which("dotnet") and os.path.isdir(DOTNET_PROJECT),
                     "needs websockets and the .NET SDK")
class CSharpRobotTests(_WebSocketController, _Conformance, unittest.IsolatedAsyncioTestCase):
    @classmethod
    def setUpClass(cls):
        _DotnetRobot.build()   # once, outside the event loop

    async def start_robot(self, controller):
        return _DotnetRobot(self.url)


async def _noop():
    return None


if __name__ == "__main__":
    unittest.main()
