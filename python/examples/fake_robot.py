"""
A pretend robot (perfect tracking) for trying the controller without Unity.

    python examples/fake_robot.py                                   # → ws://localhost:8765/motion
    python examples/fake_robot.py --url ws://192.168.1.10:8765/motion --id arm-02
"""
import argparse
import asyncio
import logging
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
if hasattr(sys.stdout, "reconfigure"):  # Windows consoles default to cp1252
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

from remote_control import FakeDriver, Joint, RobotRuntime, robot_transport_from_url  # noqa: E402

JOINTS = [
    Joint("shoulder_yaw", "revolute", -2.97, 2.97, 2.0),
    Joint("shoulder_pitch", "revolute", -1.75, 1.75, 2.0),
    Joint("elbow", "revolute", -2.44, 2.44, 2.0),
    Joint("wrist", "revolute", -3.14, 3.14, 2.0),
    Joint("gripper", "prismatic", 0.0, 0.05, 0.1),
]


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="ws://localhost:8765/motion")
    ap.add_argument("--id", default="fake-arm")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    runtime = RobotRuntime(robot_transport_from_url(args.url), FakeDriver(JOINTS), args.id, name="Fake arm")
    last = {}

    def on_state():
        st = runtime.executor.state_payload()
        key = (st["state"], st["goal_id"], st["pause_reason"])
        if key != last.get("k"):
            last["k"] = key
            print(f"state={st['state']} goal={st['goal_id']} reason={st['pause_reason']}")

    await runtime.start()
    print(f"fake robot '{args.id}' → {args.url}  (Ctrl+C to quit)")
    try:
        while True:
            await asyncio.sleep(0.05)
            on_state()
    finally:
        await runtime.stop()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
