"""
A pretend robot (perfect tracking) for trying the controller without Unity.

    python examples/fake_robot.py                                   # connector from %RC_CONFIG_DIR%\remote_control.json
    python examples/fake_robot.py --id arm-02
    python examples/fake_robot.py --url ws://192.168.1.10:8765/motion  # explicit override, ignores the file
"""
import argparse
import asyncio
import logging
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
if hasattr(sys.stdout, "reconfigure"):  # Windows consoles default to cp1252
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

from remote_control import (ConfigError, FakeDriver, Joint, RobotRuntime, load_system_config,  # noqa: E402
                            robot_connector_from_config, robot_connector_from_url, system_config_path)

JOINTS = [
    Joint("shoulder_yaw", "revolute", -2.97, 2.97, 2.0),
    Joint("shoulder_pitch", "revolute", -1.75, 1.75, 2.0),
    Joint("elbow", "revolute", -2.44, 2.44, 2.0),
    Joint("wrist", "revolute", -3.14, 3.14, 2.0),
    Joint("gripper", "prismatic", 0.0, 0.05, 0.1),
]


def describe(section):
    """One-line description of a connector config section, without secrets."""
    if section.get("url"):
        return section["url"]
    opts = ", ".join(f"{k}={v}" for k, v in section.items() if k not in ("type", "password") and v is not None)
    return f"{section.get('type')} ({opts})"


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default=None, help="connector URL, overriding the config file (%%RC_CONFIG_DIR%%\\remote_control.json)")
    ap.add_argument("--id", default="fake-arm", help="robot_id")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    if args.url:
        connector = robot_connector_from_url(args.url)
    else:
        try:
            config = load_system_config()
        except ConfigError as exc:
            print(f"config: {exc}\n(or pass --url ws://localhost:8765/motion)")
            return
        print(f"config: {system_config_path()}")
        connector = robot_connector_from_config(config)
        args.url = describe(config["connector"])
    runtime = RobotRuntime(connector, FakeDriver(JOINTS), args.id, name="Fake arm")
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
