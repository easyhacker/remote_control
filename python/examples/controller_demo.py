"""
Interactive controller: waits for a robot, then lets you send goals and interrupt them.

    python examples/controller_demo.py                       # connector from %RC_CONFIG_DIR%\remote_control.json
    python examples/controller_demo.py --url ws://0.0.0.0:9000/motion      # explicit override, ignores the file
    python examples/controller_demo.py --script              # run a scripted demo and exit (CI / smoke test)

Keys (type + Enter):
    g  go: send the demo sequence (all joints, timed poses)
    h  home: all joints to 0
    p  pause      r  resume      c  cancel the current goal      s  stop everything
    i  robot info / state        q  quit
"""
import argparse
import asyncio
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
if hasattr(sys.stdout, "reconfigure"):  # Windows consoles default to cp1252
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

from remote_control import (ConfigError, GoalRejected, MotionController,  # noqa: E402
                            controller_connector_from_config, controller_connector_from_url,
                            load_system_config, system_config_path)


def describe(section):
    """One-line description of a connector config section, without secrets."""
    if section.get("url"):
        return section["url"]
    opts = ", ".join(f"{k}={v}" for k, v in section.items() if k not in ("type", "password") and v is not None)
    return f"{section.get('type')} ({opts})"


def clamp(joint, x):
    if joint.lower is not None:
        x = max(joint.lower, x)
    if joint.upper is not None:
        x = min(joint.upper, x)
    return x


def _timed(robot, poses, min_segment=1.5):
    """Give each pose a time_from_start: at least min_segment per move, and slow enough for every joint's
    max_velocity (the robot checks 1.5 × average speed against it, so leave that margin plus 10 %)."""
    out, t = [], 0.0
    prev = [0.0 if j.lower is None or j.upper is None else clamp(j, 0.0) for j in robot.joints]
    for pose in poses:
        need = min_segment
        for j, a, b in zip(robot.joints, prev, pose):
            if j.max_velocity:
                need = max(need, 1.5 * abs(b - a) / j.max_velocity * 1.1)
        t += need
        out.append((pose, round(t, 3)))
        prev = pose
    return out


def demo_points(robot, scale=0.6):
    """A few timed poses that use each joint within its limits."""
    names = robot.joint_names
    poses = []
    for k, phase in enumerate([0.0, 1.0, -0.8, 0.5, 0.0]):
        pose = []
        for i, j in enumerate(robot.joints):
            span = (j.upper - j.lower) / 2 if j.lower is not None and j.upper is not None else 1.0
            mid = (j.upper + j.lower) / 2 if j.lower is not None and j.upper is not None else 0.0
            if j.type == "prismatic":
                x = mid + span * (0.8 if k % 2 else -0.8)
            else:
                x = mid + span * scale * phase * math.cos(i * 0.9) * (0.5 if span > math.pi else 1.0)
            pose.append(round(clamp(j, x), 4))
        poses.append(pose)
    return names, _timed(robot, poses)


def home_points(robot):
    return robot.joint_names, _timed(robot, [[clamp(j, 0.0) for j in robot.joints]], min_segment=2.0)


def print_event(ev):
    kind = ev["event"]
    if kind == "feedback":
        return
    detail = {k: v for k, v in ev.items() if k not in ("event", "positions")}
    pos = ev.get("positions")
    if isinstance(pos, list):
        detail["positions"] = [round(x, 3) for x in pos]
    print(f"  ← {kind} {detail}")


async def interactive(controller):
    loop = asyncio.get_event_loop()
    keys: "asyncio.Queue[str]" = asyncio.Queue()

    def read_stdin():
        for line in sys.stdin:
            loop.call_soon_threadsafe(keys.put_nowait, line.strip().lower())
        loop.call_soon_threadsafe(keys.put_nowait, "q")

    threading.Thread(target=read_stdin, daemon=True).start()
    goal = None
    print("keys: g go · h home · p pause · r resume · c cancel · s stop · i info · q quit")
    while True:
        key = await keys.get()
        robot = next((r for r in controller.robots.values() if r.online), None)
        if key == "q":
            return
        if robot is None:
            print("no robot online yet")
            continue
        try:
            if key in ("g", "h"):
                names, pts = demo_points(robot) if key == "g" else home_points(robot)
                goal = await robot.execute(names, pts, report="points")
                goal.on("*", print_event)
                print(f"→ goal {goal.goal_id} accepted (queue position {goal.queue_position})")
            elif key in ("p", "r", "c"):
                if goal is None:
                    print("no goal yet")
                    continue
                fn = {"p": goal.pause, "r": goal.resume, "c": goal.cancel}[key]
                print("  ack", await fn())
            elif key == "s":
                print("  ack", await robot.stop())
            elif key == "i":
                print(f"  {robot.name} ({robot.robot_id}) joints={robot.joint_names}")
                print(f"  state={robot.state}")
        except GoalRejected as exc:
            print(f"  rejected: {exc.reason}")
        except Exception as exc:  # keep the prompt alive
            print(f"  error: {exc}")


async def scripted(robot):
    """Exercise execute / pause / resume / cancel / stop against a real robot; exit code 0 on success."""
    ok = True

    def check(name, cond):
        nonlocal ok
        print(("PASS " if cond else "FAIL ") + name)
        ok = ok and bool(cond)

    names, pts = demo_points(robot)
    goal = await robot.execute(names, pts[:2], report="all", progress_hz=10)
    goal.on("*", print_event)
    r = await goal.result(timeout=30)
    check("demo goal succeeded", r["status"] == "succeeded")
    check("point reports", [p["point_index"] for p in goal.points_reached] == [0, 1])
    err = max(p["max_error"] for p in goal.points_reached)
    print(f"     max tracking error at points: {err:.4f}")
    check("tracking error < 0.1", err < 0.1)

    goal = await robot.execute(*home_points(robot), report="all", progress_hz=20)
    await asyncio.sleep(0.6)
    check("pause ack", (await goal.pause())["ok"])
    # "paused" means the timeline has stopped; a physical arm then needs a moment to settle onto the held pose
    deadline = time.monotonic() + 5.0
    while robot.state.get("state") != "paused" and time.monotonic() < deadline:
        await asyncio.sleep(0.02)
    check("reaches paused state", robot.state.get("state") == "paused")
    await asyncio.sleep(0.3)
    held = goal.last_feedback["positions"]
    await asyncio.sleep(0.5)
    drift = max(abs(a - b) for a, b in zip(held, goal.last_feedback["positions"]))
    check(f"holds while paused (drift {drift:.4f})", drift < 0.01)
    check("resume ack", (await goal.resume())["ok"])
    check("home succeeded", (await goal.result(timeout=30))["status"] == "succeeded")

    first = await robot.execute(names, pts[:2])
    second = await robot.execute(*home_points(robot))
    await asyncio.sleep(0.5)
    await first.cancel()
    check("cancel → canceled", (await first.result(timeout=10))["status"] == "canceled")
    check("queued goal then runs", (await second.result(timeout=30))["status"] == "succeeded")

    goal = await robot.execute(names, pts[:2])
    await asyncio.sleep(0.5)
    await robot.stop()
    check("stop → stopped", (await goal.result(timeout=10))["status"] == "stopped")

    try:
        bad = [[(j.upper or 0) + 1.0 for j in robot.joints]]
        await robot.execute(names, [(bad[0], 1.0)])
        check("out-of-limit goal rejected", False)
    except GoalRejected as exc:
        check(f"out-of-limit goal rejected ({exc.reason})", True)
    return ok


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default=None, help="connector URL, overriding the config file (%%RC_CONFIG_DIR%%\\remote_control.json)")
    ap.add_argument("--robot", default=None, help="robot_id to wait for (default: first to connect)")
    ap.add_argument("--script", action="store_true", help="run the scripted test and exit")
    ap.add_argument("--timeout", type=float, default=120)
    args = ap.parse_args()

    config = {}
    if args.url:
        connector = controller_connector_from_url(args.url)
    else:
        try:
            config = load_system_config()
        except ConfigError as exc:
            print(f"config: {exc}\n(or pass --url ws://0.0.0.0:8765/motion)")
            return 2
        print(f"config: {system_config_path()}")
        connector = controller_connector_from_config(config)
    hb = config.get("heartbeat", {})
    controller = MotionController(connector, heartbeat_interval=float(hb.get("interval", 0.5)),
                                  heartbeat_timeout=float(hb.get("timeout", 2.0)))
    args.url = args.url or describe(config["connector"])
    controller.on("robot_online", lambda e: print(f"● robot online: {e['robot_id']}"))
    controller.on("robot_offline", lambda e: print(f"○ robot offline: {e['robot_id']}"))
    await controller.start()
    print(f"controller listening on {args.url} — start the robot (Unity: press Play)")
    try:
        if args.script:
            robot = await controller.wait_for_robot(args.robot, timeout=args.timeout)
            print(f"robot {robot.robot_id}: joints {robot.joint_names}")
            t0 = time.monotonic()
            ok = await scripted(robot)
            print(f"{'ALL PASS' if ok else 'FAILED'} in {time.monotonic() - t0:.1f}s")
            return 0 if ok else 1
        await interactive(controller)
        return 0
    finally:
        await controller.stop()


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
