"""
Example external motion planner for the Robotic Toolbox's Motion tab.

It connects to the Toolbox's planner endpoint and sends one joint pose per frame (60 per second): every joint of the
Toolbox's current planner scope sways in a sine wave around its current position. The Toolbox forwards the poses to the
robot only while "Follow planner" is on; Pause / Resume / Stop in the Motion tab act on the motion.

    python examples/planner_example.py                         # ws://127.0.0.1:8770/planner
    python examples/planner_example.py --url ws://127.0.0.1:8770/planner --amplitude 0.3 --period 4

A real planner replaces next_pose() with its own computation and can use the "state" messages (measured positions,
about 30 per second) to close the loop.
"""
import argparse
import asyncio
import json
import math
import time

try:
    from websockets.asyncio.client import connect
except ImportError:                       # websockets < 13
    from websockets.client import connect  # type: ignore


def next_pose(t: float, centre: dict, limits: dict, amplitude: float, period: float) -> dict:
    """Every joint sways around its starting position, shifted in phase, kept inside its limits."""
    pose = {}
    for i, (name, x0) in enumerate(centre.items()):
        x = x0 + amplitude * math.sin(2 * math.pi * t / period + i * 0.8)
        lo, hi = limits.get(name, (None, None))
        pose[name] = min(max(x, lo if lo is not None else x), hi if hi is not None else x)
    return pose


async def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--url", default="ws://127.0.0.1:8770/planner")
    ap.add_argument("--amplitude", type=float, default=0.3, help="rad (or m for prismatic joints)")
    ap.add_argument("--period", type=float, default=4.0, help="seconds per swing")
    ap.add_argument("--rate", type=float, default=60.0, help="poses per second")
    args = ap.parse_args()

    async with connect(args.url) as ws:
        hello = json.loads(await ws.recv())
        limits = {j["name"]: (j.get("lower"), j.get("upper")) for j in hello.get("joints", [])}
        print(f"connected to the Toolbox (robot {hello.get('robot')}); press 'Follow planner' in the Motion tab")
        state = {"positions": {}, "following": False}
        centre: dict = {}
        scope: list = hello.get("scope", [])

        async def receive() -> None:
            nonlocal scope
            async for text in ws:
                msg = json.loads(text)
                if msg.get("type") == "hello":           # sent again when following starts on a new scope
                    scope = msg.get("scope", [])
                    centre.clear()
                elif msg.get("type") == "state":
                    state.update(msg)

        receiver = asyncio.ensure_future(receive())
        t0 = time.monotonic()
        try:
            while True:
                await asyncio.sleep(1.0 / args.rate)
                if not state["following"] or not scope:
                    t0 = time.monotonic()
                    continue
                if not centre:                           # sway around where the joints are now
                    centre.update({n: state["positions"].get(n, 0.0) for n in scope})
                pose = next_pose(time.monotonic() - t0, centre, limits, args.amplitude, args.period)
                await ws.send(json.dumps({"positions": pose}))
        finally:
            receiver.cancel()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
