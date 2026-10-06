"""
Compare a robot's saved description (from `d` in controller_demo.py) with its URDF.

    python tools/check_description.py <data_dir>/<project>/<stage>/<robot>/description.json path/to/robot.urdf

Checks, per joint: type, parent / child, origin (position and rotation) and axis direction. Then it runs forward
kinematics on the URDF with the joint positions in the description and compares every link pose (relative to
the description's root link) with the pose the robot reported. Exit code 0 when everything matches.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

from remote_control.kinematics import (forward_kinematics, load_urdf_tree, origin_pose,  # noqa: E402
                                       pose_mul, quat_mul)


def inverse(pose):
    x, y, z, w = pose["orientation"]
    qi = (-x, -y, -z, w)
    from remote_control.kinematics import quat_rotate
    p = quat_rotate(qi, [-c for c in pose["position"]])
    return {"position": list(p), "orientation": list(qi)}


def angle_between(qa, qb):
    """Rotation angle (rad) of qa⁻¹·qb."""
    d = quat_mul((-qa[0], -qa[1], -qa[2], qa[3]), tuple(qb))
    return 2 * math.acos(min(1.0, abs(d[3])))


def dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("description")
    ap.add_argument("urdf")
    ap.add_argument("--tol-pos", type=float, default=1e-3, help="metres (default 1 mm)")
    ap.add_argument("--tol-rot", type=float, default=1e-2, help="radians (default 0.01)")
    args = ap.parse_args()

    with open(args.description, encoding="utf-8") as f:
        desc = json.load(f)
    urdf = load_urdf_tree(args.urdf)
    u_by_name = {j["name"]: j for j in urdf["joints"]}
    u_by_child = {j["child"]: j for j in urdf["joints"]}
    ok = True

    def bad(msg):
        nonlocal ok
        ok = False
        print("  FAIL " + msg)

    print(f"{desc.get('project')} / {desc.get('stage')} / {desc.get('robot')}  vs  {urdf.get('robot')} ({args.urdf})")
    print("joints:")
    name_map = {}   # description command_name → URDF joint name
    for dj in desc.get("joints", []):
        uj = u_by_name.get(dj["name"]) or u_by_child.get(dj.get("child"))
        if uj is None:
            bad(f"{dj['name']}: not in the URDF")
            continue
        if dj.get("command_name"):
            name_map[dj["command_name"]] = uj["name"]
        label = f"{uj['name']:<24}"
        problems = []
        if dj["type"] != uj["type"] and not {dj["type"], uj["type"]} <= {"revolute", "continuous"}:
            problems.append(f"type {dj['type']} != {uj['type']}")
        if (dj.get("parent"), dj.get("child")) != (uj["parent"], uj["child"]):
            problems.append(f"links {dj.get('parent')}→{dj.get('child')} != {uj['parent']}→{uj['child']}")
        if dj.get("origin"):
            a, b = origin_pose(dj["origin"]), origin_pose(uj["origin"])
            dp = dist(a["position"], b["position"])
            dr = angle_between(a["orientation"], b["orientation"])
            if dp > args.tol_pos:
                problems.append(f"origin xyz off by {dp * 1000:.2f} mm")
            if dr > args.tol_rot:
                problems.append(f"origin rotation off by {math.degrees(dr):.2f}°")
        if uj["type"] != "fixed" and dj.get("axis"):
            norm = math.sqrt(sum(x * x for x in uj["axis"])) or 1.0
            ua = [c / norm for c in uj["axis"]]
            dot = sum(x * y for x, y in zip(dj["axis"], ua))
            if dot < -0.999:
                problems.append("axis REVERSED")
            elif dot < 0.999:
                problems.append(f"axis {dj['axis']} != {uj['axis']}")
        if problems:
            bad(label + "; ".join(problems))
        else:
            print(f"  ok   {label}{uj['type']}")

    links = {l["name"]: l["pose"] for l in desc.get("links", []) if l.get("pose")}
    root = desc.get("root")
    if root in links:
        print(f"forward kinematics (relative to {root}):")
        positions = {name_map.get(k, k): v for k, v in desc.get("positions", {}).items() if v is not None}
        urdf_root = dict(urdf, root=root)
        fk = forward_kinematics(urdf_root, positions)
        inv_root = inverse(links[root])
        worst_p, worst_r = 0.0, 0.0
        for name, pose in links.items():
            if name not in fk:
                continue
            rel = pose_mul(inv_root, pose)
            dp = dist(rel["position"], fk[name]["position"])
            dr = angle_between(rel["orientation"], fk[name]["orientation"])
            worst_p, worst_r = max(worst_p, dp), max(worst_r, dr)
            if dp > args.tol_pos * 5 or dr > args.tol_rot * 5:
                bad(f"{name:<24}pose off by {dp * 1000:.2f} mm / {math.degrees(dr):.2f}°")
        print(f"  largest difference: {worst_p * 1000:.3f} mm, {math.degrees(worst_r):.3f}°")
    print("ALL MATCH" if ok else "MISMATCHES FOUND")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
