"""
Make any ROS 2 (ros2_control) robot a Remote Control robot, reachable over ws://, mqtt:// or ros2://.

The bridge runs the MotionExecutor (timing, pause ramps, reports) and streams position targets to the
robot's controller; it reads /joint_states for feedback. Run it in a sourced ROS 2 environment.

  # RoboSynth solution 18 (JointGroupPositionController), limits from its URDF, controller over MQTT:
  python examples/ros2_robot_bridge.py --id robot_0625 \\
      --command-topic /arm_position_controller/commands \\
      --joints L_shoulder_joint,L_arm1_joint,L_arm2_joint,L_wrist1_joint,L_wrist2_joint,L_claw_joint,L_finger_l_joint,R_shoulder_joint,R_arm1_joint,R_arm2_joint,R_wrist1_joint,R_wrist2_joint,R_claw_joint,R_finger_r_joint \\
      --urdf path/to/robot_0625_ros2.urdf

  # joint_trajectory_controller instead:
  python examples/ros2_robot_bridge.py --command-type trajectory \\
      --command-topic /arm_controller/joint_trajectory --joints j1,j2,j3

The connection to the controller comes from the system config file, %RC_CONFIG_DIR%\remote_control.json
(its "connector" section); --url overrides it.

Try it without hardware: python examples/fake_ros2_control.py (same --joints / --command-topic).
"""
import argparse
import asyncio
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

from remote_control import (ConfigError, Joint, RobotRuntime, load_system_config, load_urdf_joints,  # noqa: E402
                            robot_connector_from_config, robot_connector_from_url, system_config_path)
from remote_control.drivers.ros2 import Ros2JointDriver  # noqa: E402


def describe(section):
    """One-line description of a connector config section, without secrets."""
    if section.get("url"):
        return section["url"]
    opts = ", ".join(f"{k}={v}" for k, v in section.items() if k not in ("type", "password") and v is not None)
    return f"{section.get('type')} ({opts})"


async def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--url", default=None, help="connector URL, overriding the config file (%%RC_CONFIG_DIR%%\\remote_control.json)")
    ap.add_argument("--id", default="ros2-arm", help="robot_id announced to the controller")
    ap.add_argument("--name", default=None)
    ap.add_argument("--joints", required=True, help="comma-separated joint names, in the controller's command order")
    ap.add_argument("--expose", default=None, help="subset of --joints to expose (default: all)")
    ap.add_argument("--command-topic", default="/arm_position_controller/commands")
    ap.add_argument("--command-type", choices=["position", "trajectory"], default="position")
    ap.add_argument("--joint-states", default="/joint_states")
    ap.add_argument("--urdf", default=None, help="read limits / max velocities from this URDF")
    ap.add_argument("--max-velocity", type=float, default=1.0, help="default max velocity (rad/s or m/s)")
    ap.add_argument("--rate", type=float, default=100.0, help="command rate, Hz")
    ap.add_argument("--decel", type=float, default=0.4, help="seconds to slow to a halt on pause/cancel/stop")
    ap.add_argument("--domain", type=int, default=None, help="ROS_DOMAIN_ID of the robot")
    args = ap.parse_args()

    command_joints = [j.strip() for j in args.joints.split(",") if j.strip()]
    exposed = [j.strip() for j in args.expose.split(",")] if args.expose else command_joints
    if args.urdf:
        joints = load_urdf_joints(args.urdf, exposed, default_max_velocity=args.max_velocity)
    else:
        joints = [Joint(n, max_velocity=args.max_velocity) for n in exposed]

    driver = Ros2JointDriver(joints, args.command_topic, command_type=args.command_type,
                             joint_state_topic=args.joint_states, command_joints=command_joints,
                             domain_id=args.domain, node_name="rc_robot_bridge")
    print(f"waiting for {args.joint_states} …")
    if not await asyncio.get_event_loop().run_in_executor(None, driver.wait_ready, 30.0):
        print(f"no joint states for {command_joints} on {args.joint_states}")
        driver.close()
        return 1

    if args.url:
        connector = robot_connector_from_url(args.url)
    else:
        try:
            config = load_system_config()
        except ConfigError as exc:
            print(f"config: {exc}\n(or pass --url ...)")
            driver.close()
            return 2
        print(f"config: {system_config_path()}")
        connector = robot_connector_from_config(config)
        args.url = describe(config["connector"])
    runtime = RobotRuntime(connector, driver, args.id, name=args.name or args.id,
                           tick_hz=args.rate, decel_time=args.decel)
    await runtime.start()
    print(f"bridge '{args.id}': {len(joints)} joints → {args.command_topic} ({args.command_type}); controller {args.url}")
    last = None
    try:
        while True:
            await asyncio.sleep(0.1)
            st = runtime.executor.state_payload()
            key = (runtime.connector.connected, st["state"], st["goal_id"], st["pause_reason"])
            if key != last:
                last = key
                print(f"{'connected' if key[0] else 'offline  '}  state={st['state']} goal={st['goal_id']}"
                      + (f" ({st['pause_reason']})" if st["pause_reason"] else ""))
    finally:
        await runtime.stop()
        driver.close()


if __name__ == "__main__":
    try:
        sys.exit(asyncio.run(main()))
    except KeyboardInterrupt:
        pass
