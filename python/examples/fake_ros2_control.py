"""
A pretend ros2_control robot for trying examples/ros2_robot_bridge.py without hardware.

Subscribes to a forward position controller's commands (std_msgs/Float64MultiArray) or a
joint_trajectory_controller topic (trajectory_msgs/JointTrajectory), follows them with a first-order lag,
and publishes sensor_msgs/JointState at 100 Hz. Run it in a sourced ROS 2 environment.

  python examples/fake_ros2_control.py --joints j1,j2,j3
  python examples/fake_ros2_control.py --joints j1,j2,j3 --command-type trajectory --command-topic /arm_controller/joint_trajectory
"""
import argparse
import math

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--joints", required=True)
    ap.add_argument("--command-topic", default="/arm_position_controller/commands")
    ap.add_argument("--command-type", choices=["position", "trajectory"], default="position")
    ap.add_argument("--joint-states", default="/joint_states")
    ap.add_argument("--time-constant", type=float, default=0.03, help="tracking lag, seconds")
    args = ap.parse_args()

    names = [j.strip() for j in args.joints.split(",") if j.strip()]
    pos = {n: 0.0 for n in names}
    target = dict(pos)
    rclpy.init()
    node = rclpy.create_node("fake_ros2_control")

    if args.command_type == "position":
        def on_cmd(msg: Float64MultiArray) -> None:
            target.update(zip(names, msg.data))
        node.create_subscription(Float64MultiArray, args.command_topic, on_cmd, 10)
    else:
        def on_traj(msg: JointTrajectory) -> None:
            if msg.points:
                target.update(zip(msg.joint_names, msg.points[-1].positions))
        node.create_subscription(JointTrajectory, args.command_topic, on_traj, 10)

    pub = node.create_publisher(JointState, args.joint_states, 10)
    dt = 0.01
    alpha = 1.0 - math.exp(-dt / max(args.time_constant, 1e-4))

    def step() -> None:
        for n in names:
            pos[n] += alpha * (target[n] - pos[n])
        msg = JointState(name=names, position=[pos[n] for n in names])
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)

    node.create_timer(dt, step)
    print(f"fake ros2_control: {len(names)} joints, commands on {args.command_topic}, states on {args.joint_states}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
