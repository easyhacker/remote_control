"""
JointDriver for real ROS 2 robots (ros2_control), so any ROS robot can be a Remote Control robot over
ws://, mqtt:// or ros2:// (see examples/ros2_robot_bridge.py).

Reads   sensor_msgs/JointState            (default /joint_states)
Writes  either
        std_msgs/Float64MultiArray        → a forward position controller, e.g.
                                            position_controllers/JointGroupPositionController
                                            (/arm_position_controller/commands); the full vector in
                                            `command_joints` order is sent every tick
        trajectory_msgs/JointTrajectory   → joint_trajectory_controller's ~/joint_trajectory topic; one
                                            point per tick, reached `lookahead` seconds later

The MotionExecutor already does the interpolation, pause ramps and timing — the ROS controller only has
to track a stream of position targets.

Joint limits / max velocities come from the URDF (load_urdf_joints) or are given explicitly.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Dict, List, Optional, Sequence

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ..executor import JointDriver
from ..trajectory import Joint
from ..urdf import load_urdf_joints  # noqa: F401  (re-exported for convenience)

log = logging.getLogger(__name__)


class Ros2JointDriver(JointDriver):
    """
    joints          the joints exposed over the protocol (limits / max velocity)
    command_joints  order of the controller's command vector (default: the names of `joints`)
    """

    def __init__(self, joints: List[Joint], command_topic: str, *, command_type: str = "position",
                 joint_state_topic: str = "/joint_states", command_joints: Optional[Sequence[str]] = None,
                 lookahead: float = 0.05, domain_id: Optional[int] = None, node_name: str = "rc_joint_driver") -> None:
        if command_type not in ("position", "trajectory"):
            raise ValueError("command_type must be 'position' or 'trajectory'")
        self._joints = list(joints)
        self.command_joints = list(command_joints or [j.name for j in joints])
        unknown = [j.name for j in joints if j.name not in self.command_joints]
        if unknown:
            raise ValueError(f"joints not in command_joints: {', '.join(unknown)}")
        self.command_type = command_type
        self.lookahead = lookahead
        self._lock = threading.Lock()
        self._measured: Dict[str, float] = {}
        self._targets: Optional[Dict[str, float]] = None   # held targets for the full command vector

        self.context = Context()
        if domain_id is None:
            rclpy.init(context=self.context)
        else:
            rclpy.init(context=self.context, domain_id=domain_id)
        self.node = rclpy.create_node(node_name, context=self.context)
        msg_type = Float64MultiArray if command_type == "position" else JointTrajectory
        self._pub = self.node.create_publisher(msg_type, command_topic, 10)
        self.node.create_subscription(JointState, joint_state_topic, self._on_state, qos_profile_sensor_data)
        self._executor = SingleThreadedExecutor(context=self.context)
        self._executor.add_node(self.node)
        self._running = True
        self._thread = threading.Thread(target=self._spin, name="rc-ros2-driver", daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        while self._running and self.context.ok():
            self._executor.spin_once(timeout_sec=0.05)

    def _on_state(self, msg: JointState) -> None:
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._measured[name] = pos
            if self._targets is None and all(n in self._measured for n in self.command_joints):
                self._targets = {n: self._measured[n] for n in self.command_joints}

    def wait_ready(self, timeout: float = 10.0) -> bool:
        """Block until joint states for every command joint have been received."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._targets is not None:
                    return True
            time.sleep(0.02)
        return False

    def close(self) -> None:
        self._running = False
        self._thread.join(timeout=2.0)
        self._executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        if self.context.ok():
            rclpy.shutdown(context=self.context)

    # ── JointDriver ──────────────────────────────────────────────────────────

    def joints(self) -> List[Joint]:
        return list(self._joints)

    def read_positions(self) -> Dict[str, float]:
        with self._lock:
            return {j.name: self._measured.get(j.name, 0.0) for j in self._joints}

    def write_targets(self, targets: Dict[str, float]) -> None:
        with self._lock:
            if self._targets is None:
                log.warning("no joint states yet — command ignored")
                return
            self._targets.update(targets)
            vector = [self._targets[n] for n in self.command_joints]
        if self.command_type == "position":
            self._pub.publish(Float64MultiArray(data=vector))
        else:
            sec = int(self.lookahead)
            msg = JointTrajectory(joint_names=self.command_joints, points=[JointTrajectoryPoint(
                positions=vector, time_from_start=Duration(sec=sec, nanosec=int((self.lookahead - sec) * 1e9)))])
            self._pub.publish(msg)
