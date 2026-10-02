"""Remote Control Motion Protocol v1 — transport-neutral motion goals for robots and simulators."""
from .controller import GoalHandle, GoalRejected, MotionController, RobotHandle, RobotOffline
from .executor import FakeDriver, JointDriver, MotionExecutor
from .protocol import PROTOCOL_VERSION, Channel, Envelope, GoalStatus, MsgType, RobotState
from .robot import RobotRuntime
from .trajectory import GoalError, Joint, Trajectory
from .transports import controller_transport_from_url, robot_transport_from_url

__version__ = "0.1.0"

__all__ = [
    "MotionController", "RobotHandle", "GoalHandle", "GoalRejected", "RobotOffline",
    "RobotRuntime", "MotionExecutor", "JointDriver", "FakeDriver",
    "Envelope", "MsgType", "Channel", "GoalStatus", "RobotState", "PROTOCOL_VERSION",
    "Joint", "Trajectory", "GoalError",
    "controller_transport_from_url", "robot_transport_from_url",
]
