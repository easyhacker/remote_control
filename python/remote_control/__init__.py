"""Remote Control Motion Protocol v1 — connector-neutral motion goals for robots and simulators."""
from .connectors import (CONFIG_DIR_ENV, CONFIG_FILE_NAME, ConfigError, ControllerConnector, RobotConnector,
                         controller_connector_from_config, controller_connector_from_system_config,
                         controller_connector_from_url, load_config, load_system_config,
                         robot_connector_from_config, robot_connector_from_system_config,
                         robot_connector_from_url, system_config_path)
from .controller import GoalHandle, GoalRejected, MotionController, RobotHandle, RobotOffline
from .executor import FakeDriver, JointDriver, MotionExecutor
from .protocol import PROTOCOL_VERSION, Channel, Envelope, GoalStatus, MsgType, RobotState
from .robot import RobotRuntime
from .trajectory import GoalError, Joint, Trajectory
from .urdf import load_urdf_joints

# pre-0.3 names
controller_transport_from_url = controller_connector_from_url
robot_transport_from_url = robot_connector_from_url

__version__ = "0.3.0"

__all__ = [
    "MotionController", "RobotHandle", "GoalHandle", "GoalRejected", "RobotOffline",
    "RobotRuntime", "MotionExecutor", "JointDriver", "FakeDriver",
    "Envelope", "MsgType", "Channel", "GoalStatus", "RobotState", "PROTOCOL_VERSION",
    "Joint", "Trajectory", "GoalError", "load_urdf_joints",
    "ControllerConnector", "RobotConnector", "ConfigError", "load_config",
    "controller_connector_from_url", "robot_connector_from_url",
    "controller_connector_from_config", "robot_connector_from_config",
    "controller_connector_from_system_config", "robot_connector_from_system_config",
    "load_system_config", "system_config_path", "CONFIG_DIR_ENV", "CONFIG_FILE_NAME",
]
