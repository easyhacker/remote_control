"""
Robot-side motion executor — the protocol's behaviour, written once and independent of transport.

It is driven by two calls from a single thread/task:
    handle(env)  — a message from the controller (execute / pause / resume / cancel / stop)
    tick(dt)     — advance motion by dt seconds and write joint targets to the driver
and reports through the `emit(type, goal_id, payload)` callback.

Interruptions never stop instantly: pause/cancel/stop ramp the goal's time scale (rate) from 1 to 0
over `decel_time`, so the motion slows along its planned path; resume ramps it back up.

Several goals can run at once when they move different joints (on_busy="parallel", e.g. the left arm and the
right arm of a dual-arm robot); each one is paused / resumed / cancelled on its own, and stop ends them all.
on_busy="queue" goals wait until the robot is idle, as before.

Stream goals (execute with "stream": true, no points) follow poses the controller sends in `stream` messages, e.g. one
per frame from an external motion planner: every tick the goal's joints move towards the newest pose, limited by
max_velocity, and the goal is paused / cancelled / stopped like any other.

The C# port (unity/.../Runtime/Core/MotionExecutor.cs) mirrors this file — keep them in step.
"""
from __future__ import annotations

import abc
from collections import deque
from typing import Any, Callable, Deque, Dict, List, Optional, Set

from .protocol import Envelope, GoalStatus, MsgType, RobotState
from .trajectory import GoalError, GoalSpec, Joint, Trajectory, check_segment_speed, clamp_to_limits, parse_goal

Emit = Callable[[str, Optional[str], Dict[str, Any]], None]


class JointDriver(abc.ABC):
    """What actually moves: a simulator articulation, ros2_control, a servo board…"""

    @abc.abstractmethod
    def joints(self) -> List[Joint]:
        ...

    @abc.abstractmethod
    def read_positions(self) -> Dict[str, float]:
        ...

    @abc.abstractmethod
    def write_targets(self, targets: Dict[str, float]) -> None:
        ...


class FakeDriver(JointDriver):
    """Perfect-tracking driver for tests and the fake robot example."""

    def __init__(self, joints: List[Joint], initial: Optional[Dict[str, float]] = None) -> None:
        self._joints = joints
        self.positions = {j.name: 0.0 for j in joints}
        self.positions.update(initial or {})

    def joints(self) -> List[Joint]:
        return list(self._joints)

    def read_positions(self) -> Dict[str, float]:
        return dict(self.positions)

    def write_targets(self, targets: Dict[str, float]) -> None:
        self.positions.update(targets)


class _Goal:
    """One accepted goal: queued (traj is None) or running (in MotionExecutor.actives).

    Its own timeline: `time` advances by dt × `rate`; `rate` ramps towards `target_rate` (1 = run, 0 = halt), which
    is how pause / cancel / stop slow it down along the path. Each goal has its own rate, so pausing one goal does
    not affect another one running in parallel."""

    def __init__(self, goal_id: str, spec: GoalSpec) -> None:
        self.id = goal_id
        self.spec = spec
        self.joint_set: Set[str] = set(spec.joint_names)   # running goals never share a joint
        self.traj: Optional[Trajectory] = None
        self.time = 0.0                          # position on the goal's timeline (s)
        self.rate = 1.0                          # current time scale
        self.target_rate = 1.0                   # time scale being ramped to
        self.end_status: Optional[str] = None   # set while slowing down to cancel/stop
        self.end_message = ""
        self.pause_reason: Optional[str] = None
        self.next_point = 0
        self.since_feedback = 0.0
        self.last_commanded: List[float] = []
        self.stream_target: Optional[List[float]] = None   # stream goals: the newest streamed pose
        self.stream_end = False                            # stream goals: no more poses will come
        self.stream_count = 0

    @property
    def parallel(self) -> bool:
        """on_busy="parallel": may run next to goals on other joints. Other modes run only when the robot is idle."""
        return self.spec.on_busy == "parallel"

    @property
    def state(self) -> str:
        if self.end_status:
            return RobotState.STOPPING
        if self.target_rate == 0:
            return RobotState.PAUSED if self.rate == 0 else RobotState.PAUSING
        return RobotState.EXECUTING if self.rate == 1 else RobotState.RESUMING


class MotionExecutor:
    def __init__(self, driver: JointDriver, emit: Emit, decel_time: float = 0.4) -> None:
        self.driver = driver
        self.emit = emit
        self.decel_time = max(decel_time, 1e-3)
        self.joints: Dict[str, Joint] = {j.name: j for j in driver.joints()}
        self.actives: List[_Goal] = []          # running goals (disjoint joint sets)
        self.queue: Deque[_Goal] = deque()
        self._known_ids: set = set()
        self._last_state: Optional[Dict[str, Any]] = None

    @property
    def active(self) -> Optional[_Goal]:
        """The first running goal (older single-goal API)."""
        return self.actives[0] if self.actives else None

    def _find_active(self, goal_id: Optional[str]) -> Optional[_Goal]:
        return next((g for g in self.actives if g.id == goal_id), None)

    # ── state ────────────────────────────────────────────────────────────────

    @property
    def state(self) -> str:
        """Robot state: idle, or the 'busiest' state of the running goals."""
        if not self.actives:
            return RobotState.IDLE
        # A robot is "paused" only when every running goal is; one moving goal makes it "executing" etc.
        states = {g.state for g in self.actives}
        for s in (RobotState.EXECUTING, RobotState.RESUMING, RobotState.PAUSING, RobotState.STOPPING):
            if s in states:
                return s
        return RobotState.PAUSED

    def state_payload(self) -> Dict[str, Any]:
        pos = self.driver.read_positions()
        first = self.active
        paused = next((g for g in self.actives if g.pause_reason), None)
        # goal_id / pause_reason keep the single-goal meaning for older controllers; "active" and "goals" list
        # every running goal for controllers that use parallel goals.
        return {
            "state": self.state,
            "goal_id": first.id if first else None,
            "active": [g.id for g in self.actives],
            "goals": {g.id: {"state": g.state, "joints": list(g.spec.joint_names), "pause_reason": g.pause_reason}
                      for g in self.actives},
            "queued": [g.id for g in self.queue],
            "pause_reason": paused.pause_reason if paused else None,
            "positions": {n: pos.get(n) for n in self.joints},
        }

    def _publish_state(self, force: bool = False) -> None:
        st = self.state_payload()
        key = {k: v for k, v in st.items() if k != "positions"}
        if force or key != self._last_state:
            self._last_state = key
            self.emit(MsgType.STATE, None, st)

    # ── incoming messages ────────────────────────────────────────────────────

    def handle(self, env: Envelope) -> None:
        t = env.type
        if t == MsgType.EXECUTE:
            self._on_execute(env)
        elif t == MsgType.PAUSE:
            self._ack(env, *self.pause(env.goal_id, "requested"))
        elif t == MsgType.RESUME:
            self._ack(env, *self.resume(env.goal_id))
        elif t == MsgType.CANCEL:
            self._ack(env, *self.cancel(env.goal_id))
        elif t == MsgType.STOP:
            self._ack(env, *self.stop())
        elif t == MsgType.STREAM:
            self._on_stream(env)        # no ack: streams arrive at frame rate
        self._publish_state()

    def _on_stream(self, env: Envelope) -> None:
        """The newest pose for a stream goal (running or still queued); poses for unknown goals are dropped."""
        g = self._find_active(env.goal_id) or next((q for q in self.queue if q.id == env.goal_id), None)
        if g is None or not g.spec.stream or g.end_status:
            return
        positions = env.payload.get("positions")
        if isinstance(positions, list) and len(positions) == len(g.spec.joint_names):
            try:
                g.stream_target = [clamp_to_limits(self.joints[n], float(x))
                                   for n, x in zip(g.spec.joint_names, positions)]
                g.stream_count += 1
            except (TypeError, ValueError):
                pass
        if env.payload.get("end") is True:
            g.stream_end = True

    def _ack(self, env: Envelope, ok: bool, message: str) -> None:
        self.emit(MsgType.ACK, env.goal_id,
                  {"ref_seq": env.seq, "ref_type": env.type, "ok": ok, "message": message})

    def _on_execute(self, env: Envelope) -> None:
        goal_id = env.goal_id
        if not goal_id:
            self.emit(MsgType.REJECTED, None, {"reason": "execute needs a goal_id"})
            return
        if goal_id in self._known_ids:
            self.emit(MsgType.REJECTED, goal_id, {"reason": "duplicate goal_id"})
            return
        try:
            spec = parse_goal(env.payload, self.joints)
        except GoalError as exc:
            self.emit(MsgType.REJECTED, goal_id, {"reason": str(exc)})
            return
        busy = bool(self.actives) or bool(self.queue)
        if busy and spec.on_busy == "reject":
            self.emit(MsgType.REJECTED, goal_id, {"reason": "robot is busy"})
            return
        if busy and spec.on_busy == "replace":
            for q in list(self.queue):
                self._finish(q, GoalStatus.CANCELED, "replaced by a new goal")
            self.queue.clear()
            for g in self.actives:
                if not g.end_status:
                    self._begin_end(g, GoalStatus.CANCELED, "replaced by a new goal")
        # Every goal enters the queue; _start_ready() decides whether it may start right away.
        goal = _Goal(goal_id, spec)
        self._known_ids.add(goal_id)
        self.queue.append(goal)
        self._start_ready()
        if goal in self.actives:
            position = 0
        else:   # goals that still have to finish before this one can start
            ahead = [g for g in self.actives if not goal.parallel or g.joint_set & goal.joint_set]
            ahead += [q for q in self.queue if q is not goal and self.queue.index(q) < self.queue.index(goal)
                      and (not goal.parallel or q.joint_set & goal.joint_set)]
            position = max(1, len(ahead))
        self.emit(MsgType.ACCEPTED, goal_id, {"queue_position": position})

    # ── control (also used locally, e.g. by the heartbeat watchdog) ──────────

    def pause(self, goal_id: Optional[str], reason: str = "requested"):
        g = self._find_active(goal_id)
        if g is None:
            if any(q.id == goal_id for q in self.queue):
                return False, "goal is queued, not running"
            return False, "no such goal"
        if g.end_status:
            return False, "goal is ending"
        if g.target_rate == 0:
            return True, "already paused"
        g.target_rate = 0.0
        g.pause_reason = reason
        return True, ""

    def resume(self, goal_id: Optional[str]):
        g = self._find_active(goal_id)
        if g is None:
            return False, "no such goal"
        if g.end_status:
            return False, "goal is ending"
        if g.target_rate == 1:
            return True, "already running"
        g.target_rate = 1.0
        g.pause_reason = None
        return True, ""

    def cancel(self, goal_id: Optional[str]):
        for q in list(self.queue):
            if q.id == goal_id:
                self.queue.remove(q)
                self._finish(q, GoalStatus.CANCELED, "canceled while queued")
                return True, ""
        g = self._find_active(goal_id)
        if g is None:
            return False, "no such goal"
        if g.end_status:
            return True, "already ending"
        self._begin_end(g, GoalStatus.CANCELED, "canceled")
        return True, ""

    def stop(self):
        for q in list(self.queue):
            self._finish(q, GoalStatus.STOPPED, "robot stopped")
        self.queue.clear()
        for g in self.actives:
            self._begin_end(g, GoalStatus.STOPPED, "robot stopped")
        return True, ""

    def pause_for(self, reason: str) -> None:
        """Pause everything that is running (e.g. reason='connection_lost')."""
        changed = False
        for g in self.actives:
            if not g.end_status and g.target_rate != 0:
                self.pause(g.id, reason)
                changed = True
        if changed:
            self._publish_state()

    def abort_all(self, message: str) -> None:
        for q in list(self.queue):
            self._finish(q, GoalStatus.ABORTED, message)
        self.queue.clear()
        for g in list(self.actives):
            self._finish(g, GoalStatus.ABORTED, message)
        self.actives.clear()
        self._publish_state()

    def _begin_end(self, g: _Goal, status: str, message: str) -> None:
        g.end_status = status
        g.end_message = message
        g.target_rate = 0.0  # if already paused (rate 0), the next tick finishes it

    # ── motion ───────────────────────────────────────────────────────────────

    def _start_ready(self) -> None:
        """Start queued goals that may run now, in queue order:
        a "parallel" goal when none of its joints is used by a running goal or by an earlier queued goal;
        any other goal only when the robot is idle and nothing queued is ahead of it.

        `busy` collects the joints of running goals and of goals skipped so far, so goals on the same joints always
        start in the order they arrived, while a goal on free joints may overtake them."""
        busy: Set[str] = set()
        for g in self.actives:
            busy |= g.joint_set
        for g in list(self.queue):
            if g.parallel:
                ok = not (g.joint_set & busy)
            else:
                ok = not self.actives and self.queue[0] is g
            if ok and self._begin(g):
                self.queue.remove(g)
                busy |= g.joint_set
                continue
            if ok:            # could not start (aborted while checking): it left the queue
                continue
            if not g.parallel:
                break         # a sequential goal waits for idle; everything behind it waits too
            busy |= g.joint_set
        self._publish_state()

    def _begin(self, g: _Goal) -> bool:
        """Start a goal from the joints' current positions. The move to the first point is checked against the
        joints' max velocity only now, since the start position is not known earlier. False if it was aborted."""
        spec = g.spec
        pos = self.driver.read_positions()
        start = [pos[n] for n in spec.joint_names]
        if spec.stream:                 # no trajectory: hold here until the first streamed pose
            g.last_commanded = start
            if g.stream_target is None:
                g.stream_target = list(start)
            self.actives.append(g)
            return True
        try:
            check_segment_speed(spec.joint_names, self.joints, start, spec.positions[0],
                                spec.times[0], "start→point 0")
        except GoalError as exc:
            self.queue.remove(g)
            self._finish(g, GoalStatus.ABORTED, str(exc))
            return False
        g.traj = Trajectory(start, spec.times, spec.positions, spec.interpolation)
        g.last_commanded = start
        self.actives.append(g)
        return True

    def tick(self, dt: float) -> None:
        if not self.actives:
            if self.queue:
                self._start_ready()
            return
        # Each running goal writes only its own joints; a finished goal frees its joints for queued goals.
        finished = False
        for g in list(self.actives):
            if self._tick_goal(g, dt):
                self.actives.remove(g)
                finished = True
        if finished or self.queue:
            self._start_ready()
        self._publish_state()

    def _ramp(self, g: _Goal, dt: float) -> float:
        """Ramp the goal's time scale towards its target; returns the mean rate over this tick."""
        step = dt / self.decel_time
        r0 = g.rate
        if g.rate < g.target_rate:
            g.rate = min(g.target_rate, g.rate + step)
        elif g.rate > g.target_rate:
            g.rate = max(g.target_rate, g.rate - step)
        return 0.5 * (r0 + g.rate)

    def _tick_stream(self, g: _Goal, dt: float) -> bool:
        """A stream goal: move each joint towards the newest streamed pose, at most rate × speed × max_velocity, so
        a jump in the stream becomes a fast but limited move, and pause / cancel / stop slow it down to a halt."""
        mean_rate = self._ramp(g, dt)
        g.time += dt * mean_rate
        target = g.stream_target or g.last_commanded
        cmd = []
        for name, x, goal_x in zip(g.spec.joint_names, g.last_commanded, target):
            vmax = self.joints[name].max_velocity
            limit = (vmax * g.spec.stream_speed if vmax else float("inf")) * mean_rate * dt
            cmd.append(x + max(-limit, min(limit, goal_x - x)))
        g.last_commanded = cmd
        self.driver.write_targets(dict(zip(g.spec.joint_names, cmd)))
        if g.spec.reports_progress:
            g.since_feedback += dt
            if g.since_feedback >= 1.0 / g.spec.progress_hz:
                g.since_feedback = 0.0
                self._feedback(g)
        if g.end_status and g.rate == 0:
            self._finish(g, g.end_status, g.end_message)
            return True
        if g.stream_end and all(abs(a - b) < 1e-9 for a, b in zip(cmd, target)):
            self._finish(g, GoalStatus.SUCCEEDED, "stream ended")
            return True
        return False

    def _tick_goal(self, g: _Goal, dt: float) -> bool:
        """Advance one goal; True when it has ended."""
        if g.spec.stream:
            return self._tick_stream(g, dt)
        assert g.traj is not None
        # Ramp the time scale towards its target, integrating time with the mean rate
        g.time = min(g.traj.duration, g.time + dt * self._ramp(g, dt))

        cmd = g.traj.sample(g.time)
        g.last_commanded = cmd
        self.driver.write_targets(dict(zip(g.spec.joint_names, cmd)))

        if g.spec.reports_points:
            while g.next_point < len(g.spec.times) and g.time >= g.spec.times[g.next_point] - 1e-9:
                self._report_point(g, g.next_point)
                g.next_point += 1
        else:
            while g.next_point < len(g.spec.times) and g.time >= g.spec.times[g.next_point] - 1e-9:
                g.next_point += 1

        if g.spec.reports_progress:
            g.since_feedback += dt
            if g.since_feedback >= 1.0 / g.spec.progress_hz:
                g.since_feedback = 0.0
                self._feedback(g)

        if g.time >= g.traj.duration:
            self._finish(g, GoalStatus.SUCCEEDED, "")
            return True
        if g.end_status and g.rate == 0:
            self._finish(g, g.end_status, g.end_message)
            return True
        return False

    def _measured(self, g: _Goal) -> List[float]:
        pos = self.driver.read_positions()
        return [pos[n] for n in g.spec.joint_names]

    def _report_point(self, g: _Goal, index: int) -> None:
        measured = self._measured(g)
        target = g.spec.positions[index]
        err = max((abs(m - c) for m, c in zip(measured, target)), default=0.0)
        self.emit(MsgType.POINT_REACHED, g.id,
                  {"point_index": index, "positions": measured, "max_error": err})

    def _feedback(self, g: _Goal) -> None:
        if g.spec.stream:
            self.emit(MsgType.FEEDBACK, g.id, {
                "state": g.state, "stream": True, "time": round(g.time, 4), "rate": round(g.rate, 4),
                "poses_received": g.stream_count, "positions": self._measured(g),
            })
            return
        assert g.traj is not None
        self.emit(MsgType.FEEDBACK, g.id, {
            "state": g.state, "point_index": min(g.next_point, len(g.spec.times) - 1),
            "time": round(g.time, 4), "duration": g.traj.duration, "rate": round(g.rate, 4),
            "positions": self._measured(g),
        })

    def _finish(self, g: _Goal, status: str, message: str) -> None:
        positions = self._measured(g)
        self.emit(MsgType.RESULT, g.id, {"status": status, "positions": positions, "message": message})
