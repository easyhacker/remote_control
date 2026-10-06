# Remote Control Motion Protocol, v1

A transport-neutral protocol for sending motion goals to a robot or simulator (timed joint poses), getting
progress reports back, and interrupting motion (pause, resume, cancel, stop).

The same messages travel over every transport (WebSocket, MQTT, ROS 2). Only the transport adapter changes.

## Roles

- **Controller**: decides what the robot should do (backend, script, operator UI). Sends goals and control messages.
- **Robot**: executes motion (Unity scene, Isaac Sim, real robot bridge). Validates goals, runs them, reports back.

The robot connects out to the controller (WebSocket client, or MQTT client to a broker), so robots behind
home or office routers work without opening ports.

## Envelope

Every message is one JSON object:

```json
{ "v": 1, "type": "execute", "robot_id": "arm-01", "goal_id": "3f2c9a", "seq": 42, "ts": 1727880000.12, "payload": {} }
```

| Field | Type | Meaning |
|---|---|---|
| `v` | int | Protocol version (1). Receivers reject messages with a different major version. |
| `type` | string | Message type (below). |
| `robot_id` | string | Robot the message is about. |
| `goal_id` | string, optional | Goal the message is about. |
| `seq` | int | Per-sender counter, starting at 1 and increasing by 1. Receivers drop any number they have already seen (MQTT QoS 1 can deliver twice). Numbers may arrive out of order across channels (MQTT `cmd` and `ctrl` are separate topics), so receivers accept out-of-order numbers within a window of 1024. The counter resets when a `hello` / `welcome` starts a new session. |
| `ts` | float | Sender's wall-clock time, in seconds. Informational only; never used for timing. |
| `payload` | object | Type-specific body. |

## Channels

Transports keep three logical channels so that a `pause` is never queued behind a large goal:

| Channel | Types |
|---|---|
| `command` | `execute` |
| `control` | `pause`, `resume`, `cancel`, `stop`, `describe`, `visualize`, `target`, `heartbeat`, `welcome` |
| `status` | everything sent by the robot |

## Units

Revolute joints use **radians**; prismatic joints use **meters**. Times are **seconds**.

## Messages: robot → controller

### `hello`
Sent right after connecting. It is sent again every `max(1 s, heartbeat_timeout)` until a `welcome` arrives, because over a broker a hello can be published while no controller is listening.
```json
{ "protocol": 1, "name": "Demo arm", "software": "remote-control-unity/0.1",
  "joints": [ { "name": "shoulder", "type": "revolute", "lower": -3.14, "upper": 3.14, "max_velocity": 2.0 } ],
  "supports": { "pause": true, "report_points": true, "report_progress": true, "pose_targets": false,
                "describe": true, "visualize": true },
  "instance": "4f1c0a9e2b7d",
  "project": "My project", "stage": "robot0625",
  "state": { "...": "same as the state message" } }
```
`lower` / `upper` / `max_velocity` may be `null` (unlimited).
`instance` is random per robot session. It lets the controller tell a reconnect of the same robot (same instance)
from a different robot that announces an id already in use (see [Robot ids](#robot-ids)).
`project` and `stage` name where the robot lives (Unity: the project folder and the scene). Together with the
`robot_id` they identify the robot's data on the controller (see [Robot data](#robot-data)). Both default to `"default"`.

### `accepted` / `rejected`
The reply to every `execute`, sent before any motion starts.
```json
{ "queue_position": 0 }                  // accepted: 0 = running now, n = n goals ahead of it
{ "reason": "point 2: shoulder=3.5 above upper limit 3.14" }   // rejected
```

### `point_reached`
Sent when `report` includes points. Sent as the goal's timeline passes each point's `time_from_start`.
```json
{ "point_index": 1, "positions": [0.31, 0.79], "max_error": 0.012 }
```
`positions` are the measured positions on the first control tick at or after the point's time;
`max_error` is the largest |measured − point position| over the goal's joints. It includes tracking lag and up to
one tick of motion.

### `feedback`
Sent at `progress_hz` while the goal runs, when `report` includes progress.
```json
{ "state": "executing", "point_index": 1, "time": 3.2, "duration": 7.0, "rate": 1.0, "positions": [0.2, 0.6] }
```
`time` is the goal timeline (it stops advancing while paused); `rate` is the current time scale
(1 = running, 0 = paused, in between while slowing down or speeding up).

### `result`
Sent exactly once per accepted goal.
```json
{ "status": "succeeded", "positions": [0.0, 0.0], "message": "" }
```
`status` is one of `succeeded`, `canceled`, `stopped`, `aborted`.

### `state`
Sent on every state change, and inside `hello`.
```json
{ "state": "executing", "goal_id": "3f2c9a", "queued": ["a1b2c3"], "pause_reason": null,
  "positions": { "shoulder": 0.1, "elbow": 0.4 } }
```
`positions` covers all of the robot's joints, keyed by name. The `positions` arrays in other messages follow the goal's `joint_names` order.
`state` is `idle`, `executing`, `pausing`, `paused`, `resuming` or `stopping`.

### `ack`
The reply to `pause` / `resume` / `cancel` / `stop`.
```json
{ "ref_seq": 17, "ref_type": "pause", "ok": false, "message": "no such goal" }
```

### `description`
The reply to `describe`. `ref_seq` is the `seq` of the request.
```json
{ "ref_seq": 21, "ok": true, "message": "",
  "project": "My project", "stage": "robot0625", "robot": "unity-arm", "name": "Unity arm", "model": "robot_0625_ros2",
  "frame": "ros: x forward, y left, z up; metres; orientation quaternion [x, y, z, w]",
  "base_pose": { "position": [0.0, 0.0, 0.215], "orientation": [0.0, 0.0, 0.0, 1.0] },
  "positions": { "R_shoulder": 0.0, "R_arm1": 0.63 },
  "state": "idle",
  "root": "base",
  "links": [ { "name": "base", "pose": { "position": [0, 0, 0.215], "orientation": [0, 0, 0, 1] } } ],
  "joints": [ { "name": "R_arm1_joint", "type": "revolute", "parent": "R_shoulder", "child": "R_arm1",
                "origin": { "xyz": [0.0466, -0.0064, 0.0365], "rpy": [0.0, 0.0, 0.0] }, "axis": [0, 0, -1],
                "lower": -0.52, "upper": 1.75, "max_velocity": 2.0,
                "command_name": "R_arm1", "position": 0.63 } ] }
```
- All poses use the ROS / URDF convention named in `frame`: x forward, y left, z up, metres, quaternions `[x, y, z, w]`.
- `base_pose` is where the robot's root link is in the world (the scene): the robot's location.
- `root`, `links` and `joints` (the kinematic tree) are present only when the request asked for `tree: true`.
  `links[].pose` is each link's current world pose.
- `joints[]` follow URDF: `origin` is the child frame relative to the parent at position 0, and `axis` is in the child (joint)
  frame. `type` is `revolute`, `continuous`, `prismatic`, `fixed` or `spherical`.
  `command_name` is the name `execute` and `positions` use; it is `null` for joints that cannot be commanded.
  `position` is the current value (rad / m).
- A robot that knows no geometry sends `joints` with `parent` / `child` / `origin` left out or `null`.
- On failure: `{ "ref_seq": 21, "ok": false, "message": "..." }`.

### `selected`
The user picked a visualized item in the robot's viewer (clicked a frame in Unity's Game view, selected it in the
Hierarchy). Robots without a viewer never send it.
```json
{ "id": "frame:pick", "source": "click" }
```

### `edited`
The user moved an `editable` item in the viewer (Unity: Move / Rotate tools in the Scene view). Sent once the item
stops changing. `pose` is the new pose in the `parent` link (ROS convention); the controller decides what to do
with it (the Robotic Toolbox saves it as the TCP or frame).
```json
{ "id": "tcp:right arm", "parent": "R_claw",
  "pose": { "position": [0, 0, 0.12], "orientation": [0, 0, 0, 1] }, "source": "editor" }
```

### `heartbeat`
An empty payload. See [Heartbeats](#heartbeats).

## Messages: controller → robot

### `welcome`
The reply to `hello`. Sets the heartbeat timing.
```json
{ "heartbeat_interval": 0.5, "heartbeat_timeout": 2.0 }
```
If it also carries `robot_id` (and `instance` equal to the robot's), the id is taken: the robot switches to that id,
reconnects and sends `hello` under it (see [Robot ids](#robot-ids)).

### `execute`
```json
{ "joint_names": ["shoulder", "elbow"],
  "points": [ { "positions": [0.0, 0.5], "time_from_start": 2.0 },
              { "positions": [0.3, 0.8], "time_from_start": 4.5 } ],
  "report": "points",
  "progress_hz": 10,
  "on_busy": "queue",
  "interpolation": "cubic" }
```
- `joint_names`: a subset of the robot's joints, in any order. Joints not listed hold their position.
- `points`: at least one. `time_from_start` > 0 and strictly increasing.
- `report`: `none` | `points` | `progress` | `all`. The default is `points`.
- `on_busy` (what to do if a goal is already active):
  - `queue` (default): run after the active goal and any queued ones.
  - `replace`: slow the active goal to a halt (it ends `canceled`), drop the queue, then run this goal.
  - `reject`: reject this goal.
- `interpolation`: `cubic` (default; smooth, zero velocity at the start and end) or `linear`.

The motion starts at the robot's position when the goal begins and reaches `points[0]` at `points[0].time_from_start`.

### `pause` / `resume`
`goal_id` is set in the envelope. `pause` slows the timeline to a halt along the path, then holds.
`resume` speeds it back up from the same spot. The goal's remaining timing is preserved.

### `cancel`
`goal_id` is set in the envelope. A queued goal is removed (result `canceled`). The active goal slows to a halt,
then ends `canceled`, and the next queued goal starts.

### `stop`
No `goal_id`. The active goal slows to a halt and ends `stopped`. Every queued goal ends `stopped`. The robot holds position.

### `describe`
Asks for a `description`. Sent on the control channel; it never interrupts motion.
```json
{ "tree": true }
```
`tree: false` asks only for names, `base_pose` and `positions` (cheap: the controller uses it to read current positions,
for example when saving a pose). Robots that support it set `supports.describe` in `hello`.

### `visualize`
Shows markers in the robot's viewer (Unity draws them on top of the scene). Robots that support it set
`supports.visualize`; the reply is an `ack` (`ok: false` with a message for unknown links or unsupported kinds).
```json
{ "replace": true,
  "items": [
    { "id": "chain:base:R_claw", "kind": "chain", "links": ["base", "body", "R_shoulder", "R_arm1"], "end": "tcp:R_claw" },
    { "id": "tcp:R_claw", "kind": "frame", "parent": "R_claw",
      "pose": { "position": [0, 0, 0.1], "orientation": [0, 0, 0, 1] }, "label": "TCP R_claw", "style": "tcp" },
    { "id": "frame:pick", "kind": "frame", "parent": "base",
      "pose": { "position": [0.3, -0.2, 0.4], "orientation": [0, 0, 0, 1] },
      "label": "pick", "style": "target", "selectable": true, "selected": true } ] }
```
- `replace: true` removes everything shown before; otherwise items update by `id`, and `{ "id": …, "remove": true }`
  removes one.
- `frame`: axes (x red, y green, z blue) at `pose` in the `parent` link's frame (ROS convention). It moves with that
  link. `parent` `""` or `"@scene"` means the viewer's world. `size` is the axis length in metres; `style` is a hint
  (`tcp`, `target`, `frame`). `selectable` frames can be picked by the user, which sends `selected`; `editable` frames
  can be moved by the user, which sends `edited`. `name` is the object's name in the viewer's scene
  (e.g. `TCP_right_arm`).
- `replace: true` updates listed items in place and removes the others, so an object the user has selected or is
  dragging is not re-created.
- `chain`: a line through the origins of `links`, then to the item named by `end` (e.g. the TCP frame).

### `target`
Creates, moves, deletes or selects a target owned by the robot's scene (robots with `supports.targets`, e.g. Unity,
where targets are scene objects under `Targets` or attached to other objects). The reply is an `ack`.
```json
{ "op": "create", "name": "pick", "reference": "link:base",
  "pose": { "position": [0.4, -0.2, 0.3], "orientation": [0, 0, 0, 1] }, "parent": "Table/Fixture" }
{ "op": "update", "id": "target:Targets/pick", "reference": "scene", "pose": { … } }
{ "op": "delete", "id": "target:Targets/pick" }
{ "op": "select", "id": "target:Targets/pick" }
{ "op": "settings", "attach": true, "orientation": "approach" }
```
- `reference` says what `pose` is relative to: `scene`, `robot` (the robot instance) or `link:<name>`.
- `parent` (create): scene path of the object to attach to; default the scene's `Targets` object.
- `select` highlights the target and, in the Unity Editor, selects it for the Move / Rotate tools.
- `settings.attach`: Ctrl+click in the viewer attaches new targets to the clicked object instead of `Targets`.
- `settings.orientation` of Ctrl+click targets: `approach` (default; z into the surface, x towards the robot),
  `surface` (z out of the surface, x along the view) or `tcp` (the TCP's current orientation).
  Click targets are named after the clicked object: `<object>_<n>` with the lowest free n.

Such robots list their targets in every `description` (also `tree: false`), with the robot's own location:
```json
"robot_pose": { "position": [0, 0, 0], "orientation": [0, 0, 0, 1] },
"targets": [ { "id": "target:Targets/pick", "name": "pick", "path": "Targets/pick", "parent": "Targets",
               "pose_in_scene": { … }, "pose_in_robot": { … }, "pose_in_root": { … } } ]
```
`pose_in_root` is relative to the description's root link, which is what the controller uses for kinematics.

### `heartbeat`
An empty payload.

## Robot ids

`robot_id` must be unique per controller.
- Unity generates it from the GameObject name (`robot_0625_ros2`) and numbers duplicates in the loaded scenes
  (`-2`, `-3` …), unless the Robot Id field is set.
- If a robot announces an id that is already online, and its `instance` differs while the first robot's link is
  alive, the controller keeps the first robot and answers the newcomer with a `welcome` carrying a free id (`arm-2`).
  The newcomer reconnects under it and both run normally. A reconnect of the same robot (same `instance`) keeps its id.
- This needs the connector to tell robots apart by link (WebSocket). Over MQTT and ROS 2 the topics are named after
  the id, so two robots sharing an id share topics; keep ids unique there.

## Robot data

The protocol only carries `describe` / `description`. The reference controller (`python/remote_control`) also keeps files
per robot, under the `data_dir` of the system config:

```
<data_dir>/<project>/<stage>/<robot_id>/
    description.json    the last description (tree: true), with saved_at
    poses.json          named joint poses: { "poses": { "<name>": { "positions": { "<joint>": value }, "saved_at": "..." } } }
```

Names come from the robot's `hello` / `description`. Characters not allowed in file names (`<>:"/\|?*`) become `_`.
Moving to a saved pose is an ordinary one-point `execute`.

## Goal lifecycle

```
             execute
                │
        ┌───────┴────────┐
    rejected          accepted ──► queued ──┐
                                            ▼
                     ┌──────────────► executing ──────────────► succeeded
                     │                 │     ▲
               resume│           pause │     │ resume
                     │                 ▼     │
                     └─────────────── paused ┘
   cancel / stop / on_busy=replace from executing or paused ──► slowing down ──► canceled / stopped
   failure (driver error, robot shut down) ──► aborted
```

## Timing and interruption rules

1. Times are durations on the goal's own timeline, never wall-clock times.
2. Pause, cancel and stop never halt instantly. The robot scales the timeline rate from 1 to 0 over its
   deceleration time (default 0.4 s), so the motion slows along the planned path. Resume ramps the rate
   back from 0 to 1. Emergency stop belongs to hardware or local safety, not this protocol.
3. The robot validates the whole goal before accepting it: known joints, matching lengths, position limits,
   and speed. Speed is checked against the estimated peak, `1.5 × |Δposition| / Δt` per segment, which must not exceed `max_velocity`.
4. Every control message gets an `ack`. Repeating a control message is harmless. For example, a second pause ends with `ok: true` and the message "already paused".

## Heartbeats

Both sides send `heartbeat` every `heartbeat_interval` seconds.

- If the robot hears nothing from the controller for `heartbeat_timeout` seconds, or the connection drops,
  it pauses the running goal with `pause_reason: "connection_lost"` and re-sends `hello` until it is welcomed again. The goal stays paused across reconnects
  until the controller sends `resume` or `cancel`.
- If the controller hears nothing from the robot for `heartbeat_timeout`, it treats the robot as offline.

## Reconnecting

After reconnecting, the robot sends a fresh `hello` that includes its current `state`. A goal paused for
`connection_lost` is listed there, so the controller can resume or cancel it.

## Transport mappings

| | WebSocket | MQTT | ROS 2 |
|---|---|---|---|
| Address | Robot dials `ws://host:port/motion` | Both dial `mqtt://[user:pass@]broker:1883/<prefix>` (`mqtts://` for TLS) | Both join `ros2://<namespace>[?domain=N]` |
| command | JSON text frame | `<prefix>/<robot_id>/cmd`, QoS 1 | `/<ns>/<robot>/cmd` (`std_msgs/String`, reliable) |
| control | JSON text frame, handled ahead of goals | `<prefix>/<robot_id>/ctrl`, QoS 1 (heartbeats QoS 0) | `/<ns>/<robot>/ctrl` |
| status | JSON text frame | `<prefix>/<robot_id>/status`, QoS 1 (heartbeats QoS 0) | `/<ns>/<robot>/status` |
| presence | the socket itself | `<prefix>/<robot_id>/online` and `<prefix>/_controller/online`: retained `"1"`, with `"0"` as last will | `/<ns>/<robot>/online` and `/<ns>/_controller/online`: transient-local `"1"`, plus ROS graph matching |

### MQTT details

- Client ids are `rc-robot-<robot_id>` and `rc-controller-<random>`. Sessions are clean (MQTT 3.1.1). The default prefix is `rc`.
- A robot treats itself as **connected** only while it is connected to the broker **and** the retained
  controller presence is `"1"`. When the controller disappears, its last will sets presence to `"0"`, and robots
  pause exactly as they would on a dropped WebSocket. When the controller comes back, they send `hello` again.
- The controller opens a link for a robot on its first `status` message and binds it on `hello`. It closes the
  link when the robot's presence becomes `"0"`.
- `robot_id` must be a valid topic level: no `/`, `+` or `#`, and not `_controller`.
- One controller per prefix. Multiple controllers would need goal ownership, which v1 does not define.

### ROS 2 details

- Envelopes travel as JSON in `std_msgs/String`. No custom interfaces are used, so any ROS 2 install works
  without a colcon build. QoS is reliable with keep-last 100. Presence topics are transient-local (latched).
- `<robot>` in topic names is the `robot_id`, with characters ROS names don't allow replaced by `_` and an `r_`
  prefix if it starts with a digit. The envelope always carries the real id.
- ROS has no last will, so presence also comes from the graph. A robot is **connected** while the controller's presence is
  `"1"` **and** the controller publishes to the robot's `cmd`/`ctrl` topics **and** subscribes to its `status`. Neither side
  says hello until both directions are matched, so the first messages are not lost to DDS discovery.
- The controller discovers robots by their `/<ns>/*/status` topics. It drops a robot when nothing publishes that topic any more.
- This transport connects controllers and robots *over* ROS. Driving a ros2_control robot *from* a robot runtime is a
  separate concern: the `Ros2JointDriver` streams position targets to a forward position controller or a
  `joint_trajectory_controller` topic. Native interop for ROS clients (a `FollowJointTrajectory` action server in front
  of the executor) is planned.
