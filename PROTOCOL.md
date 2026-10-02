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
| `control` | `pause`, `resume`, `cancel`, `stop`, `heartbeat`, `welcome` |
| `status` | everything sent by the robot |

## Units

Revolute joints use **radians**; prismatic joints use **meters**. Times are **seconds**.

## Messages: robot → controller

### `hello`
Sent right after connecting. It is sent again every `max(1 s, heartbeat_timeout)` until a `welcome` arrives, because over a broker a hello can be published while no controller is listening.
```json
{ "protocol": 1, "name": "Demo arm", "software": "remote-control-unity/0.1",
  "joints": [ { "name": "shoulder", "type": "revolute", "lower": -3.14, "upper": 3.14, "max_velocity": 2.0 } ],
  "supports": { "pause": true, "report_points": true, "report_progress": true, "pose_targets": false },
  "state": { "...": "same as the state message" } }
```
`lower` / `upper` / `max_velocity` may be `null` (unlimited).

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

### `heartbeat`
An empty payload. See [Heartbeats](#heartbeats).

## Messages: controller → robot

### `welcome`
The reply to `hello`. Sets the heartbeat timing.
```json
{ "heartbeat_interval": 0.5, "heartbeat_timeout": 2.0 }
```

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

### `heartbeat`
An empty payload.

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

| | WebSocket | MQTT | ROS 2 (planned) |
|---|---|---|---|
| Address | Robot dials `ws://host:port/motion` | Both dial `mqtt://[user:pass@]broker:1883/<prefix>` (`mqtts://` for TLS) | `ros2://<namespace>` |
| command | JSON text frame | `<prefix>/<robot_id>/cmd`, QoS 1 | `ExecuteMotion` action goal |
| control | JSON text frame, handled ahead of goals | `<prefix>/<robot_id>/ctrl`, QoS 1 (heartbeats QoS 0) | action cancel + `pause` / `resume` / `stop` services |
| status | JSON text frame | `<prefix>/<robot_id>/status`, QoS 1 (heartbeats QoS 0) | action feedback / result + `state` topic |
| presence | the socket itself | `<prefix>/<robot_id>/online` and `<prefix>/_controller/online`: retained `"1"`, with `"0"` as last will | node graph |

### MQTT details

- Client ids are `rc-robot-<robot_id>` and `rc-controller-<random>`. Sessions are clean (MQTT 3.1.1). The default prefix is `rc`.
- A robot treats itself as **connected** only while it is connected to the broker **and** the retained
  controller presence is `"1"`. When the controller disappears, its last will sets presence to `"0"`, and robots
  pause exactly as they would on a dropped WebSocket. When the controller comes back, they send `hello` again.
- The controller opens a link for a robot on its first `status` message and binds it on `hello`. It closes the
  link when the robot's presence becomes `"0"`.
- `robot_id` must be a valid topic level: no `/`, `+` or `#`, and not `_controller`.
- One controller per prefix. Multiple controllers would need goal ownership, which v1 does not define.
