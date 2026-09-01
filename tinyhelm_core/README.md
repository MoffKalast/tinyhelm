# The Core

The proverbial rug that really ties the helm together. The main state machine that manages controllers and takes suggestions from monitors. 

It receives a behaviour command, turns it into an intention, hands that intention to whichever controller can execute it, and points the mux at that controller's `cmd_vel` topic. Monitors watch what is being steered and report how they feel about it; the helm decides what that means. Keeping the arbitration in one node and out of the controllers is what lets a controller stay a plain line follower with no idea that obstacles or teleop exist.

## Enabling

Nothing happens until `/tinyhelm/enabled` is set to true, which arms the system. On enable the helm records a home position from TF, unless one was set manually, and drops into stationkeeping. On disable it stops the active controller, clears the mission from every monitor and hands the mux back to teleop only.

## Behaviours

Each behaviour is a small function that turns an incoming message into an intention: a plan, the controller that should run it, and what happens when it finishes. Adding one means writing that function and adding a block to `behaviour_topics` in `helm.yaml`.

| Behaviour | Topic | Type | Result |
| --- | --- | --- | --- |
| `goal` | `/tinyhelm/goal` | PoseStamped | Transit to a point, hold there |
| `goal_and_return` | `/tinyhelm/goal_and_return` | PoseStamped | Out to a point and back to where the command arrived, hold |
| `waypoints` | `/tinyhelm/waypoints` | Path | Run the path once, hold at the end |
| `waypoints_and_return` | `/tinyhelm/waypoints_and_return` | Path | Run it, then run it backwards, hold |
| `loiter_circle` | `/tinyhelm/loiter_circle` | Path | Closed loop, restarted forever |
| `loiter_line` | `/tinyhelm/loiter_line` | Path | Out and back along the path, forever |
| `stationkeeping` | `/tinyhelm/hold_position` | PoseStamped | Hold a point, or the current position if an ESTOP case is triggered. |

Every path behaviour is anchored (not in the naval sense) before it is published, meaning the vessel's current position is prepended as the first pose. The first leg is then the real transit out to the first waypoint instead of something the controller has to invent. The anchor is taken at publish time, not when the command arrived, so a loiter that restarts anchors from wherever the vessel actually is.

Both loiter behaviours rotate the path so the pose nearest the vessel comes first. Ordering a loiter from the far end of the pattern does not send the vessel back to the start line.

An empty Path sent to any behaviour is read as an estop and becomes stationkeeping.

## Controllers

A controller is registered by a block under `controllers` in `helm.yaml` listing its topic names. The helm publishes plans on them, subscribes to the status and markers, and relays whatever the controller reports as its current path to the monitors. Topic names are hardcoded inside each controller node, so these blocks have to match them rather than rename them.

Controllers report `ControllerStatus`: IDLE, ACTIVE, FINISHED, ESTOPPED, PREEMPTED, ABORTED, ERROR.

When a plan finishes, the helm pins the last waypoint and holds there, but only if the vessel is actually near it. The reach is the sum of the params listed in `hold_reach_params`, by default the stationkeeping divergence plus the waypoint arrival threshold. Finishing well short of the last waypoint means the plan was truncated underneath the controller, and holding at a waypoint the vessel never reached would drive it somewhere nobody asked for, so it holds in place instead.

## Monitors

A monitor sees the mission as executed and the path currently being steered, and publishes a `MonitorStatus`. The helm remembers the last status from each monitor and acts on the worst of them, which is why the statuses are ordered by severity:

| Status | Helm response |
| --- | --- |
| `OK` | Nothing |
| `REPLAN` | Accept the monitor's revised path and hand it to the active controller |
| `SLOW` | Scale forward speed to half (by default) |
| `HOLD` | Scale forward speed to zero |
| `ESTOP` | Disable cmd_vel_mux, switch to stationkeeping |

REPLAN deliberately sits lowest above OK. It is a proposal attached to a mission that is otherwise going fine, and it must never outrank a monitor that wants the vessel stopped.

SLOW and HOLD are the same mechanism at different settings and neither cancels anything, so a monitor can hold the vessel still and let it resume on its own once it is happy again.

## Speed Scaling

The scale is applied in the mux rather than inside the controllers, so no controller needs to know it is happening. Only forward speed is scaled: turning and vertical authority are exactly what lets a vessel approach something carefully, and throttling them would make a cautious approach worse rather than safer. Teleop is never scaled.

## cmd_vel Mux

`cmd_vel_mux` subscribes to every controller's velocity topic and forwards exactly one of them to `/cmd_vel`, chosen by the helm over `/cmd_vel_mux/_active_topic`. An empty selection means nothing navigational gets through.

Teleop always wins. Any message on `/cmd_vel_teleop` passes straight to the output unscaled and suppresses navigation until `teleop_timeout` seconds of silence. While that is happening the mux announces it on `/teleop_override_active`, and when control is released during a position hold the helm re-pins the hold to wherever the vessel now is, so letting go of the stick does not send it flying back.

## Markers

Markers from the active controller, every monitor and the home position are aggregated into one array on `/tinyhelm/markers` and republished at a fixed rate, keyed by source so a fast publisher replaces its own set instead of stacking duplicates. One topic to display rather than one per node.

## Params

Set in `param/helm.yaml`. `robot_frame` and `planning_frame` are global and read by every node in the stack; the rest is under `tinyhelm_core`.

```yaml
robot_frame: base_link
planning_frame: local

tinyhelm_core:
    home_topic: /tinyhelm/set_home      # PoseStamped, failsafe location, auto-set on enable unless set manually
    estop_topic: /tinyhelm/estop        # Empty, stop and go to stationkeeping
    enabled_topic: /tinyhelm/enabled    # Bool, enables or disables the whole stack
    markers_topic: /tinyhelm/markers    # MarkerArray, aggregate of the active controller and all monitors

    marker_rate: 10.0
    marker_clear_period: 5.0

    hold_reach_params:
        - /tinyhelm_stationkeeping/max_divergence
        - /tinyhelm_waypoints/xy_distance_threshold

    cmd_vel_mux:
        out_topic: /cmd_vel
        teleop_topic: /cmd_vel_teleop
        teleop_timeout: 2.0
        selector_topic: /cmd_vel_mux/_active_topic
        speed_scale_topic: /cmd_vel_mux/_speed_scale
        slow_speed_scale: 0.5
```

The `controllers`, `monitors` and `behaviour_topics` blocks are omitted here, see the file itself.

## Subscribed Topics

- `/tinyhelm/enabled` (Bool), arms or disarms the entire stack
- `/tinyhelm/estop` (Empty), immediate transition to stationkeeping
- `/tinyhelm/set_home` (PoseStamped), manual failsafe position, overrides the automatic one permanently
- `/teleop_override_active` (Bool), from the mux
- one behaviour topic per entry in `behaviour_topics`
- per controller, its status, markers and current path topics
- per monitor, its status, markers and revised path topics

## Published Topics

- `/cmd_vel` (Twist), from the mux, the only navigational output of the stack
- `/tinyhelm/markers` (MarkerArray), aggregate visualisation
- `/tinyhelm/enabled` (Bool), latched, echoed back so a frontend can read the current state
- `/cmd_vel_mux/_active_topic` (String), which controller currently owns the output
- `/cmd_vel_mux/_speed_scale` (Float32), latched
- per controller, its plan topic
- per monitor, the mission and current path relays

## Messages

`ControllerStatus` and `MonitorStatus`, both a severity-ordered enum plus an optional human readable message. 
