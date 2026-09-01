# The Stationkeeping Controller

A position hold controller. Given a point, it keeps the vessel within a set distance of it and does nothing while it is already close enough.

Outside `max_divergence` the controller turns towards the target and drives at it. Inside the deadzone, `deadzone_fraction` of that divergence, it does nothing at all and lets the vessel drift. Between the two it is winding down, which gives it hysteresis: the vessel has to actually reach the deadzone before the controller goes quiet, and has to escape the full divergence before it wakes up again. The result is an occasional correction burst rather than a permanent hum, unless there's a constant current in which case it acts like a "virtual anchor" placed at the stationkeeping point.

`max_divergence` is also read by the helm as part of its hold reach, so it doubles as the definition of what counts as being at a waypoint at the end of a plan.

## Params

Frames come from the global `/robot_frame` and `/planning_frame`, everything else is private to the node and normally comes from the planner config loaded alongside it:

```yaml
tinyhelm_stationkeeping:
    max_linear_speed: 0.45
    max_turning_speed: 0.9

    # How far the vessel may drift before it gets pulled back.
    max_divergence: 1.0

    # Fraction of max_divergence treated as good enough, inside this nothing is commanded.
    deadzone_fraction: 0.1

    # PID params for heading control.
    P: 3.0
    I: 0.001
    D: 65.0

    # Update rate, should be about the same as localization rate.
    rate: 30
```

See `tinyhelm_core/param/planner_asv.yaml` and `planner_auv.yaml` for the shipped surface and underwater tunings.

## Subscribed Topics

- `/stationkeeping/_pose` (PoseStamped), the position to hold. Transformed into the planning frame if it arrives in another. Receiving one starts the hold, there is no separate arm step

- `/stationkeeping/_clear` (Empty), stops immediately and forgets the target

- `/stationkeeping/_enabled` (Bool), starts or stops holding without discarding the node's state

## Published Topics

- `/cmd_vel_stationkeeping` (Twist), muxed onto `/cmd_vel` by the helm core

- `/stationkeeping/_status` (ControllerStatus), latched controller state the helm core reacts to

- `/stationkeeping/_enabled` (Bool), latched

- `/stationkeeping/_markers` (MarkerArray), target, divergence radius and deadzone

Topic names are hardcoded in the node, so the `controllers/stationkeeping` block in `helm.yaml` has to match them.

## Dynamic Reconfigure Params

Every param above can be changed at runtime:

- `max_linear_speed` (double_t), the maximum linear speed of the robot.

- `max_turning_speed` (double_t), the maximum speed at which the robot can turn.

- `max_divergence` (double_t), how far the vessel may drift from the target before it is corrected.

- `deadzone_fraction` (double_t), fraction of `max_divergence` treated as good enough. Raising it means fewer, larger corrections; lowering it means tighter holding at the cost of running the thrusters more.

- `P` (double_t), the proportional gain for the PID controller that controls the heading of the robot.

- `I` (double_t), the integral gain for the PID controller.

- `D` (double_t), the derivative gain for the PID controller.

- `rate` (int_t), the rate at which the robot updates its position and velocity.