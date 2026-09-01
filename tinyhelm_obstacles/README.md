# The Obstacle Monitor

Builds a local costmap from range data, watches the route the vessel is currently steering, and proposes a way around anything that turns up on it.

It reports a status to the helm and offers a revised path, and the helm decides whether to use it. That is what lets the avoidance be switched off, ignored, or overruled by a more serious monitor without any of the controllers changing behaviour.

## Nodes

Four, plus a second instance of the scan converter:

- `scan_to_cloud` (C++), LaserScan to PointCloud2. Runs twice, once per input stream
- `costmap_node.py`, accumulates evidence into a scrolling local grid
- `planner_node.py`, answers plan requests with a Theta* route, and watches a route for intrusions
- `obstacle_monitor_node.py`, tracks mission progress, decides when to ask for a replan, reports to the helm

They are split this way because the planner holds no mission state at all. Everything it needs arrives in the request, so a stale reply cannot make it plan against a corridor that has since moved.

## Input points

There are three point clouds the obstacle node accepts: reliable, unreliable and free.

Generally speaking the helm assumes this:
- reliable points have been confirmed by multiple sensors and we can treat them as correctly registered obstacles
- unreliable points might be correct, or might just be noise, so they're only used for keeping costmap entires from reliable points alive, and extending them out further
- free points and the opposite of reliable, and clear the costmap where open water is confirmed traversable, which helps with faster clearance of spaces where dynamic obstacles have smeared a large blocked area along the costmap

By default, two `scan_to_cloud` nodes are spun up to turn laserscans into inputs for the reliable and unreliable cloud, which has to do with wasrt_ros integration.

## Evidence, not hit counts

Cells accumulate seconds observed rather than numbers of returns. A cell earns at most one credit per `confirm_period` however many points land in it, so a dense sensor and a sparse one agree on what an obstacle is, and standing still next to something does not inflate it into certainty faster than passing it slowly.

A cell counts as an obstacle after `confirm_seconds` and saturates at `memory_seconds`. When it stops being observed there is a `grace_seconds` window before anything decays, then it drains at `forget_ratio` seconds of silence per second of observation, so forgetting is deliberately slower than learning. Something seen for a long time survives a while behind the vessel; something glimpsed once does not.

The grid is a window of `costmap_size` metres that scrolls with the vessel, and information falls off the trailing edge. It only scrolls once the vessel has moved `scroll_hysteresis_cells`, so working back and forth across a cell boundary does not shed the trailing edge over and over.

On top of the hard obstacles is a soft penalty that falls away over `soft_radius`. It does not forbid anything, it just makes routes that hug obstacles more expensive than routes that do not, which is what produces a standoff distance without a hard wall to get trapped against.

## The corridor

The route may only leave the mission line by `max_lateral_detour`, forming a capsule around each leg, and it must keep `clearance` from any obstacle. Clearance is read from the active controller's `max_line_divergence` via `divergence_param`, so the corridor the planner clears and the corridor the line follower is trying to stay inside are the same number by construction rather than by two configs happening to agree.

Bounding the detour matters more than it looks. An unbounded planner asked to get around a breakwater will happily route several hundred metres around the far end of it, which is a legal path and a terrible decision. Refusing is the better answer, and the monitor reports the refusal rather than quietly doing something surprising.

## Planning

Theta* over the costmap, run on a coarse layer for the heuristic and the fine grid for the route. 

Replies carry the failure reasons explicitly rather than an empty path:

| Result | Meaning |
| --- | --- |
| `OK` | Route found |
| `GOAL_IN_OBSTACLE` | The waypoint is inside something solid |
| `GOAL_OUTSIDE_CORRIDOR` | The waypoint is further off the line than `max_lateral_detour` |
| `START_TRAPPED` | The vessel is enclosed, no route leaves |
| `NO_ROUTE` | Nothing gets through at the required clearance |
| `NO_COSTMAP` | No grid received yet |
| `INTERNAL_ERROR` | Bug |

`start_nudged` is set when the vessel's own cell was inside the inflation and the search had to begin nearby. It happens routinely when something drifts up against the hull, and the first point of the returned path is that nearby point rather than the vessel. Goals inside obstacles are also nudged outside of them when possible.

Between replans the planner keeps checking the current route against every costmap update and reports `PathStatus` when it flips between clear and obstructed, plus periodically either way at `status_period`. The report says which leg is blocked and how far along the route, so the monitor can tell an obstruction under the bow from one three waypoints away and react differently. The clearance the route was planned against is carried along with it, so the trip threshold cannot drift out of step with the route.

The monitor advances its progress and redraws its corridor on the steady report, so it never touches the grid itself.

## Params

Set in `tinyhelm_core/param/obstacles.yaml`. Frames come from the global `/robot_frame` and `/planning_frame`.

```yaml
tinyhelm_scan_to_reliable_cloud:
    frame: laser
    # points used for new obstacles
    scan_topic: /scan_verified
    cloud_topic: /reliable_cloud

tinyhelm_scan_to_unreliable_cloud:
    frame: laser
    # points used only for keeping existing detections alive
    scan_topic: /scan_filtered
    cloud_topic: /unreliable_cloud

tinyhelm_obstacles:
    costmap_resolution: 0.5
    costmap_size: 150.0
    scroll_hysteresis_cells: 5

    soft_radius: 8.0

    confirm_period: 0.2
    confirm_seconds: 0.8
    memory_seconds: 120.0
    grace_seconds: 3.0
    forget_ratio: 2.0

    # Downsampling for the heuristic layer, only ever used to estimate cost. 4 * 0.5 m = 2 m cells
    coarse_factor: 4

    status_period: 1.0

    # Full param path, has to point at the active controller
    divergence_param: /tinyhelm_waypoints/max_line_divergence
    robot_width: 1.0

    max_lateral_detour: 20.0
    request_timeout: 2.0
```

## Subscribed Topics

- `/scan_verified` (LaserScan) and `/scan_filtered` (LaserScan), the two input streams

- `/reliable_cloud`, `/unreliable_cloud`, `/free_cloud` (PointCloud2), what the costmap actually consumes. The first two come from the scan converters; `free_cloud` is an optional clearing stream if you have a source of known-empty space

- `/obstacles/clear_costmap` (Empty), wipes all accumulated evidence

- `/obstacles/_mission_in` (Path), the mission as executed, including the transit leg

- `/obstacles/_current_path_in` (Path), relay of what the active controller is actually following

- `/tinyhelm/enabled` (Bool), the costmap stops consuming clouds while the stack is disabled

## Published Topics

- `/obstacles/_status` (MonitorStatus), latched, what the helm reacts to

- `/obstacles/_revised_path_out` (Path), latched, the proposed detour

- `/obstacles/_remaining` (Path), latched, what is left of the mission

- `/obstacles/costmap` (OccupancyGrid), latched, the accumulated evidence grid

- `/obstacles/_markers` (MarkerArray), latched, corridor and obstruction visualisation

- `/obstacles/plan_request` (PlanRequest), `/obstacles/plan_reply` (PlanReply), `/obstacles/path_watch` (PathWatch), `/obstacles/path_status` (PathStatus), internal traffic between the monitor and the planner

Topic names are hardcoded in the nodes, so the `monitors/obstacles` block in `helm.yaml` has to match them.

## Messages

- `PlanRequest`, a start, a goal, a hard clearance and the corridor polyline with its radius. Sent as a polyline rather than as capsules so it stays readable on the wire and both ends derive the same tube from it. `soft_radius` is not sent, both ends read it from the same parameter

- `PlanReply`, result code, the route, whether the start was nudged, and timing

- `PathWatch`, the route to keep re-examining, with the clearance it was planned against

- `PathStatus`, blocked or not, which leg, how far along, and the minimum clearance seen 
