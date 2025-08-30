# tinyhelm

MOOS Ivp-Helm but smol.

## Core

/tinyhelm/set_home (PoseStamped)
/tinyhelm/plan/goal (PoseStamped)
/tinyhelm/plan/waypoints (Path)
/tinyhelm/plan/loiter (Path)
/tinyhelm/estop (Empty), goes to idle until a new plan is received

## Baheviours

Each behaviour should have the following API:

/behaviour_name/goal (PoseStamped, single goal, optional)
/behaviour_name/waypoints (Path, multiple goals, optional)
/behaviour_name/enabled (Bool, publishes a latched state and monitors for changes)
/behaviour_name/cmd_vel (Twist, output, relayed to /cmd_vel)
/behaviour_name/debug_markers (MarkerArray, visualization, relayed to /tinyhelm/debug_markers when planner is active) 
/behaviour_name/plan (Path, visualization, relayed to /tinyhelm/plan when planner is active) 