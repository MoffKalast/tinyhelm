# tinyhelm

MOOS Ivp-Helm for ROS but smol.

## Installation

```bash
cd ~/catkin_ws/src
git clone -b ros1 https://github.com/MoffKalast/tinyhelm.git
cd ..
rosdep install -i --from-path src/tinyhelm -y
catkin_make
```

On 20.04 some apt packages may not be available, in that case install them with pip.

## Core

```bash
/tinyhelm/set_home (PoseStamped)
/tinyhelm/plan/goal (PoseStamped)
/tinyhelm/plan/waypoints (Path)
/tinyhelm/plan/loiter (Path)
/tinyhelm/estop (Empty), goes to idle until a new plan is received
```

## Behaviors

Each behaviour should have the following API:

```bash
/behaviour_name/goal (PoseStamped, single goal, optional)
/behaviour_name/waypoints (Path, multiple goals, optional)
/behaviour_name/enabled (Bool, publishes a latched state and monitors for changes)
/behaviour_name/cmd_vel (Twist, output, relayed to /cmd_vel)
/behaviour_name/debug_markers (MarkerArray, visualization, relayed to /tinyhelm/debug_markers when planner is active) 
/behaviour_name/plan (Path, visualization, relayed to /tinyhelm/plan when planner is active) 
```