# tinyhelm waypoints planner

Based on the Simple Projecting Line Planner.

A local planner that takes two goals (last and next), and follows a projected goal that keeps it close to the line segment between the two goals. Designed to resist strong side forces, such as from wind or water current.

![demo image](docs/demo.gif)

## Params

```xml
<node name="line_planner" pkg="line_planner" type="line_planner_node.py" output="screen">
	<param name="robot_frame" value="base_link"/>
	<param name="planning_frame" value="map"/>

	<param name="publish_debug_markers" value="true"/>
	<param name="ignore_altitude" value="false"/> <!-- Won't bother with z values -->

	<param name="max_linear_speed" value="3.0"/>
	<param name="max_turning_speed" value="2.0"/>
	<param name="max_vertical_speed" value="10.0"/>

	<!-- If base_link is this far away from the line, the projected distance will be min and scale to max when it's on the line.-->
	<param name="max_line_divergence" value="1.5"/>

	<!-- If any obstacles are defined then line divergence + robot width are used for finding a suitably wide path.-->
	<param name="robot_width" value="3.0"/>

	<param name="min_project_dist" value="0.3"/>
	<param name="max_project_dist" value="5.0"/>

	<!-- Distance at which the goal is considered reached.-->
	<param name="xy_distance_threshold" value="1.0"/>
	<param name="z_distance_threshold" value="0.5"/>

	<!-- PID params for heading control.-->
	<param name="P" value="2.0"/>
	<param name="I" value="0.002"/>
	<param name="D" value="65.0"/>

	<!-- If the robot frame is away from the line, the goal will be mirrored into the opposite direction and multiplied with this value.-->
	<param name="side_offset_mult" value="0.8"/>

	<!-- Update rate, should be about the same as localization rate.-->
	<param name="rate" value="30"/>
</node>
```

Here's a diagram showing the possible states of the planner, and which distances each parameter affects:

![diagram](docs/diagram.png)


## Subscribed Topics

- `/move_base_simple/goal` (PoseStamped), takes the current position as the starting point and moves towards the goal

- `/move_base_simple/clear` (Empty), stops all movement immediately

- `/move_base_simple/waypoints` (Path), takes each two consecutive points and navigates along the line between them

## Published Topics

- `/cmd_vel` (Twist), publishes velocity for vehicle motion

- `line_planner/active` (Bool), publishes navigation status

- `line_planner/plan` (Path), publishes a nav plan, also the entire route if given

- `line_planner/markers` (MarkerArray), publishes debug markers shown above

- `line_planner/vertical_target` (Float32), publishes the current altitude target

 ## Dynamic Reconfigure Params

- `publish_debug_markers` (bool_t), if set to True, the node will publish markers for debugging purposes.

- `ignore_altitude` (bool_t), if true, Z values will be disregarded.

- `max_linear_speed` (double_t), the maximum linear speed of the robot.

- `max_turning_speed` (double_t), the maximum speed at which the robot can turn.

- `max_vertical_speed` (double_t), the maximum vertical speed of the robot.

- `max_line_divergence` (double_t), the maximum distance that the robot can diverge from the line between the goals.

- `min_project_dist` (double_t), the minimum projection distance for the goal.

- `max_project_dist` (double_t), the maximum projection distance for the goal.

- `goal_distance_threshold` (double_t), the distance at which a goal is considered reached.

- `P` (double_t), the proportional gain for the PID controller that controls the heading of the robot.

- `I` (double_t), the integral gain for the PID controller.

- `D` (double_t), the derivative gain for the PID controller.

- `side_offset_mult` (double_t), multiplier for the side projection of the robot's position.

- `rate` (int_t), the rate at which the robot updates its position and velocity.