#!/usr/bin/env python3
import rospy
import math
import tf
import tf2_ros

from utils import *
from utils import point_segment_distance
from markers import DebugMarkers

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, Bool, Float32

from tf.transformations import euler_from_quaternion
from tf2_geometry_msgs import do_transform_pose

from dynamic_reconfigure.server import Server as DynamicReconfigureServer

from tinyhelm_core.msg import ControllerStatus
from tinyhelm_waypoints.cfg import WaypointsConfig
from nav_msgs.msg import Path

ROBOT_FRAME = "base_link"
PLANNING_FRAME = "local"

class GoalServer:
	"""Owns the route and which leg of it is being followed.

	A path arrives with no indication of whether it is a new mission, the same one re-sent, or a
	revision of the one in progress, so where to pick it up is worked out geometrically instead.
	Both tests resolve to the earliest candidate: a survey pattern's parallel lines sit within a
	divergence of each other, so preferring the furthest along would let one detour around a speck
	of an obstacle skip half the mission. Repeating a leg is recoverable, skipping one is not."""

	def __init__(self, tf2_buffer, set_status, update_plan, get_thresholds):
		self.tf2_buffer = tf2_buffer
		self.set_status = set_status
		self.update_plan = update_plan
		self.get_thresholds = get_thresholds

		self.start_goal = None
		self.end_goal = None
		self.route = []
		self.route_index = 0

		self.path_sub = rospy.Subscriber("/waypoints/_path", Path, self.path_callback)
		self.clear_goals_sub = rospy.Subscriber("/waypoints/_clear", Empty, self.reset)

		self.vertical_pub = rospy.Publisher("/waypoints/_vertical_target", Float32, queue_size=1)

	def reset(self, msg):
		self.start_goal = None
		self.end_goal = None
		self.route = []
		self.route_index = 0
		self.update_plan()
		self.set_status(ControllerStatus.ESTOPPED, "Planner stopped.")

	def publish_vertical_target(self):
		msg = Float32()
		msg.data = self.end_goal.position.z
		self.vertical_pub.publish(msg)

	def robot_pose(self):
		try:
			return transform_to_pose(self.tf2_buffer.lookup_transform(PLANNING_FRAME, ROBOT_FRAME, rospy.Time(0)))
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
			rospy.logwarn("TF2 exception: %s", e)
			return None

	def to_planning_frame(self, msg: Path):
		"""The whole path is transformed once here, so everything downstream can assume the planning
		frame and compare poses against each other without checking headers again."""
		frame = msg.header.frame_id
		if not frame or frame == PLANNING_FRAME:
			return list(msg.poses)

		try:
			transform = self.tf2_buffer.lookup_transform(PLANNING_FRAME, frame, rospy.Time(0))
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
			rospy.logwarn("Cannot transform path from frame %s: %s", frame, e)
			return None

		rospy.loginfo("Path received in %s frame, transformed to %s.", frame, PLANNING_FRAME)
		return [do_transform_pose(pose, transform) for pose in msg.poses]

	def select_anchor_index(self, poses):
		"""Index of the pose to treat as the leg anchor, or None when the vessel is not on the path
		at all and has to transit to the start of it."""
		pose = self.robot_pose()
		if pose is None:
			return None

		rx, ry = pose.position.x, pose.position.y
		reach, divergence = self.get_thresholds()

		# Standing on a waypoint. Earliest occurrence, so a path that revisits a position does not
		# start at the wrong one: goal_and_return begins and ends where we are right now
		for i, candidate in enumerate(poses):
			p = candidate.pose.position
			if math.hypot(p.x - rx, p.y - ry) <= reach:
				return min(i, len(poses) - 2)

		# Not on a waypoint, so already somewhere along a leg
		for i in range(len(poses) - 1):
			a = poses[i].pose.position
			b = poses[i + 1].pose.position
			if point_segment_distance(rx, ry, a.x, a.y, b.x, b.y) <= divergence:
				return i

		return None

	def begin_transit(self, target):
		"""Anchor the leg at the vessel, for a path it is not on yet."""
		pose = self.robot_pose()
		if pose is None:
			return False

		self.start_goal = pose
		self.end_goal = target
		self.publish_vertical_target()
		return True

	def path_callback(self, msg: Path):
		if len(msg.poses) == 0:
			rospy.loginfo("Empty path, stopping.")
			self.reset(None)
			return

		poses = self.to_planning_frame(msg)
		if poses is None:
			return

		self.route = poses
		anchor = self.select_anchor_index(poses) if len(poses) >= 2 else None

		rospy.loginfo("------------------")

		if anchor is None:
			self.route_index = 0
			if not self.begin_transit(self.route[0].pose):
				return
			rospy.loginfo("Path of %d poses received, transiting to its start.", len(poses))
		else:
			self.route_index = anchor + 1
			self.start_goal = self.route[anchor].pose
			self.end_goal = self.route[anchor + 1].pose
			self.publish_vertical_target()
			rospy.loginfo("Path of %d poses received, picking up at leg %d.", len(poses), anchor)

		rospy.loginfo("Goal: X: %f, Y: %f, Z: %f", self.end_goal.position.x, self.end_goal.position.y, self.end_goal.position.z)

		self.update_plan()
		self.set_status(ControllerStatus.ACTIVE, f"Following leg {self.route_index - 1} of {len(poses) - 1}.")

	def get_goals(self):
		return self.start_goal, self.end_goal

	def goal_reached(self):
		if self.route_index < len(self.route) - 1:
			rospy.loginfo(f"Goal #{self.route_index} reached.")
			self.route_index += 1
			self.set_goal_pair(self.route[self.route_index].pose)
			self.set_status(ControllerStatus.ACTIVE, f"Goal #{self.route_index} reached, continuing route.")
		else:
			rospy.loginfo("-> Route finished.")
			self.start_goal = None
			self.end_goal = None
			self.set_status(ControllerStatus.FINISHED, "Route finished.")

		self.update_plan()

	def set_goal_pair(self, endgoal):
		self.start_goal = self.end_goal
		self.end_goal = endgoal
		self.publish_vertical_target()
		self.update_plan()

class LineFollowingController:
	def __init__(self):
		rospy.init_node("tinyhelm_waypoints")
		global ROBOT_FRAME, PLANNING_FRAME

		ROBOT_FRAME = rospy.get_param('/robot_frame', 'base_link')
		PLANNING_FRAME = rospy.get_param('/planning_frame', 'local')

		self.params = rospy.get_param("/tinyhelm_waypoints", {})
		if not self.params:
			rospy.logwarn("No parameters found under 'tinyhelm_waypoints'. Did you load the YAML file?")
			raise SystemExit(1)
		
		self.MIN_GOAL_XY_DIST = self.params.get('xy_distance_threshold')
		self.MIN_GOAL_Z_DIST = self.params.get('z_distance_threshold')

		self.MAX_LINEAR_SPD = self.params.get('max_linear_speed')
		self.MAX_VERTICAL_SPD = self.params.get('max_vertical_speed')
		self.MAX_ANGULAR_SPD = self.params.get('max_turning_speed')

		self.LINE_DIVERGENCE = self.params.get('max_line_divergence')
		self.ROBOT_WIDTH = self.params.get('robot_width')
		self.MIN_PROJECT_DIST = self.params.get('min_project_dist')
		self.MAX_PROJECT_DIST =self.params.get('max_project_dist')

		self.update_effective_divergence()

		self.SIDE_OFFSET_MULT = self.params.get('side_offset_mult')
		self.IGNORE_ALTITUDE = self.params.get('ignore_altitude')
		self.rate_hz = self.params.get('rate')
		self.RATE = rospy.Rate(self.rate_hz)

		self.markers = DebugMarkers(PLANNING_FRAME, "/waypoints/_markers")

		self.tf2_buffer = tf2_ros.Buffer()
		self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

		self.cmd_vel_pub = rospy.Publisher("/cmd_vel_waypoints", Twist, queue_size=1)

		self.status_pub = rospy.Publisher("/waypoints/_status", ControllerStatus, queue_size=1, latch=True)
		self.active_pub = rospy.Publisher("/waypoints/_active", Bool, queue_size=1, latch=True)
		self.plan_pub = rospy.Publisher("/waypoints/_plan", Path, queue_size=1, latch=True)

		self.pid = PID(self.params.get('P'), self.params.get('I'),self.params.get('D'))
		self.pid_vert = PID(self.params.get('P'), self.params.get('I'),self.params.get('D'))

		self.goal_server = GoalServer(self.tf2_buffer, self.set_status, self.update_plan, lambda: (self.MIN_GOAL_XY_DIST, self.LINE_DIVERGENCE))
		self.active = False

		self.reconfigure_server = DynamicReconfigureServer(WaypointsConfig, self.dynamic_reconfigure_callback)
		self.active_pub.publish(False)

		rospy.loginfo("Line planner started.")
		rospy.loginfo("Robot frame: "+ROBOT_FRAME)
		rospy.loginfo("Planning frame: "+PLANNING_FRAME)
		self.set_status(ControllerStatus.IDLE, "Waypoint planner initialized.")

	def update_effective_divergence(self):
		# The obstacle planner only clears a corridor max_line_divergence wide, so the hull stays
		# inside it only if we reach maximum correction half a robot width before the edge.
		self.effective_divergence = max(0.0, self.LINE_DIVERGENCE - self.ROBOT_WIDTH / 2.0)

		if self.effective_divergence == 0.0:
			rospy.logwarn("robot_width %.2f leaves no room inside max_line_divergence %.2f, correction will always be maximal.", self.ROBOT_WIDTH, self.LINE_DIVERGENCE)

	def set_status(self, status, string):
		msg = ControllerStatus()
		msg.status = status
		msg.message = string
		self.status_pub.publish(msg)

	def dynamic_reconfigure_callback(self, config, level):
		self.pid.kp = config.P
		self.pid.ki = config.I
		self.pid.kd = config.D

		self.pid_vert.kp = config.P
		self.pid_vert.ki = config.I
		self.pid_vert.kd = config.D

		self.MIN_GOAL_XY_DIST = config.xy_distance_threshold
		self.MIN_GOAL_Z_DIST = config.z_distance_threshold

		self.MAX_LINEAR_SPD = config.max_linear_speed
		self.MAX_ANGULAR_SPD = config.max_turning_speed
		self.MAX_VERTICAL_SPD = config.max_vertical_speed

		self.LINE_DIVERGENCE = config.max_line_divergence
		self.ROBOT_WIDTH = config.robot_width
		self.MIN_PROJECT_DIST = config.min_project_dist
		self.MAX_PROJECT_DIST = config.max_project_dist

		self.update_effective_divergence()

		self.SIDE_OFFSET_MULT = config.side_offset_mult

		self.IGNORE_ALTITUDE = config.ignore_altitude

		self.rate_hz = config.rate
		self.RATE = rospy.Rate(self.rate_hz)

		return config

	def get_angle_error(self, current_pose, target_position):
		_, _, current_yaw = euler_from_quaternion([
			current_pose.orientation.x,
			current_pose.orientation.y,
			current_pose.orientation.z,
			current_pose.orientation.w,
		])

		delta_x = target_position.x - current_pose.position.x
		delta_y = target_position.y - current_pose.position.y
		target_yaw = math.atan2(delta_y, delta_x)

		angle_error = target_yaw - current_yaw
		angle_error = -math.atan2(math.sin(angle_error), math.cos(angle_error))

		return angle_error

	def get_distance(self, pose, goal):
		deltax = goal.position.x - pose.position.x
		deltay = goal.position.y - pose.position.y
		return math.sqrt(deltax** 2 + deltay ** 2)

	def get_linear_velocity(self, distance, angle_error):
		ANGLE_40_RAD = math.radians(40)
		ANGLE_60_RAD = math.radians(60)
		ANGLE_120_RAD = math.radians(120)
		ANGLE_140_RAD = math.radians(140)

		abs_angle_error = math.fabs(angle_error)
		vel_multiplier = 0.0

		if abs_angle_error <= ANGLE_40_RAD:
			# Full speed ahead
			vel_multiplier = 1.0
		elif abs_angle_error <= ANGLE_60_RAD:
			# Linearly decrease speed from 100% at 40 deg to 0% at 60 deg.
			scale = (abs_angle_error - ANGLE_40_RAD) / (ANGLE_60_RAD - ANGLE_40_RAD)
			vel_multiplier = 1.0 - scale
		elif abs_angle_error <= ANGLE_120_RAD:
			# Turn in place between 60 and 120 deg
			vel_multiplier = 0.0
		elif abs_angle_error <= ANGLE_140_RAD:
			# Reverse from 0% at 120 deg to -100% at 180 deg for J turn behaviour.
			scale = (abs_angle_error - ANGLE_120_RAD) / (ANGLE_140_RAD - ANGLE_120_RAD)
			vel_multiplier = -scale
		elif abs_angle_error > ANGLE_140_RAD:
			vel_multiplier = -1.0


		linear_vel = self.MAX_LINEAR_SPD * vel_multiplier

		# Slow down while waiting for depth to match
		if distance < self.MIN_GOAL_XY_DIST:
			linear_vel *= 0.1

		return max(-self.MAX_LINEAR_SPD, min(linear_vel, self.MAX_LINEAR_SPD))


	def update(self):
		start_goal, end_goal = self.goal_server.get_goals()

		if start_goal == None or end_goal == None:
			if self.active:
				self.reset()
			return
		
		try:
			self.active = True
			self.active_pub.publish(True)
			pose = transform_to_pose(self.tf2_buffer.lookup_transform(PLANNING_FRAME, ROBOT_FRAME, rospy.Time(0)))

			target_position = project_position(
				start_goal,
				end_goal,
				pose,
				self.MIN_PROJECT_DIST,
				self.MAX_PROJECT_DIST,
				self.effective_divergence,
				self.SIDE_OFFSET_MULT
			)
			
			angle_error = self.get_angle_error(pose, target_position)
			angular_velocity = clamp(self.pid.compute(angle_error), -self.MAX_ANGULAR_SPD, self.MAX_ANGULAR_SPD)
			target_distance = self.get_distance(end_goal, pose)
			linear_velocity = self.get_linear_velocity(target_distance, angle_error)

			vertical_error = pose.position.z - end_goal.position.z
			vertical_velocity = clamp(self.pid_vert.compute(vertical_error), -self.MAX_VERTICAL_SPD, self.MAX_VERTICAL_SPD)
				
			if target_distance > self.MIN_GOAL_XY_DIST:
				pass
			elif math.fabs(vertical_error) > self.MIN_GOAL_Z_DIST and not self.IGNORE_ALTITUDE:
				angular_velocity = 0
			else:
				linear_velocity = 0
				angular_velocity = 0
				vertical_velocity = 0
				self.goal_server.goal_reached()

			self.send_twist(linear_velocity, angular_velocity, vertical_velocity)

			self.markers.draw_debug_markers(target_position, start_goal, end_goal, self.MIN_GOAL_XY_DIST, self.LINE_DIVERGENCE)

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.logwarn("TF Exception")

	def update_plan(self):

		msg = Path()
		msg.header.frame_id = PLANNING_FRAME

		start_goal, _ = self.goal_server.get_goals()

		if start_goal is None or not self.goal_server.route:
			self.plan_pub.publish(Path())
			return

		# Anchor of the leg in progress followed by everything still to visit, which is what the
		# monitors watch and what a revision of this plan has to line up with
		start = PoseStamped()
		start.header.frame_id = PLANNING_FRAME
		start.pose = start_goal

		msg.poses.append(start)
		msg.poses.extend(self.goal_server.route[self.goal_server.route_index:])

		self.plan_pub.publish(msg)

	def send_twist(self, linear, angular, vert):

		#sanity check, just in case
		if math.isnan(linear):
			linear = 0

		if math.isnan(angular):
			angular = 0

		if self.IGNORE_ALTITUDE or math.isnan(vert):
			vert = 0

		twist = Twist()
		twist.linear.x = linear
		twist.linear.z = vert
		twist.angular.z = angular
		self.cmd_vel_pub.publish(twist)

	def handle_time_jump(self):
		rospy.logwarn("Time moved backwards, dropping the mission and resetting.")
		self.tf2_buffer.clear()
		self.goal_server.reset(None)
		self.reset()
		self.RATE = rospy.Rate(self.rate_hz)

	def shutdown_cleanup(self):
		self.set_status(ControllerStatus.ERROR, "Waypoint planner shutdown.")
		self.reset()

	def reset(self):
		self.send_twist(0, 0, 0)

		self.markers.delete_debug_markers()

		self.active = False
		self.active_pub.publish(False)
		self.pid.reset()
		self.pid_vert.reset()

ctrl = LineFollowingController()
rospy.on_shutdown(ctrl.shutdown_cleanup)

last_time = rospy.Time.now()

while not rospy.is_shutdown():
	try:
		now = rospy.Time.now()
		if now < last_time:
			ctrl.handle_time_jump()
		last_time = now

		ctrl.update()
		ctrl.RATE.sleep()
	except rospy.exceptions.ROSTimeMovedBackwardsException:
		ctrl.handle_time_jump()
		last_time = rospy.Time.now()
