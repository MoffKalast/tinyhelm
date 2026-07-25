#!/usr/bin/env python3
from xmlrpc.client import Boolean
import rospy
import math
import tf
import tf2_ros

from utils import *
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

	def __init__(self, tf2_buffer, set_status, update_plan):
		self.start_goal = None
		self.end_goal = None
		self.tf2_buffer = tf2_buffer
		self.set_status = set_status
		self.update_plan = update_plan

		self.simple_goal_sub = rospy.Subscriber("/waypoints/_path", Path, self.route_callback)
		self.simple_goal_sub = rospy.Subscriber("/waypoints/_goal", PoseStamped, self.goal_callback)
		self.revise_sub = rospy.Subscriber("/waypoints/_revise", Path, self.revise_callback)
		self.clear_goals_sub = rospy.Subscriber("/waypoints/_clear", Empty, self.reset)

		self.vertical_pub = rospy.Publisher("/waypoints/_vertical_target", Float32, queue_size=1)

		self.start_goal = None
		self.end_goal = None
		self.route = []
		self.route_index = 0

	def reset(self,msg):
		self.start_goal = None
		self.end_goal = None
		self.route = []
		self.route_index = 0
		self.update_plan()
		self.set_status(ControllerStatus.ESTOPPED, "Planner stopped.")

	def goal_callback(self, goal):
		self.route = []
		self.route_index = 0
		self.process_goal(goal)
		self.update_plan()

		self.set_status(ControllerStatus.ACTIVE, "Navigation started!")

	def route_callback(self, msg):
		rospy.loginfo("New route received.")

		if len(msg.poses) == 0:
			rospy.loginfo("Empty route, stopping.")
			self.reset(None)
			return

		self.route = msg.poses
		self.route_index = 0
		self.process_goal(self.route[0])
		self.update_plan()

		self.set_status(ControllerStatus.ACTIVE, "Navigation started!")

	def revise_callback(self, msg):
		if len(msg.poses) < 2:
			rospy.logwarn("Revised plan needs at least two poses, ignoring.")
			return

		frame = msg.header.frame_id
		if frame and frame != PLANNING_FRAME:
			try:
				transform = self.tf2_buffer.lookup_transform(PLANNING_FRAME, frame, rospy.Time(0))
				msg.poses = [do_transform_pose(p, transform) for p in msg.poses]
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
				rospy.logwarn("Cannot transform revised plan from frame %s: %s", frame, e)
				return

		# A revision differs from a new route: the first pose is the line anchor of the leg
		# in progress, not a goal to visit, so there is no demand to backtrack to it
		rospy.loginfo("Plan revised: %d poses.", len(msg.poses))
		self.route = msg.poses
		self.route_index = 1
		self.start_goal = msg.poses[0].pose
		self.end_goal = msg.poses[1].pose

		vert_msg = Float32()
		vert_msg.data = self.end_goal.position.z
		self.vertical_pub.publish(vert_msg)

		self.update_plan()
		self.set_status(ControllerStatus.ACTIVE, "Plan revised.")

	def get_goals(self):
		return self.start_goal, self.end_goal
	
	def goal_reached(self):
		if len(self.route) > 0:
			rospy.loginfo(f"Goal #{self.route_index} reached.")
			if self.route_index < len(self.route)-1:
				self.route_index +=1
				self.process_goal(self.route[self.route_index])
				self.set_status(ControllerStatus.ACTIVE, f"Goal #{self.route_index} reached, continuing route.")
			else:
				rospy.loginfo("-> Route finished.")
				self.start_goal = None
				self.end_goal = None
				self.set_status(ControllerStatus.FINISHED, "Route finished.")
		else:
			rospy.loginfo("Simple goal reached.")
			self.start_goal = None
			self.end_goal = None
			self.set_status(ControllerStatus.FINISHED, "Simple goal reached.")

		self.update_plan()


	def set_goal_pair(self, endgoal):
		if len(self.route) == 0 or self.route_index == 0:
			try:
				self.start_goal = transform_to_pose(self.tf2_buffer.lookup_transform(PLANNING_FRAME, ROBOT_FRAME, rospy.Time(0)))
				self.end_goal = endgoal
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
				rospy.logwarn("TF2 exception: %s", e)
		else:
			self.start_goal = self.end_goal
			self.end_goal = endgoal

		vert_msg = Float32()
		vert_msg.data = self.end_goal.position.z
		self.vertical_pub.publish(vert_msg)

		self.update_plan()

	def process_goal(self, goal):
		if PLANNING_FRAME == goal.header.frame_id:
			rospy.loginfo("------------------")
			rospy.loginfo("Received goal in planning ("+PLANNING_FRAME+") frame.")
			rospy.loginfo("Position: X: %f, Y: %f, Z: %f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z)
			rospy.loginfo("Orientation: X: %f, Y: %f, Z: %f, W: %f", goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w)
			self.set_goal_pair(goal.pose)
		else:
			try:
				transform = self.tf2_buffer.lookup_transform(PLANNING_FRAME, goal.header.frame_id, rospy.Time(0))
				goal_transformed = do_transform_pose(goal, transform)
				self.set_goal_pair(goal_transformed.pose)
				rospy.loginfo("------------------")
				rospy.loginfo("Received goal in "+goal.header.frame_id+" frame, transformed to "+PLANNING_FRAME+".")
				rospy.loginfo("Position: X: %f, Y: %f, Z: %f", goal_transformed.pose.position.x, goal_transformed.pose.position.y, goal_transformed.pose.position.z)
				rospy.loginfo("Orientation: X: %f, Y: %f, Z: %f, W: %f", goal_transformed.pose.orientation.x, goal_transformed.pose.orientation.y, goal_transformed.pose.orientation.z, goal_transformed.pose.orientation.w)

			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
				rospy.logwarn("TF2 exception: %s", e)
				return

class LineFollowingController:
	def __init__(self):
		rospy.init_node("tinyhelm_waypoints")
		global ROBOT_FRAME, PLANNING_FRAME

		ROBOT_FRAME = rospy.get_param('/robot_frame', 'base_link')
		PLANNING_FRAME = rospy.get_param('/planning_frame', 'local')
		
		self.MIN_GOAL_XY_DIST = rospy.get_param('~xy_distance_threshold', 2.0)
		self.MIN_GOAL_Z_DIST = rospy.get_param('~z_distance_threshold', 0.5)

		self.MAX_LINEAR_SPD = rospy.get_param('~max_linear_speed', 1.0)
		self.MAX_VERTICAL_SPD = rospy.get_param('~max_vertical_speed', 0.5)
		self.MAX_ANGULAR_SPD = rospy.get_param('~max_turning_speed', 2.0)

		self.LINE_DIVERGENCE = rospy.get_param('~max_line_divergence', 1.0)
		self.ROBOT_WIDTH = rospy.get_param('~robot_width', 0.0)
		self.MIN_PROJECT_DIST = rospy.get_param('~min_project_dist', 0.15)
		self.MAX_PROJECT_DIST = rospy.get_param('~max_project_dist', 1.2)

		self.update_effective_divergence()

		self.SIDE_OFFSET_MULT = rospy.get_param('~side_offset_mult', 0.8)

		self.IGNORE_ALTITUDE = rospy.get_param('~ignore_altitude', True)

		self.RATE = rospy.Rate(rospy.get_param('~rate', 30))

		self.markers = DebugMarkers(PLANNING_FRAME, "/waypoints/_markers")

		self.tf2_buffer = tf2_ros.Buffer()
		self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

		self.cmd_vel_pub = rospy.Publisher("/cmd_vel_waypoints", Twist, queue_size=1)

		self.status_pub = rospy.Publisher("/waypoints/_status", ControllerStatus, queue_size=1, latch=True)
		self.active_pub = rospy.Publisher("/waypoints/_active", Bool, queue_size=1, latch=True)
		self.plan_pub = rospy.Publisher("/waypoints/_plan", Path, queue_size=1, latch=True)

		self.pid = PID(
			rospy.get_param('~P', 3.0),
			rospy.get_param('~I', 0.001), 
			rospy.get_param('~D', 65.0)
		)

		self.pid_vert = PID(
			rospy.get_param('~P', 3.0),
			rospy.get_param('~I', 0.001), 
			rospy.get_param('~D', 65.0)
		)

		self.goal_server = GoalServer(self.tf2_buffer, self.set_status, self.update_plan)
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

		self.RATE = rospy.Rate(config.rate)

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
			# Linearly decrease speed from 100% at 30 deg to 0% at 60 deg.
			scale = (abs_angle_error - ANGLE_40_RAD) / (ANGLE_60_RAD - ANGLE_40_RAD)
			vel_multiplier = 1.0 - scale
		elif abs_angle_error <= ANGLE_120_RAD:
			# Turn in place between 60 and 120 deg
			vel_multiplier = 0.0
		elif abs_angle_error <= ANGLE_140_RAD:
			# Reverse from 0% at 120 deg to -100% at 180 deg for J turn behaviour.
			scale = (abs_angle_error - ANGLE_120_RAD) / (ANGLE_140_RAD - ANGLE_120_RAD)
			vel_multiplier = -scale

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

		start_goal, end_goal = self.goal_server.get_goals()

		if start_goal is None:
			self.plan_pub.publish(Path())
			return

		if len(self.goal_server.route) > 0:
			start = PoseStamped()
			start.header.frame_id = PLANNING_FRAME
			start.pose = start_goal

			msg.poses.append(start)
			msg.poses.extend(self.goal_server.route[self.goal_server.route_index:])

		else:
			if end_goal is None:
				self.plan_pub.publish(Path())
				return

			start_stamped = PoseStamped()
			start_stamped.header.frame_id = PLANNING_FRAME
			start_stamped.pose = start_goal

			end_stamped = PoseStamped()
			end_stamped.header.frame_id = PLANNING_FRAME
			end_stamped.pose = end_goal

			msg.poses = [start_stamped, end_stamped]

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

while not rospy.is_shutdown():
	ctrl.update()
	ctrl.RATE.sleep()
