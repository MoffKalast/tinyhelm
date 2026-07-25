#!/usr/bin/env python3
import math
import rospy
import tf
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped, Twist, Point, TransformStamped, Pose
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, Bool, Empty
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

from tinyhelm_core.msg import ControllerStatus
from tinyhelm_stationkeeping.cfg import StationKeepingConfig

from tf2_geometry_msgs import do_transform_pose
from tf.transformations import euler_from_quaternion

from utils import transform_to_pose, normalize_angle, clamp, PID

class StationKeepingNode:
	def __init__(self):
		rospy.init_node("tinyhelm_stationkeeping")

		self.ROBOT_FRAME = rospy.get_param('/robot_frame', 'base_link')
		self.PLANNING_FRAME = rospy.get_param('/planning_frame', 'local')

		self.MAX_LINEAR_SPD = rospy.get_param("~max_linear_speed", 0.45)
		self.MAX_ANGULAR_SPD = rospy.get_param("~max_turning_speed", 0.9)
		self.MAX_DIVEGENCE = rospy.get_param("~max_divergence", 5.0)
		self.RATE = rospy.Rate(rospy.get_param("~rate", 30))

		self.DEADZONE_FRACT = rospy.get_param("~deadzone_fraction", 0.1)
		self.enabled = False
		self.goal_pose = None  # PoseStamped
		self.pid = PID(
			rospy.get_param('~P', 3.0),
			rospy.get_param('~I', 0.001),
			rospy.get_param('~D', 65.0)
		)
		self.deadzone = self.DEADZONE_FRACT * self.MAX_DIVEGENCE

		self.tf2_buffer = tf2_ros.Buffer()
		self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

		self.cmd_vel_pub = rospy.Publisher("/cmd_vel_stationkeeping", Twist, queue_size=1)
		self.marker_pub = rospy.Publisher("/stationkeeping/_markers", MarkerArray, queue_size=1)
		self.status_pub = rospy.Publisher("/stationkeeping/_status", ControllerStatus, queue_size=1, latch=True)

		self.goal_sub = rospy.Subscriber("/stationkeeping/_pose", PoseStamped, self.position_callback, queue_size=1)

		self.estop_sub = rospy.Subscriber("/stationkeeping/_clear", Empty, self.estop_callback)

		# Tracking enable and disable messages while publishing latched state
		self.enabled_sub = rospy.Subscriber("/stationkeeping/_enabled", Bool, self.enabled_callback)
		self.enabled_pub = rospy.Publisher("/stationkeeping/_enabled", Bool, queue_size=1, latch=True)
		self.enabled_pub.publish(self.enabled)

		# Dynamic reconfigure server
		self.dyn_srv = DynamicReconfigureServer(StationKeepingConfig, self.reconfigure_callback)

		self.set_status(ControllerStatus.IDLE, "Stationkeeping initialized.")
		rospy.loginfo("Station keeping node initialized")

	def set_status(self, status, string):
		msg = ControllerStatus()
		msg.status = status
		msg.message = string
		self.status_pub.publish(msg)

	def reconfigure_callback(self, config, level):
		self.MAX_LINEAR_SPD = config.max_linear_speed
		self.MAX_ANGULAR_SPD = config.max_turning_speed
		self.MAX_DIVEGENCE = config.max_divergence
		self.RATE = rospy.Rate(config.rate)
		self.DEADZONE_FRACT = config.deadzone_fraction
		self.deadzone = self.DEADZONE_FRACT * self.MAX_DIVEGENCE

		self.pid.kp = config.P
		self.pid.ki = config.I
		self.pid.kd = config.D
		return config

	def position_callback(self, msg: PoseStamped):

		if self.PLANNING_FRAME == msg.header.frame_id:
			rospy.loginfo("------------------")
			rospy.loginfo("Received target in planning ("+self.PLANNING_FRAME+") frame.")
		else:
			try:
				transform = self.tf2_buffer.lookup_transform(self.PLANNING_FRAME, msg.header.frame_id, rospy.Time(0))
				msg = do_transform_pose(msg, transform)
				rospy.loginfo("------------------")
				rospy.loginfo("Received goal in "+msg.header.frame_id+" frame, transformed to "+self.PLANNING_FRAME+".")
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
				rospy.logwarn("TF2 exception: %s", e)
				return

		pose = msg.pose
		rospy.loginfo("Position: X: %f, Y: %f, Z: %f", pose.position.x, pose.position.y, pose.position.z)
		rospy.loginfo("Orientation: X: %f, Y: %f, Z: %f, W: %f", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

		self.set_status(ControllerStatus.ACTIVE, f"Holding position {pose.position.x},{pose.position.y}.")

		self.goal_pose = msg
		self.enabled_callback(Bool(True))

	def estop_callback(self, msg: Empty):
		self.enabled_callback(Bool(False))

	def enabled_callback(self, msg: Bool):
		if msg.data != self.enabled:
			self.enabled = msg.data
			self.enabled_pub.publish(self.enabled)

			if not self.enabled:
				self.send_twist(0, 0)
				self.delete_debug_markers()
				self.goal_pose = None
				rospy.loginfo("Stationkeeping stopped.")
				self.set_status(ControllerStatus.ESTOPPED, f"Stationkeeping stopped.")

	def get_goal_distance(self, transform: TransformStamped):
		goal_x = self.goal_pose.pose.position.x
		goal_y = self.goal_pose.pose.position.y

		x = transform.transform.translation.x
		y = transform.transform.translation.y

		return math.hypot(goal_x - x, goal_y - y)
	
	def get_angle_error(self, current_pose: Pose, target_position: Point):
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

	def update(self):
		if not self.enabled:
			return

		trans = self.tf2_buffer.lookup_transform(self.PLANNING_FRAME, self.ROBOT_FRAME, rospy.Time(0))

		if self.goal_pose is None:
			self.goal_pose = PoseStamped()
			self.goal_pose.header.frame_id = self.PLANNING_FRAME
			self.goal_pose.pose = transform_to_pose(trans)
			self.set_status(ControllerStatus.ACTIVE, f"Stationkeeping started in place.")

		self.publish_markers()

		dist = self.get_goal_distance(trans)

		if dist < self.deadzone:
			self.send_twist(0, 0)
			return

		heading_error = self.get_angle_error(transform_to_pose(trans), self.goal_pose.pose.position)

		# Decide forward or reverse
		forward = True
		if abs(heading_error) > math.pi / 2.0:
			forward = False
			# Flip to reverse heading
			if heading_error > 0:
				heading_error -= math.pi
			else:
				heading_error += math.pi
			heading_error = normalize_angle(heading_error)

		# Angular velocity via PID
		angular_vel = self.pid.compute(heading_error)

		# Linear velocity scaling
		frac = (dist - self.deadzone) / (self.MAX_DIVEGENCE - self.deadzone)
		frac = clamp(frac, 0.0, 1.0)
		linear_vel = self.MAX_LINEAR_SPD * math.pow(frac, 0.7)

		# Smooth ramp based on heading error
		min_angle = math.radians(5)
		max_angle = math.radians(20)

		if heading_error > min_angle:
			# Map heading_error from [min_angle, max_angle] -> [1, 0]
			scale = 1.0 - clamp((heading_error - min_angle) / (max_angle - min_angle), 0.0, 1.0)
			linear_vel *= scale

		if not forward:
			linear_vel = -linear_vel

		self.send_twist(linear_vel, angular_vel)

	def run(self):
		while not rospy.is_shutdown():
			try:
				self.update()
			except Exception as e:
				rospy.logwarn(str(e))
				self.set_status(ControllerStatus.ERROR, str(e))
			self.RATE.sleep()

	def send_twist(self, linear, angular):
		if math.isnan(linear):
			linear = 0
		if math.isnan(angular):
			angular = 0

		linear = clamp(linear, -self.MAX_LINEAR_SPD, self.MAX_LINEAR_SPD)
		angular = clamp(angular, -self.MAX_ANGULAR_SPD, self.MAX_ANGULAR_SPD)

		twist = Twist()
		twist.linear.x = linear
		twist.angular.z = angular
		self.cmd_vel_pub.publish(twist)

	def delete_debug_markers(self):
		marker = Marker()
		marker.action = Marker.DELETEALL
		markerArray = MarkerArray()
		markerArray.markers.append(marker)
		self.marker_pub.publish(markerArray)

	def publish_markers(self):
		if self.goal_pose is None:
			return

		arr = MarkerArray()
		header = Header(frame_id=self.PLANNING_FRAME, stamp=rospy.Time.now())

		deadzone = self.DEADZONE_FRACT * self.MAX_DIVEGENCE

		def make_circle_marker(radius, color, ns, mid):
			marker = Marker()
			marker.header = header
			marker.ns = ns
			marker.id = mid
			marker.type = Marker.LINE_STRIP
			marker.action = Marker.ADD
			marker.scale.x = 0.05
			marker.color.r = color[0]
			marker.color.g = color[1]
			marker.color.b = color[2]
			marker.color.a = color[3]

			pts = []
			N = 64
			for i in range(N + 1):  # close the loop
				angle = 2 * math.pi * i / N
				p = Point()
				p.x = self.goal_pose.pose.position.x + radius * math.cos(angle)
				p.y = self.goal_pose.pose.position.y + radius * math.sin(angle)
				p.z = 0.0
				pts.append(p)
			marker.points = pts
			return marker

		# Deadzone circle (green, semi-transparent)
		arr.markers.append(make_circle_marker(
			deadzone, (0.0, 1.0, 0.0, 0.4), "deadzone", 0
		))

		# Max divergence circle (blue, semi-transparent)
		arr.markers.append(make_circle_marker(
			self.MAX_DIVEGENCE, (0.0, 0.0, 1.0, 0.4), "divergence", 1
		))

		# Goal center marker (red point/small sphere)
		center = Marker()
		center.header = header
		center.ns = "goal_center"
		center.id = 2
		center.type = Marker.SPHERE
		center.action = Marker.ADD
		center.pose.position = self.goal_pose.pose.position
		center.pose.orientation.w = 1.0
		center.scale.x = 0.15
		center.scale.y = 0.15
		center.scale.z = 0.15
		center.color.r = 1.0
		center.color.g = 0.0
		center.color.b = 0.0
		center.color.a = 0.9
		arr.markers.append(center)

		self.marker_pub.publish(arr)


if __name__ == "__main__":
	try:
		node = StationKeepingNode()
		node.run()
	except rospy.ROSInterruptException:
		pass
