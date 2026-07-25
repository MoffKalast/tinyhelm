#!/usr/bin/env python3
import math

import rospy
import tf2_geometry_msgs
import tf2_ros

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

from mission_state import WALK_ABANDONED, WALK_RUNNING, WALK_WITHHELD, MissionState, ReplanWalk, drop_passed_legs
from tinyhelm_core.msg import MonitorStatus
from tinyhelm_obstacles.msg import PathStatus, PathWatch, PlanReply, PlanRequest

class ObstacleMonitorNode:
	"""Watches the mission the helm is executing and the route it is currently steering, and when that
	route is obstructed proposes a corrected one. It commands nothing: it publishes a revised path and
	a status, and the helm decides what to do with them.

	There is no tick. Everything happens because something arrived: a mission, a route, a report that
	the route is obstructed, or a reply from the planner. The one timer here is a watchdog on an
	outstanding request, which exists because a planner that has died or dropped a message is
	otherwise indistinguishable from one still thinking.

	Nothing in this node touches a grid or runs a search, which is the point: the expensive work sits
	behind a topic pair and this side stays responsive enough to keep the helm honest."""

	def __init__(self):
		self.planning_frame = rospy.get_param("/planning_frame", "local")
		self.robot_frame = rospy.get_param("/robot_frame", "base_link")

		self.divergence_param = rospy.get_param("~divergence_param", "/tinyhelm_waypoints/max_line_divergence")
		self.soft_radius = rospy.get_param("~soft_radius", 15.0)
		self.corridor_radius = rospy.get_param("~max_lateral_detour", 20.0)
		self.request_timeout = rospy.get_param("~request_timeout", 2.0)
		self.request_retries = rospy.get_param("~request_retries", 2)

		self.state = MissionState(
			self.corridor_radius,
			rospy.get_param("~waypoint_reached_radius", 6.0),
			rospy.get_param("~unreachable_cycles", 3),
		)

		self.current_path = []
		self.walk = None
		self.request_id = 0
		self.awaiting = None
		self.awaiting_since = rospy.Time(0)
		self.attempts = 0
		self.blocked = False

		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

		self.path_pub = rospy.Publisher("/obstacles/_revised_path_out", Path, queue_size=1, latch=True)
		self.status_pub = rospy.Publisher("/obstacles/_status", MonitorStatus, queue_size=5, latch=True)
		self.remaining_pub = rospy.Publisher("/obstacles/_remaining", Path, queue_size=1, latch=True)
		self.markers_pub = rospy.Publisher("/obstacles/_markers", MarkerArray, queue_size=1, latch=True)

		self.request_pub = rospy.Publisher("/obstacles/plan_request", PlanRequest, queue_size=1)
		self.watch_pub = rospy.Publisher("/obstacles/path_watch", PathWatch, queue_size=1, latch=True)

		rospy.Subscriber("/obstacles/_mission_in", Path, self.mission_callback, queue_size=1)
		rospy.Subscriber("/obstacles/_current_path_in", Path, self.current_path_callback, queue_size=1)
		rospy.Subscriber("/obstacles/path_status", PathStatus, self.path_status_callback, queue_size=1)
		rospy.Subscriber("/obstacles/plan_reply", PlanReply, self.plan_reply_callback, queue_size=5)

		self.watchdog = rospy.Timer(rospy.Duration(0.5), self.check_timeout)

		self.publish_status(MonitorStatus.OK, "No active mission.")
		rospy.loginfo("obstacle monitor: corridor radius %.0fm, clearance follows %s", self.corridor_radius, self.divergence_param)

	def clearance(self):
		"""How far the route must stay from an obstacle. This is the controller's allowed divergence
		from its line, because the vessel may sit anywhere within that band under wind or current and
		the corridor has to be clear wherever it ends up. The controller holds its hull inside the same
		band by giving up half a robot width of it, so the margin is accounted for once and here."""
		divergence = rospy.get_param(self.divergence_param, None)
		if divergence is None:
			rospy.logwarn_throttle(30.0, "obstacle monitor: %s is unset, falling back to the corridor radius" % self.divergence_param)
			return self.corridor_radius

		return divergence

	def robot_position(self):
		try:
			tf = self.tf_buffer.lookup_transform(self.planning_frame, self.robot_frame, rospy.Time(0))
			return tf.transform.translation.x, tf.transform.translation.y
		except tf2_ros.TransformException as e:
			rospy.logwarn_throttle(5.0, "obstacle monitor: no %s -> %s: %s" % (self.planning_frame, self.robot_frame, e))
			return None

	def to_planning_frame(self, msg):
		"""Everything downstream assumes the planning frame, and untransformed coordinates would have
		it monitoring and planning somewhere else entirely without saying so."""
		frame = msg.header.frame_id
		if not frame or frame == self.planning_frame or not msg.poses:
			return list(msg.poses)

		try:
			tf = self.tf_buffer.lookup_transform(self.planning_frame, frame, rospy.Time(0))
		except tf2_ros.TransformException as e:
			rospy.logwarn("obstacle monitor: mission in '%s' could not be transformed, ignoring it: %s" % (frame, e))
			return None

		return [tf2_geometry_msgs.do_transform_pose(pose, tf) for pose in msg.poses]

	def mission_callback(self, msg):
		poses = self.to_planning_frame(msg)
		if poses is None:
			return

		self.abandon_walk()
		self.state.set_mission([(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in poses])
		self.blocked = False

		if not self.state.active():
			self.current_path = []
			self.publish_watch()
			self.publish_markers([])
			self.publish_status(MonitorStatus.OK, "No active mission.")
			return

		rospy.loginfo("obstacle monitor: mission of %d waypoints", len(self.state.mission))
		self.refresh_progress()
		self.publish_status(MonitorStatus.OK, "Mission accepted, corridor clear so far.")

	def current_path_callback(self, msg):
		poses = self.to_planning_frame(msg)
		if poses is None:
			return

		self.current_path = [(p.pose.position.x, p.pose.position.y) for p in poses]
		self.refresh_progress()
		self.publish_watch()

	def refresh_progress(self):
		position = self.robot_position()
		if position is None:
			return

		if self.state.update_progress(position[0], position[1]):
			rospy.loginfo("obstacle monitor: %d waypoints remaining", len(self.state.remaining()))

		self.publish_remaining()
		self.publish_markers(self.state.corridor_polyline(position[0], position[1]))

	def path_status_callback(self, msg):
		"""The planner reports on the route rather than the monitor inspecting the map, so this is the
		only place an intrusion is learned about."""
		self.refresh_progress()

		if not msg.blocked:
			if self.blocked:
				rospy.loginfo("obstacle monitor: route clear again")
				self.abandon_walk()
			self.blocked = False
			self.publish_status(MonitorStatus.OK, "Route clear.")
			return

		self.blocked = True
		if self.walk is not None or self.awaiting is not None:
			return

		if not self.state.active():
			self.publish_status(MonitorStatus.WARN, "Route obstructed but no mission to correct against.")
			return

		rospy.logwarn("obstacle monitor: route obstructed on leg %d, %.0fm ahead, clearance %.1fm", msg.blocked_leg, msg.blocked_distance, msg.min_clearance)
		self.begin_walk()

	def begin_walk(self):
		position = self.robot_position()
		if position is None:
			return

		self.walk = ReplanWalk(self.state, position[0], position[1])
		self.send_next()

	def abandon_walk(self):
		self.walk = None
		self.awaiting = None
		self.attempts = 0

	def send_next(self):
		pending = self.walk.pending_request() if self.walk else None
		if pending is None:
			self.finish_walk(self.walk.settle() if self.walk else WALK_ABANDONED)
			return

		start, goal, index = pending
		position = self.robot_position()
		if position is None:
			self.abandon_walk()
			return

		self.request_id += 1
		self.awaiting = self.request_id
		self.awaiting_since = rospy.Time.now()
		self.attempts = 1
		self.publish_request(start, goal, position)

	def publish_request(self, start, goal, position):
		msg = PlanRequest()
		msg.request_id = self.awaiting
		msg.start = Point(start[0], start[1], 0.0)
		msg.goal = Point(goal[0], goal[1], 0.0)
		msg.clearance = self.clearance()
		msg.soft_radius = self.soft_radius
		msg.corridor = [Point(x, y, 0.0) for x, y in self.state.corridor_polyline(position[0], position[1])]
		msg.corridor_radius = self.corridor_radius
		self.request_pub.publish(msg)

	def resend(self):
		pending = self.walk.pending_request() if self.walk else None
		position = self.robot_position()
		if pending is None or position is None:
			self.abandon_walk()
			return

		start, goal, _ = pending
		self.attempts += 1
		self.awaiting_since = rospy.Time.now()
		self.publish_request(start, goal, position)

	def check_timeout(self, _):
		"""A planner that has died and one still thinking look identical from here, so an outstanding
		request is given a bounded number of attempts and then the correction is dropped rather than
		waiting on it forever."""
		if self.awaiting is None:
			return

		if (rospy.Time.now() - self.awaiting_since).to_sec() < self.request_timeout:
			return

		if self.attempts > self.request_retries:
			rospy.logerr("obstacle monitor: planner did not answer request %d after %d attempts, dropping the correction", self.awaiting, self.attempts)
			self.abandon_walk()
			self.publish_status(MonitorStatus.INTERNAL_ERROR, "Planner is not answering.")
			return

		rospy.logwarn("obstacle monitor: request %d timed out, resending", self.awaiting)
		self.resend()

	def plan_reply_callback(self, msg):
		if self.walk is None or self.awaiting is None or msg.request_id != self.awaiting:
			return

		self.awaiting = None
		index = self.walk.target_index()

		reachable = msg.result == PlanReply.OK and len(msg.path) >= 2
		outcome = self.walk.accept(reachable, [(p.x, p.y) for p in msg.path], self.reason_text(msg.result))

		if outcome == WALK_RUNNING:
			self.send_next()
			return

		self.finish_walk(outcome, index, msg.result)

	def reason_text(self, result):
		return {
			PlanReply.GOAL_IN_OBSTACLE: "waypoint is inside an obstacle",
			PlanReply.GOAL_OUTSIDE_CORRIDOR: "waypoint lies outside its own corridor",
			PlanReply.START_TRAPPED: "no clear water around the vessel to start from",
			PlanReply.NO_ROUTE: "no route within the corridor",
			PlanReply.NO_COSTMAP: "no costmap yet",
		}.get(result, "unknown")

	def finish_walk(self, outcome, index=None, result=None):
		walk = self.walk
		self.abandon_walk()

		if outcome == WALK_WITHHELD:
			self.publish_status(MonitorStatus.WARN, "Waypoint %s unreachable (%s), confirming before correcting." % (index, self.reason_text(result)))
			return

		if outcome == WALK_ABANDONED or walk is None or len(walk.points) < 2:
			self.publish_status(MonitorStatus.OBSERVED_ERROR, "No usable corrected course through the remaining waypoints.")
			return

		self.publish_revision(walk)

	def publish_revision(self, walk):
		position = self.robot_position()
		points = walk.points
		if position is not None:
			points = drop_passed_legs(points, position[0], position[1])
			if len(points) != len(walk.points):
				rospy.loginfo("obstacle monitor: correction opened %d leg(s) astern, trimmed", len(walk.points) - len(points))

		msg = Path()
		msg.header.frame_id = self.planning_frame
		msg.header.stamp = rospy.Time.now()
		msg.poses = [self.pose(x, y, z) for x, y, z in points]
		self.path_pub.publish(msg)

		# Watch the correction from here rather than the route it replaces. The controller will echo
		# its own version back once the helm has relayed it, but that is a full round trip away, and
		# until then the planner would keep reporting the superseded route obstructed and we would keep
		# solving the same correction over and over.
		self.current_path = [(x, y) for x, y, _ in points]
		self.publish_watch()

		if walk.any_skipped:
			detail = "; ".join("waypoint %d %s" % (i, why) for i, why in walk.reasons)
			self.publish_status(MonitorStatus.REPLAN, "Corrected course around obstacles, skipping %s." % detail)
		else:
			self.publish_status(MonitorStatus.REPLAN, "Corrected course planned around obstacles.")

		rospy.loginfo("obstacle monitor: published a correction of %d poses", len(points))

	def pose(self, x, y, z):
		p = PoseStamped()
		p.header.frame_id = self.planning_frame
		p.pose.position.x = x
		p.pose.position.y = y
		p.pose.position.z = z
		p.pose.orientation.w = 1.0
		return p

	def publish_remaining(self):
		msg = Path()
		msg.header.frame_id = self.planning_frame
		msg.header.stamp = rospy.Time.now()
		msg.poses = [self.pose(x, y, z) for x, y, z in self.state.remaining()]
		self.remaining_pub.publish(msg)

	def publish_watch(self):
		msg = PathWatch()
		msg.path = [Point(x, y, 0.0) for x, y in self.current_path]
		msg.clearance = self.clearance()
		self.watch_pub.publish(msg)

	def publish_status(self, status, message):
		msg = MonitorStatus()
		msg.status = status
		msg.message = message
		self.status_pub.publish(msg)

	def publish_markers(self, polyline):
		arr = MarkerArray()
		stamp = rospy.Time.now()
		for i in range(1, len(polyline)):
			arr.markers.append(self.corridor_marker(polyline[i - 1], polyline[i], i - 1, stamp))

		self.markers_pub.publish(arr)

	def corridor_marker(self, a, b, marker_id, stamp):
		"""One leg of the corridor drawn as a stadium. This is the boundary the search is actually
		held to, so it doubles as a picture of how much space a correction has to work in."""
		marker = Marker()
		marker.header.frame_id = self.planning_frame
		marker.header.stamp = stamp
		marker.ns = "corridor"
		marker.id = marker_id
		marker.type = Marker.LINE_STRIP
		marker.action = Marker.ADD
		marker.scale.x = 0.3
		marker.pose.orientation.w = 1.0
		marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.2, 0.6, 1.0, 0.7

		dx, dy = b[0] - a[0], b[1] - a[1]
		length = math.hypot(dx, dy)
		ux, uy = (dx / length, dy / length) if length > 1e-6 else (1.0, 0.0)
		nx, ny = -uy, ux
		r = self.corridor_radius
		arc = 20

		# Half turn round the far end, then half turn round the near end; the straight flanks fall out
		# of joining the two arcs
		points = []
		for ex, ey, base in ((b[0], b[1], 0.0), (a[0], a[1], math.pi)):
			for i in range(arc + 1):
				theta = base + math.pi * i / arc
				c, s = math.cos(theta), math.sin(theta)
				points.append(Point(ex + r * (c * nx + s * ux), ey + r * (c * ny + s * uy), 0.0))

		points.append(points[0])
		marker.points = points
		return marker

if __name__ == "__main__":
	rospy.init_node("tinyhelm_obstacles")
	ObstacleMonitorNode()
	rospy.spin()
