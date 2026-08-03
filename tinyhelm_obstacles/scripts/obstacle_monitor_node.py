#!/usr/bin/env python3
import math

import rospy
import tf2_ros

from geometry_msgs.msg import Point
from nav_msgs.msg import Path

from markers import DebugMarkers
from tinyhelm_core.msg import MonitorStatus
from tinyhelm_obstacles.msg import PathStatus, PathWatch, PlanReply, PlanRequest
from utils import PendingRequest, make_path, path_to_planning_frame, poses_to_xy, poses_to_xyz, robot_position, tail_cursor

ROUTE_MATCH = 0.01
REACHED_WAYPOINT = 0.01
SAME_CORRECTION = 0.5

NO_MISSION = "no_mission"
CLEAR = "clear"
CORRECTING = "correcting"
STUCK = "stuck"

REASON_TEXT = {
	PlanReply.GOAL_IN_OBSTACLE: "waypoint is inside an obstacle",
	PlanReply.GOAL_OUTSIDE_CORRIDOR: "waypoint lies outside its own corridor",
	PlanReply.START_TRAPPED: "no clear water around the vessel to start from",
	PlanReply.NO_ROUTE: "no route within the corridor",
	PlanReply.NO_COSTMAP: "no costmap yet",
	PlanReply.INTERNAL_ERROR: "the planner failed internally",
	PlanReply.OK: "ok",
}

class Correction:

	def __init__(self, mission, first_index, rx, ry):
		self.mission = mission
		self.targets = list(range(first_index, len(mission)))
		self.cursor = 0
		self.from_x = rx
		self.from_y = ry
		self.points = []
		self.origins = []
		self.skipped = []

	def target_index(self):
		return self.targets[self.cursor] if self.cursor < len(self.targets) else None

	def pending(self):
		index = self.target_index()
		if index is None:
			return None

		return (self.from_x, self.from_y), (self.mission[index][0], self.mission[index][1]), index

	def accept(self, path, index):
		z = self.mission[index][2]
		for x, y in path:
			if self.points and abs(self.points[-1][0] - x) < 1e-6 and abs(self.points[-1][1] - y) < 1e-6:
				continue
			self.points.append((x, y, z))
			self.origins.append(None)

		if not self.points:
			self.cursor += 1
			return

		if math.hypot(self.points[-1][0] - self.mission[index][0], self.points[-1][1] - self.mission[index][1]) <= REACHED_WAYPOINT:
			self.points[-1] = self.mission[index]

		self.origins[-1] = index
		self.from_x, self.from_y = self.points[-1][0], self.points[-1][1]
		self.cursor += 1

	def entries(self):
		return [(x, y, z, origin) for (x, y, z), origin in zip(self.points, self.origins)]

	def skip(self, index):
		self.skipped.append(index)
		self.cursor += 1

class ObstacleMonitorNode:

	def __init__(self):
		self.planning_frame = rospy.get_param("/planning_frame", "local")
		self.robot_frame = rospy.get_param("/robot_frame", "base_link")

		self.params = rospy.get_param("/tinyhelm_obstacles", {})
		if not self.params:
			rospy.logwarn("No parameters found under 'tinyhelm_obstacles'. Did you load the YAML file?")
			raise SystemExit(1)

		self.divergence_param = self.params.get('divergence_param')
		self.robot_width = self.params.get('robot_width')
		self.max_detour = self.params.get('max_lateral_detour')

		self.mission = []
		self.plan = []
		self.route = []
		self.pending_route = None
		self.published = []
		self.watched = []
		self.next_index = 0
		self.state = NO_MISSION
		self.walk = None
		self.request_id = 0
		self.request = PendingRequest(self.params.get('request_timeout'), self.params.get('request_retries'))
		self.last_status = None

		self.markers = DebugMarkers(self.planning_frame, self.max_detour)

		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

		self.path_pub = rospy.Publisher("/obstacles/_revised_path_out", Path, queue_size=1, latch=True)
		self.status_pub = rospy.Publisher("/obstacles/_status", MonitorStatus, queue_size=5, latch=True)
		self.remaining_pub = rospy.Publisher("/obstacles/_remaining", Path, queue_size=1, latch=True)

		self.request_pub = rospy.Publisher("/obstacles/plan_request", PlanRequest, queue_size=1)
		self.watch_pub = rospy.Publisher("/obstacles/path_watch", PathWatch, queue_size=1, latch=True)

		rospy.Subscriber("/obstacles/_mission_in", Path, self.mission_callback, queue_size=1)
		rospy.Subscriber("/obstacles/_current_path_in", Path, self.current_path_callback, queue_size=1)
		rospy.Subscriber("/obstacles/path_status", PathStatus, self.path_status_callback, queue_size=1)
		rospy.Subscriber("/obstacles/plan_reply", PlanReply, self.plan_reply_callback, queue_size=5)

		self.watchdog = rospy.Timer(rospy.Duration(0.2), self.check_timeout)

		self.publish_status(MonitorStatus.OK, "No active mission.")
		rospy.loginfo("obstacle monitor: corridor radius %.0fm, clearance follows %s", self.max_detour, self.divergence_param)

	def clearance(self):
		divergence = rospy.get_param(self.divergence_param, None)
		if divergence is None:
			rospy.logwarn_throttle(30.0, "obstacle monitor: %s is unset, falling back to robot_width" % self.divergence_param)
			return self.robot_width

		return divergence

	def position(self):
		return robot_position(self.tf_buffer, self.planning_frame, self.robot_frame)

	def active(self):
		return bool(self.mission) and self.next_index < len(self.mission) and len(self.plan) >= 2

	def geofence_polyline(self):
		return [(x, y) for x, y, _ in self.mission]

	def mission_callback(self, msg):
		poses = path_to_planning_frame(self.tf_buffer, msg, self.planning_frame)
		if poses is None:
			return

		self.abandon_walk()
		self.mission = poses_to_xyz(poses)
		self.next_index = 0

		self.adopt_route([(x, y, z, index) for index, (x, y, z) in enumerate(self.mission)])
		self.published = []

		if not self.mission:
			self.go_idle()
			return

		rospy.loginfo("obstacle monitor: mission of %d waypoints", len(self.mission))
		self.refresh()

		if self.active():
			self.state = CLEAR
			self.publish_status(MonitorStatus.OK, "Mission accepted, corridor clear so far.")
		else:
			self.go_idle()

	def current_path_callback(self, msg):
		poses = path_to_planning_frame(self.tf_buffer, msg, self.planning_frame)
		if poses is None:
			return

		self.plan = poses_to_xy(poses)

		self.watched = list(self.plan)
		self.publish_watch()

		if not self.active():
			self.abandon_walk()
			self.go_idle()
			return

		self.refresh()

	def go_idle(self):
		self.state = NO_MISSION
		self.watched = []
		self.publish_watch()
		self.markers.publish([], MonitorStatus.OK)
		self.publish_remaining()
		self.publish_status(MonitorStatus.OK, "No active mission.")

	def adopt_route(self, entries):
		"""Takes a route as immediately in force. Only the mission qualifies: it reaches the controller
		by the helm's own hand at the same moment it reaches us."""
		self.route = entries
		self.pending_route = None

	def resolve_cursor(self):
		if self.pending_route is not None:
			cursor = tail_cursor(self.pending_route, self.plan, ROUTE_MATCH)
			if cursor is not None:
				self.route = self.pending_route
				self.pending_route = None
				return cursor

		return tail_cursor(self.route, self.plan, ROUTE_MATCH)

	def next_original(self, cursor):
		for _, _, _, origin in self.route[cursor:]:
			if origin is not None:
				return origin

		return len(self.mission)

	def refresh(self):
		cursor = self.resolve_cursor()

		if cursor is None:
			if self.route and len(self.plan) >= 2:
				rospy.logwarn_throttle(10.0, "obstacle monitor: the controller's plan is not a tail of the route we published, leaving progress at %d" % self.next_index)
		else:
			index = self.next_original(cursor)
			if index != self.next_index:
				self.next_index = index
				rospy.loginfo("obstacle monitor: %d waypoints remaining", len(self.mission) - self.next_index)

		self.publish_remaining()
		self.markers.publish(self.geofence_polyline(), self.severity())

	def path_status_callback(self, msg):
		self.refresh()

		if not msg.blocked:
			if self.state in (CORRECTING, STUCK):
				rospy.loginfo("obstacle monitor: route clear again")
				self.abandon_walk()
			self.state = CLEAR
			self.publish_status(MonitorStatus.OK, "Route clear.")
			return

		if self.walk is not None or self.request.outstanding():
			return

		if not self.active():
			self.publish_status(MonitorStatus.SLOW, "Route obstructed but no mission to correct against.")
			return

		rospy.logwarn("obstacle monitor: route obstructed on leg %d, %.0fm ahead, clearance %.1fm", msg.blocked_leg, msg.blocked_distance, msg.min_clearance)
		self.begin_walk()

	def begin_walk(self):
		position = self.position()
		if position is None:
			return

		severity = MonitorStatus.HOLD if self.state == STUCK else MonitorStatus.SLOW

		self.walk = Correction(self.mission, self.next_index, position[0], position[1])
		self.state = CORRECTING
		self.publish_status(severity, "Route obstructed, planning a correction.")
		self.send_next()

	def abandon_walk(self):
		self.walk = None
		self.request.close()

	def send_next(self):
		pending = self.walk.pending() if self.walk else None
		if pending is None:
			self.finish_walk()
			return

		start, goal, index = pending
		self.request_id += 1
		self.request.open(self.request_id, index)
		self.publish_request(start, goal, index)

	def publish_request(self, start, goal, index):
		msg = PlanRequest()
		msg.request_id = self.request.request_id
		msg.start = Point(start[0], start[1], 0.0)
		msg.goal = Point(goal[0], goal[1], 0.0)
		msg.clearance = self.clearance()
		msg.corridor = [Point(x, y, 0.0) for x, y in self.geofence_polyline()]
		msg.corridor_radius = self.max_detour
		self.request_pub.publish(msg)

	def check_timeout(self, _):
		if not self.request.expired(rospy.Time.now()):
			return

		if self.request.exhausted():
			rospy.logerr("obstacle monitor: planner did not answer request %d after %d attempts, dropping the correction", self.request.request_id, self.request.attempts)
			self.abandon_walk()
			self.report_stuck("Planner is not answering.")
			return

		pending = self.walk.pending() if self.walk else None
		if pending is None:
			self.abandon_walk()
			return

		rospy.logwarn("obstacle monitor: request %d timed out, resending", self.request.request_id)
		self.request.retry()
		self.publish_request(pending[0], pending[1], pending[2])

	def plan_reply_callback(self, msg):
		if self.walk is None or not self.request.matches(msg.request_id):
			return

		index = self.request.mission_index
		self.request.close()

		if msg.result in (PlanReply.INTERNAL_ERROR, PlanReply.NO_COSTMAP):
			rospy.logwarn("obstacle monitor: planner could not answer request %d (%s)", msg.request_id, REASON_TEXT.get(msg.result, "unknown"))
			self.abandon_walk()
			self.report_stuck("Planner could not answer: %s." % REASON_TEXT.get(msg.result, "unknown"))
			return

		if msg.result == PlanReply.OK and msg.path:
			self.walk.accept([(p.x, p.y) for p in msg.path], index)
			self.send_next()
			return

		rospy.logwarn("obstacle monitor: waypoint %d unreachable (%s), routing to the next one", index, REASON_TEXT.get(msg.result, "unknown"))

		self.walk.skip(index)
		self.send_next()

	def finish_walk(self):
		walk = self.walk
		self.abandon_walk()

		if walk is None or len(walk.points) < 2:
			self.report_stuck("No usable corrected course through the remaining waypoints.")
			return

		self.publish_revision(walk)

	def report_stuck(self, message):
		self.state = STUCK
		self.publish_status(MonitorStatus.HOLD, message)

	def same_as_published(self, points):
		if len(points) != len(self.published):
			return False

		return all(math.hypot(a[0] - b[0], a[1] - b[1]) <= SAME_CORRECTION for a, b in zip(points, self.published))

	def publish_revision(self, walk):
		if self.same_as_published(walk.points):
			rospy.loginfo_throttle(5.0, "obstacle monitor: correction unchanged, leaving the route in force")
			self.state = CORRECTING
			self.publish_status(MonitorStatus.OK, "Correction already in force.")
			return

		self.published = list(walk.points)
		self.path_pub.publish(make_path(self.planning_frame, walk.points))

		self.pending_route = walk.entries()

		self.watched = [(x, y) for x, y, _ in walk.points]
		self.publish_watch()

		self.state = CORRECTING

		if walk.skipped:
			self.publish_status(MonitorStatus.REPLAN, "Corrected course around obstacles, skipping %s." % ", ".join("waypoint %d" % i for i in walk.skipped))
		else:
			self.publish_status(MonitorStatus.REPLAN, "Corrected course planned around obstacles.")

		rospy.loginfo("obstacle monitor: published a correction of %d poses", len(walk.points))

	def publish_remaining(self):
		self.remaining_pub.publish(make_path(self.planning_frame, self.mission[self.next_index:]))

	def publish_watch(self):
		msg = PathWatch()
		msg.path = [Point(x, y, 0.0) for x, y in self.watched]
		msg.clearance = self.clearance()
		self.watch_pub.publish(msg)

	def severity(self):
		return self.last_status[0] if self.last_status else MonitorStatus.OK

	def publish_status(self, status, message):

		if (status, message) == self.last_status:
			return

		changed = self.last_status is None or status != self.last_status[0]
		self.last_status = (status, message)

		msg = MonitorStatus()
		msg.status = status
		msg.message = message
		self.status_pub.publish(msg)

		if changed:
			self.markers.recolour(status)

if __name__ == "__main__":
	rospy.init_node("tinyhelm_obstacles")
	ObstacleMonitorNode()
	rospy.spin()