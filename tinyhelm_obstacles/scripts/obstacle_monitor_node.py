#!/usr/bin/env python3
import rospy
import tf2_ros

from geometry_msgs.msg import Point
from nav_msgs.msg import Path

from markers import DebugMarkers
from tinyhelm_core.msg import MonitorStatus
from tinyhelm_obstacles.msg import PathStatus, PathWatch, PlanReply, PlanRequest
from utils import PendingRequest, make_path, match_index, path_to_planning_frame, poses_to_xy, poses_to_xyz, robot_position

# How close a plan pose has to be to a mission waypoint to be the same waypoint. Corrections end
# exactly on mission waypoints, so this only has to absorb float round trips through messages.
WAYPOINT_MATCH = 0.01

NO_MISSION = "no_mission"
CLEAR = "clear"
CORRECTING = "correcting"
STUCK = "stuck"

class Correction:
	"""One correction being assembled, a leg at a time. Appends and advances; it decides nothing
	about what gets published."""

	def __init__(self, mission, first_index, rx, ry):
		self.mission = mission
		self.targets = list(range(first_index, len(mission)))
		self.cursor = 0
		self.from_x = rx
		self.from_y = ry
		self.points = []
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

		# Ends the leg on the waypoint itself rather than the nearest cell centre, so quantisation
		# cannot drift the route off the survey line, and so the next progress match still finds it
		if self.points:
			self.points[-1] = self.mission[index]

		self.from_x, self.from_y = self.mission[index][0], self.mission[index][1]
		self.cursor += 1

	def skip(self, index):
		# The vessel's current position stays the anchor for the next leg, so the correction runs from
		# where we are toward the next waypoint we can still reach
		self.skipped.append(index)
		self.cursor += 1

class ObstacleMonitorNode:
	"""Watches the route the active controller is following and offers a corrected one when it is
	obstructed.

	Two inputs with two distinct jobs. The mission is authoritative and is never overwritten by
	anything published from here: every corridor, every geofence and every leg reference comes off it
	alone, so a correction cannot walk the geofence off the survey line. The controller's plan is read
	only to work out how much of the mission is already done, by matching its poses against mission
	waypoints, which is the controller's own answer rather than an estimate of it."""

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
		self.retry_seconds = self.params.get('retry_unreachable_waypoint_seconds')

		self.mission = []
		self.plan = []
		self.watched = []
		self.next_index = 0
		self.state = NO_MISSION
		self.walk = None
		self.request_id = 0
		self.request = PendingRequest(self.params.get('request_timeout'), self.params.get('request_retries'))
		self.first_failed_at = {}
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
		"""How far the route must stay from an obstacle. This is the controller's allowed divergence
		from its line, because the vessel may sit anywhere within that band under wind or current and
		the corridor has to be clear wherever it ends up. The controller holds its hull inside the same
		band by giving up half a robot width of it, so the margin is accounted for once and there."""
		divergence = rospy.get_param(self.divergence_param, None)
		if divergence is None:
			rospy.logwarn_throttle(30.0, "obstacle monitor: %s is unset, falling back to robot_width" % self.divergence_param)
			return self.robot_width

		return divergence

	def position(self):
		return robot_position(self.tf_buffer, self.planning_frame, self.robot_frame)

	def active(self):
		return bool(self.mission) and self.next_index < len(self.mission) and len(self.plan) >= 2

	def corridor_polyline(self):
		"""The mission legs still to run, from the one being followed. Off the mission and never off
		the plan, so it stays where the survey line is however far a correction pushes us."""
		return [(x, y) for x, y, _ in self.mission[max(0, self.next_index - 1):]]

	def leg_reference(self, index, ax, ay):
		"""The one mission leg a search is asked to solve. Per leg rather than the whole remainder
		because a survey pattern folds back on itself, and a tube round the full polyline merges
		adjacent rows into one blob."""
		if index <= 0:
			return [(ax, ay), (self.mission[0][0], self.mission[0][1])]

		return [(self.mission[index - 1][0], self.mission[index - 1][1]), (self.mission[index][0], self.mission[index][1])]

	def mission_callback(self, msg):
		poses = path_to_planning_frame(self.tf_buffer, msg, self.planning_frame)
		if poses is None:
			return

		self.abandon_walk()
		self.mission = poses_to_xyz(poses)
		self.next_index = 0
		self.first_failed_at = {}

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

		# What the planner watches for obstruction, which is the plan until we publish a correction and
		# then the correction, because the controller's echo of it is a full round trip away
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
		self.markers.publish([], None)
		self.publish_remaining()
		self.publish_status(MonitorStatus.OK, "No active mission.")

	def refresh(self):
		"""Recomputes progress from scratch against the controller's plan. No ratchet and nothing
		remembered, so there is nothing here to drift."""
		index = match_index(self.mission, self.plan, WAYPOINT_MATCH)

		if index is None:
			if self.mission and len(self.plan) >= 2:
				rospy.logwarn_throttle(10.0, "obstacle monitor: plan carries none of the mission's waypoints, leaving progress at %d" % self.next_index)
		elif index != self.next_index:
			self.next_index = index
			rospy.loginfo("obstacle monitor: %d waypoints remaining", len(self.mission) - self.next_index)

		self.publish_remaining()
		self.markers.publish(self.corridor_polyline(), self.position())

	def path_status_callback(self, msg):
		"""The planner reports on the route rather than the monitor inspecting the map, so this is the
		only place an intrusion is learned about."""
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

		# HOLD only when there is no usable correction on the wire. A first attempt against a fresh
		# obstruction is usually answered in a fraction of a second, so easing off is enough.
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
		msg.corridor = [Point(x, y, 0.0) for x, y in self.leg_reference(index, start[0], start[1])]
		msg.corridor_radius = self.max_detour
		self.request_pub.publish(msg)

	def check_timeout(self, _):
		"""A planner that has died and one still thinking look identical from here, so an outstanding
		request is given a bounded number of attempts and then the correction is dropped rather than
		waiting on it forever."""
		if not self.request.expired(rospy.Time.now()):
			return

		if self.request.exhausted():
			rospy.logerr("obstacle monitor: planner did not answer request %d after %d attempts, dropping the correction", self.request.request_id, self.request.attempts)
			self.abandon_walk()
			# Never an ESTOP. That ends the mission and can leave the vessel somewhere it has to be
			# fetched from, which is far worse than waiting: the walk is dropped, the next steady report
			# starts a fresh one, and it keeps asking for as long as it takes.
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
			# Not an answer about the waypoint, so it must not count against one. Dropping the
			# correction rather than retrying is deliberate: the next steady report comes in a second
			# and starts a fresh attempt, where a retry would go straight back into the same fault.
			rospy.logwarn("obstacle monitor: planner could not answer request %d (%s)", msg.request_id, self.reason_text(msg.result))
			self.abandon_walk()
			self.report_stuck("Planner could not answer: %s." % self.reason_text(msg.result))
			return

		if msg.result == PlanReply.OK and len(msg.path) >= 2:
			self.first_failed_at.pop(index, None)
			self.walk.accept([(p.x, p.y) for p in msg.path], index)
			self.send_next()
			return

		reason = self.reason_text(msg.result)
		if not self.give_up_on(index):
			self.withhold(index, reason)
			return

		rospy.logwarn("obstacle monitor: waypoint %d unreachable for %.0fs (%s), routing to the next one", index, self.retry_seconds, reason)
		self.walk.skip(index)
		self.send_next()

	def give_up_on(self, index):
		"""Whether this waypoint has been coming back unreachable for long enough to route around it.

		Wall clock rather than an attempt count, because what usually resolves it is costmap decay
		letting go of evidence, and that happens on its own schedule. The clock is per waypoint and
		starts at its first failure; one successful plan to it clears the clock entirely, so a
		waypoint that fails again later gets a full fresh window."""
		first = self.first_failed_at.setdefault(index, rospy.Time.now())
		return (rospy.Time.now() - first).to_sec() >= self.retry_seconds

	def withhold(self, index, reason):
		"""Nothing goes out on the wire while we are still confirming. A correction that has quietly
		dropped a waypoint is worse than steering the old line slowly, and with nothing publishable
		there is no corrected course to ease off along, so this is a hold."""
		self.abandon_walk()
		self.state = STUCK
		self.publish_status(MonitorStatus.HOLD, "Waypoint %d unreachable (%s), retrying." % (index, reason))

	def reason_text(self, result):
		return {
			PlanReply.GOAL_IN_OBSTACLE: "waypoint is inside an obstacle",
			PlanReply.GOAL_OUTSIDE_CORRIDOR: "waypoint lies outside its own corridor",
			PlanReply.START_TRAPPED: "no clear water around the vessel to start from",
			PlanReply.NO_ROUTE: "no route within the corridor",
			PlanReply.NO_COSTMAP: "no costmap yet",
			PlanReply.INTERNAL_ERROR: "the planner failed internally",
		}.get(result, "unknown")

	def finish_walk(self):
		walk = self.walk
		self.abandon_walk()

		# A correction ending at the last waypoint we could reach is still a safe course to steer.
		# Only one too short to follow at all is worth nothing.
		if walk is None or len(walk.points) < 2:
			self.report_stuck("No usable corrected course through the remaining waypoints.")
			return

		self.publish_revision(walk)

	def report_stuck(self, message):
		"""We have nothing to offer. Reported as an observation and not as a command: HOLD says the
		route ahead is obstructed and there is no way round it yet, which is for the helm to act on.

		Deliberately open ended. A hold is held for as long as the water stays shut, because the map is
		evidence that decays and an obstruction that is total now may not be in a minute."""
		self.state = STUCK
		self.publish_status(MonitorStatus.HOLD, message)

	def publish_revision(self, walk):
		self.path_pub.publish(make_path(self.planning_frame, walk.points))

		# Watch the correction from here rather than the route it replaces. The controller echoes its
		# own version back once the helm has relayed it, but that is a full round trip away, and until
		# then the planner would keep reporting the superseded route obstructed.
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

	def publish_status(self, status, message):
		# The planner reports steadily rather than only on change, which is what keeps progress and the
		# corridor overlay current, but it would otherwise have us restating an unchanged status to the
		# helm every second. The topic is latched, so a late subscriber still gets the current state.
		if (status, message) == self.last_status:
			return

		self.last_status = (status, message)

		msg = MonitorStatus()
		msg.status = status
		msg.message = message
		self.status_pub.publish(msg)

if __name__ == "__main__":
	rospy.init_node("tinyhelm_obstacles")
	ObstacleMonitorNode()
	rospy.spin()
