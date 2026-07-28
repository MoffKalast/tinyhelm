#!/usr/bin/env python3
import rospy
import tf2_geometry_msgs
import tf2_ros

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path

from mission_state import WALK_ABANDONED, WALK_RUNNING, WALK_WITHHELD, MissionState, ReplanWalk, drop_passed_legs
from tinyhelm_core.msg import MonitorStatus
from tinyhelm_obstacles.msg import PathStatus, PathWatch, PlanReply, PlanRequest

from markers import DebugMarkers

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
		self.request_timeout = self.params.get('request_timeout')
		self.request_retries = self.params.get('request_retries')

		self.state = MissionState(self.params.get('unreachable_cycles'))

		self.current_path = []
		self.walk = None
		self.request_id = 0
		self.awaiting = None
		self.awaiting_since = rospy.Time(0)
		self.attempts = 0
		self.blocked = False

		# Set once a search has failed against the obstruction we are currently looking at, and only
		# cleared when the route comes good or a correction goes out. Without it a monitor that retries
		# once a second would report SLOW on every fresh attempt and HOLD on every failure, and the
		# helm would dutifully let go and take hold again all the way to the obstacle.
		self.stuck = False
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

		self.watchdog = rospy.Timer(rospy.Duration(self.params.get('planner_timeout')), self.check_timeout)

		self.publish_status(MonitorStatus.OK, "No active mission.")
		rospy.loginfo("obstacle monitor: corridor radius %.0fm, clearance follows %s", self.max_detour, self.divergence_param)

	def clearance(self):
		"""How far the route must stay from an obstacle. This is the controller's allowed divergence
		from its line, because the vessel may sit anywhere within that band under wind or current and
		the corridor has to be clear wherever it ends up. The controller holds its hull inside the same
		band by giving up half a robot width of it, so the margin is accounted for once and here."""
		divergence = rospy.get_param(self.divergence_param, None)
		if divergence is None:
			rospy.logwarn_throttle(30.0, "obstacle monitor: %s is unset, falling back to robot_width" % self.robot_width)
			return self.robot_width

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
		self.stuck = False

		if not self.state.active():
			self.current_path = []
			self.publish_watch()
			self.markers.publish([], None)
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
		self.markers.publish(self.state.corridor_polyline(), position)

	def path_status_callback(self, msg):
		"""The planner reports on the route rather than the monitor inspecting the map, so this is the
		only place an intrusion is learned about."""
		self.refresh_progress()

		if not msg.blocked:
			if self.blocked:
				rospy.loginfo("obstacle monitor: route clear again")
				self.abandon_walk()
			self.blocked = False
			self.stuck = False
			self.publish_status(MonitorStatus.OK, "Route clear.")
			return

		self.blocked = True
		if self.walk is not None or self.awaiting is not None:
			return

		if not self.state.active():
			# Nothing to correct against and no leg of ours to hold off, but something is obstructed
			self.publish_status(MonitorStatus.SLOW, "Route obstructed but no mission to correct against.")
			return

		rospy.logwarn("obstacle monitor: route obstructed on leg %d, %.0fm ahead, clearance %.1fm", msg.blocked_leg, msg.blocked_distance, msg.min_clearance)
		self.begin_walk()

	def begin_walk(self):
		position = self.robot_position()
		if position is None:
			return

		self.walk = ReplanWalk(self.state, position[0], position[1])
		# A first attempt against a fresh obstruction is usually answered in a fraction of a second, so
		# easing off is enough; it escalates only once one of them has come back empty
		self.publish_status(MonitorStatus.HOLD if self.stuck else MonitorStatus.SLOW, "Route obstructed, planning a correction.")
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

		self.request_id += 1
		self.awaiting = self.request_id
		self.awaiting_since = rospy.Time.now()
		self.attempts = 1
		self.publish_request(start, goal, index)

	def publish_request(self, start, goal, index):
		msg = PlanRequest()
		msg.request_id = self.awaiting
		msg.start = Point(start[0], start[1], 0.0)
		msg.goal = Point(goal[0], goal[1], 0.0)
		msg.clearance = self.clearance()
		# Only the leg being solved, not the whole remaining mission. The search is held to a tube round
		# this and is steered toward the line itself, and both go wrong on a pattern that doubles back.
		msg.corridor = [Point(x, y, 0.0) for x, y in self.state.leg_reference(index, start[0], start[1])]
		msg.corridor_radius = self.max_detour
		self.request_pub.publish(msg)

	def resend(self):
		pending = self.walk.pending_request() if self.walk else None
		if pending is None:
			self.abandon_walk()
			return

		start, goal, index = pending
		self.attempts += 1
		self.awaiting_since = rospy.Time.now()
		self.publish_request(start, goal, index)

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
			# Never an ESTOP. That ends the mission and can leave the vessel somewhere it has to be
			# fetched from, which is a far worse outcome than waiting: the walk is dropped, the next
			# steady report starts a fresh one, and it keeps asking for as long as it takes.
			self.report_stuck("Planner is not answering.")
			return

		rospy.logwarn("obstacle monitor: request %d timed out, resending", self.awaiting)
		self.resend()

	def plan_reply_callback(self, msg):
		if self.walk is None or self.awaiting is None or msg.request_id != self.awaiting:
			return

		self.awaiting = None
		index = self.walk.target_index()

		if msg.result in (PlanReply.INTERNAL_ERROR, PlanReply.NO_COSTMAP):
			# Not an answer about the waypoint, so it must not count toward giving up on one. Dropping
			# the correction here rather than retrying is deliberate: the next steady report comes in a
			# second and will start a fresh attempt, where a retry would go straight back into the same
			# fault and burn the timeout budget doing it.
			rospy.logwarn("obstacle monitor: planner could not answer request %d (%s)", msg.request_id, self.reason_text(msg.result))
			self.abandon_walk()
			self.report_stuck("Planner could not answer: %s." % self.reason_text(msg.result))
			return

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
			PlanReply.INTERNAL_ERROR: "the planner failed internally",
		}.get(result, "unknown")

	def finish_walk(self, outcome, index=None, result=None):
		walk = self.walk
		self.abandon_walk()

		if outcome == WALK_WITHHELD:
			# Withholding says nothing about whether there is anywhere to go. The walk confirms one
			# waypoint over several attempts and then moves on to the next, so on a long mission it can
			# withhold its way through every remaining waypoint in turn, for minutes, without ever once
			# concluding that the correction is hopeless. What decides the pace is whether it has
			# assembled a course the vessel could actually steer: with a usable prefix there is
			# somewhere safe to be going, so ease off and keep confirming; with nothing at all there is
			# no route out of here and the only safe speed is zero.
			if walk is not None and len(walk.points) >= 2:
				self.publish_status(MonitorStatus.SLOW, "Waypoint %s unreachable (%s), confirming before correcting." % (index, self.reason_text(result)))
			else:
				self.report_stuck("Waypoint %s unreachable (%s), nothing usable to steer yet." % (index, self.reason_text(result)))
			return

		if outcome == WALK_ABANDONED or walk is None or len(walk.points) < 2:
			self.report_stuck("No usable corrected course through the remaining waypoints.")
			return

		self.publish_revision(walk)

	def report_stuck(self, message):
		"""We have nothing to offer. Reported as an observation and not as a command: HOLD says the
		route ahead is obstructed and there is no way round it yet, which is for the helm to act on,
		and every later attempt against the same obstruction says the same until one of them lands.

		Deliberately open ended. A hold is held for as long as the water stays shut, because the map is
		evidence that decays and an obstruction that is total now may not be in a minute; there is no
		attempt count after which this gives up and escalates."""
		self.stuck = True
		self.publish_status(MonitorStatus.HOLD if self.blocked else MonitorStatus.SLOW, message)

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
		self.stuck = False

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
		# The planner now reports steadily rather than only on change, which is what keeps progress and
		# the corridor overlay current, but it would otherwise have us restating an unchanged status to
		# the helm every second. The topic is latched, so a late subscriber still gets the current
		# state, and the helm sees a clean edge when one actually happens.
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