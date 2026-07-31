#!/usr/bin/env python3
import math
import threading
import time
import traceback

import numpy as np
import rospy

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid

from coarse_heuristic import CoarseHeuristic
from cost_field import Capsule, CostField, corridor_from_polyline, decode_distance, nudge_goal
from theta_star import GOAL_IN_OBSTACLE, GOAL_OUTSIDE_CORRIDOR, NO_ROUTE, OK, START_TRAPPED, UNREACHABLE_COARSE, ThetaStar, smooth_path
from tinyhelm_obstacles.msg import PathStatus, PathWatch, PlanReply, PlanRequest

RESULT_CODES = {
	OK: PlanReply.OK,
	GOAL_IN_OBSTACLE: PlanReply.GOAL_IN_OBSTACLE,
	GOAL_OUTSIDE_CORRIDOR: PlanReply.GOAL_OUTSIDE_CORRIDOR,
	START_TRAPPED: PlanReply.START_TRAPPED,
	NO_ROUTE: PlanReply.NO_ROUTE,
	# Same answer on the wire: the caller can do nothing different about it, and the distinction is
	# only worth having in the log line, where it names which of the two happened
	UNREACHABLE_COARSE: PlanReply.NO_ROUTE,
}

class PlannerNode:
	"""Answers one question: how do I get from here to there without hitting anything and without
	leaving the corridor. It holds no mission, no progress and no policy, so there is nothing here to
	go stale and nothing to reset.

	A search takes long enough that several costmap updates will land during one. Rather than lock the
	map for the duration, each request takes a snapshot of the latest field and plans against that,
	which is both the cheapest and the most honest answer to "always use the newest data": a reply is
	explicitly an answer about the map as it stood when the request arrived.

	It also re-examines the route the monitor asked it to watch on every costmap update. That check
	needs the distance field and nothing else, so doing it here keeps the monitor free of the grid
	entirely."""

	def __init__(self):
		self.planning_frame = rospy.get_param("/planning_frame", "local")

		self.params = rospy.get_param("/tinyhelm_obstacles", {})
		if not self.params:
			rospy.logwarn("No parameters found under 'tinyhelm_obstacles'. Did you load the YAML file?")
			raise SystemExit(1)

		self.soft_radius = self.params.get('soft_radius')
		self.coarse_factor = self.params.get('coarse_factor')
		self.status_period = self.params.get('status_period')

		self.lock = threading.Lock()
		self.dist = None
		self.res = None
		self.origin = (0.0, 0.0)

		self.watched = []
		self.watch_clearance = 0.0
		self.warned_clearance = False
		self.last_blocked = None
		self.last_status = rospy.Time(0)

		self.search = ThetaStar()

		self.reply_pub = rospy.Publisher("/obstacles/plan_reply", PlanReply, queue_size=5)
		self.status_pub = rospy.Publisher("/obstacles/path_status", PathStatus, queue_size=5, latch=True)

		rospy.Subscriber("/obstacles/costmap", OccupancyGrid, self.costmap_callback, queue_size=1)
		rospy.Subscriber("/obstacles/plan_request", PlanRequest, self.request_callback, queue_size=1)
		rospy.Subscriber("/obstacles/path_watch", PathWatch, self.watch_callback, queue_size=1)

		rospy.loginfo("planner: waiting for a costmap on /obstacles/costmap")

	def costmap_callback(self, msg):
		try:
			self.adopt_costmap(msg)
		except Exception:
			rospy.logerr("planner: costmap rejected:\n%s", traceback.format_exc())

	def adopt_costmap(self, msg):
		values = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

		# The costmap publishes the distance field with no hard inflation baked in, so clearance is
		# applied here and can follow the controller's reconfigurable divergence band
		dist = decode_distance(values, 0.0, self.soft_radius)

		with self.lock:
			self.dist = dist
			self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
			self.res = msg.info.resolution

		self.review_watched()

	def watch_callback(self, msg):
		try:
			self.adopt_watch(msg)
		except Exception:
			rospy.logerr("planner: watch rejected:\n%s", traceback.format_exc())

	def adopt_watch(self, msg):
		with self.lock:
			self.watched = [(p.x, p.y) for p in msg.path]
			self.watch_clearance = msg.clearance
			self.last_blocked = None

		self.review_watched()

	def field_for(self, clearance, corridor, centreline=None, divergence_radius=0.0):
		with self.lock:
			if self.dist is None:
				return None
			dist = self.dist
			origin = self.origin
			res = self.res

		field = CostField()
		field.adopt(res, origin[0], origin[1], dist, clearance, self.soft_radius, corridor, centreline, divergence_radius)
		return field

	def review_watched(self):
		"""Walks the watched route and reports the first place it is obstructed. Sampling at half the
		grid resolution so a route cannot thread between samples past a cell it actually clips."""
		with self.lock:
			route = list(self.watched)
			clearance = self.watch_clearance
			if self.dist is not None:
				# Store local references to avoid lock overhead in loop
				dist_grid = self.dist
				grid_rows, grid_cols = dist_grid.shape
				origin_x, origin_y = self.origin
				res = self.res
			else:
				dist_grid = None

		if len(route) < 2:
			return

		if clearance > self.soft_radius and not self.warned_clearance:
			self.warned_clearance = True
			rospy.logwarn("planner: clearance %.1fm exceeds soft_radius %.1fm, obstructions beyond the soft radius cannot be seen", clearance, self.soft_radius)

		field = self.field_for(clearance, None)
		if field is None:
			return

		step = 0.5 * res
		travelled = 0.0
		worst = float("inf")
		blocked_leg = -1
		blocked_at = 0.0

		# Pre-calculate clearance padding in grid cells for the bounding box check
		pad_cells = int(clearance / res) + 1 if res > 0 else 0

		for leg in range(1, len(route)):
			ax, ay = route[leg - 1]
			bx, by = route[leg]
			length = ((bx - ax) ** 2 + (by - ay) ** 2) ** 0.5
			
			# Broad-Phase Bounding Box Check 
			needs_detailed_sampling = True
			
			if dist_grid is not None:
				# Find bounding box in world coordinates
				min_x, max_x = min(ax, bx), max(ax, bx)
				min_y, max_y = min(ay, by), max(ay, by)
				
				# Convert to grid coordinates with clearance padding
				min_col = int((min_x - origin_x) / res) - pad_cells
				max_col = int((max_x - origin_x) / res) + pad_cells + 1
				min_row = int((min_y - origin_y) / res) - pad_cells
				max_row = int((max_y - origin_y) / res) + pad_cells + 1
				
				# A) Outside costmap check: if the padded box is completely outside the grid,
				# we can skip detailed sampling because unseen space is clear.
				if max_col < 0 or min_col >= grid_cols or max_row < 0 or min_row >= grid_rows:
					# Entirely outside the known costmap: unseen space reads as infinitely clear,
					# capped at the soft radius so the reported worst clearance still means something.
					needs_detailed_sampling = False
					worst = min(worst, self.soft_radius)
				else:
					# B) Clear costmap check: clamp to grid bounds and check the subarray
					min_col_c = max(0, min_col)
					max_col_c = min(grid_cols, max_col)
					min_row_c = max(0, min_row)
					max_row_c = min(grid_rows, max_row)
					
					sub_grid = dist_grid[min_row_c:max_row_c, min_col_c:max_col_c]
					
					# If the minimum distance in this bounding box is strictly greater than clearance, the entire line segment is guaranteed safe. The box minimum is still a valid bound on the worst clearance actually seen along the segment.
					if sub_grid.size > 0 and np.min(sub_grid) >= clearance:
						needs_detailed_sampling = False
						worst = min(worst, np.min(sub_grid), self.soft_radius)

			# Narrow-Phase Point-by-Point Check
			if needs_detailed_sampling:
				samples = max(1, int(length / step))
				for s in range(samples + 1):
					t = s / samples
					gap = min(field.obstacle_distance_at(ax + t * (bx - ax), ay + t * (by - ay)), self.soft_radius)
					worst = min(worst, gap)
					if blocked_leg < 0 and gap < clearance:
						blocked_leg = leg
						blocked_at = travelled + t * length
			
			travelled += length

		self.report(blocked_leg >= 0, blocked_leg, blocked_at, worst)
	

	def report(self, blocked, leg, distance, clearance):
		# Reported on every change and otherwise at status_period, clear or not. The steady report is
		# what the monitor advances its progress and redraws its corridor on, and it doubles as a
		# heartbeat: a monitor that came up late still learns the state without the whole thing being
		# restated at grid rate.
		now = rospy.Time.now()
		changed = blocked != self.last_blocked
		if not changed and (now - self.last_status).to_sec() < self.status_period:
			return

		self.last_blocked = blocked
		self.last_status = now

		msg = PathStatus()
		msg.blocked = blocked
		msg.blocked_leg = leg
		msg.blocked_distance = distance
		msg.min_clearance = clearance
		self.status_pub.publish(msg)

	def request_callback(self, msg):
		"""Anything thrown in here would otherwise reach the monitor as silence, which it can only
		resolve by waiting out a timeout and retrying the same request into the same fault. An answer
		it can act on immediately is worth far more than a clean stack trace, so a failure is reported
		as one."""
		try:
			self.solve(msg)
		except Exception:
			rospy.logerr("planner: request %d failed:\n%s", msg.request_id, traceback.format_exc())
			self.publish_reply(msg.request_id, PlanReply.INTERNAL_ERROR, [], False, 0.0, 0)

	def corridor_for(self, centreline, msg):
		if len(centreline) < 2:
			return None

		corridor = corridor_from_polyline(centreline, msg.corridor_radius)

		# The tube stays anchored to the mission, so a vessel pushed off its line can end up outside it
		# with nowhere legal to begin a search. A disc at the start restores somewhere to start from
		# without letting the tube itself follow the vessel around.
		corridor.append(Capsule(msg.start.x, msg.start.y, msg.start.x, msg.start.y, msg.corridor_radius))
		return corridor

	def solve(self, msg):
		centreline = [(p.x, p.y) for p in msg.corridor]
		corridor = self.corridor_for(centreline, msg)

		field = self.field_for(msg.clearance, corridor, centreline, msg.corridor_radius)
		if field is None:
			rospy.logwarn_throttle(5.0, "planner: request %d arrived before any costmap" % msg.request_id)
			self.publish_reply(msg.request_id, PlanReply.NO_COSTMAP, [], False, 0.0, 0)
			return

		started = time.time()

		# Before the heuristic rather than inside the search, so the coarse layer is built against the
		# goal actually being aimed at. Half the corridor keeps the moved waypoint inside the tube it
		# was allowed, and the reply carries it as the last pose of the path.
		goal_x, goal_y = msg.goal.x, msg.goal.y
		nudged = nudge_goal(field, goal_x, goal_y, msg.start.x, msg.start.y, 0.5 * msg.corridor_radius)
		if nudged:
			rospy.loginfo("planner: request %d goal was blocked, moved %.1fm to clear water", msg.request_id, math.hypot(nudged[0] - goal_x, nudged[1] - goal_y))
			goal_x, goal_y = nudged

		heuristic = CoarseHeuristic(field, goal_x, goal_y, factor=self.coarse_factor)
		raw = self.search.plan(field, msg.start.x, msg.start.y, goal_x, goal_y, msg.corridor_radius, heuristic=heuristic)
		path = smooth_path(field, raw, 2.0 * field.res) if raw else []
		elapsed = time.time() - started

		code = RESULT_CODES.get(self.search.reason, PlanReply.NO_ROUTE)
		if path:
			rospy.loginfo("planner: request %d solved in %.0fms, %d expansions, %d poses", msg.request_id, elapsed * 1000.0, self.search.last_expansions, len(path))
		else:
			rospy.logwarn("planner: request %d unsolved (%s) in %.0fms, %d expansions", msg.request_id, self.search.reason, elapsed * 1000.0, self.search.last_expansions)

		self.publish_reply(msg.request_id, code, path, self.search.start_nudged, elapsed, self.search.last_expansions)

	def publish_reply(self, request_id, code, path, nudged, elapsed, expansions):
		msg = PlanReply()
		msg.request_id = request_id
		msg.result = code
		msg.start_nudged = nudged
		msg.plan_seconds = elapsed
		msg.expansions = expansions
		msg.path = [Point(x, y, 0.0) for x, y in path]
		self.reply_pub.publish(msg)

if __name__ == "__main__":
	rospy.init_node("tinyhelm_planner")
	PlannerNode()
	rospy.spin()