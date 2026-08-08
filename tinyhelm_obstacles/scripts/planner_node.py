#!/usr/bin/env python3
import math
import threading
import time
import traceback

import numpy as np
import rospy

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid

from cost_field import CostField, corridor_from_polyline, decode_distance, distance_quantum, nudge_goal
from theta_star import GOAL_IN_OBSTACLE, GOAL_OUTSIDE_CORRIDOR, NO_ROUTE, OK, START_TRAPPED, UNREACHABLE_COARSE, ThetaStar, smooth_path
from tinyhelm_obstacles.msg import PathStatus, PathWatch, PlanReply, PlanRequest
from utils import segment_bounds_hit_box, segment_hits_box

ROUTE_SAMPLE_CELLS = 0.5

RESULT_CODES = {
	OK: PlanReply.OK,
	GOAL_IN_OBSTACLE: PlanReply.GOAL_IN_OBSTACLE,
	GOAL_OUTSIDE_CORRIDOR: PlanReply.GOAL_OUTSIDE_CORRIDOR,
	START_TRAPPED: PlanReply.START_TRAPPED,
	NO_ROUTE: PlanReply.NO_ROUTE,
	UNREACHABLE_COARSE: PlanReply.NO_ROUTE,
}

class PlannerNode:

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

		self.extent = None
		self.extent_key = None
		self.corridor = None
		self.corridor_key = None
		self.corridor_flags = None
		self.watch_flags = None

		self.watched = []
		self.watch_clearance = 0.0
		self.warned_clearance = False
		self.last_blocked = None
		self.last_status = rospy.Time(0)

		self.search = ThetaStar(coarse_factor=self.coarse_factor)

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
		dist = decode_distance(values, self.soft_radius)

		with self.lock:
			self.dist = dist
			self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
			self.res = msg.info.resolution
			self.refresh_extent(dist.shape[0])

		self.review_watched()

	def refresh_extent(self, size):
		key = (self.origin[0], self.origin[1], self.res, size)
		if key == self.extent_key:
			return

		self.extent_key = key
		self.extent = (self.origin[0], self.origin[1], self.origin[0] + size * self.res, self.origin[1] + size * self.res)
		self.corridor_flags = None
		self.watch_flags = None

	def mark_capsules(self, corridor):
		min_x, min_y, max_x, max_y = self.extent
		return [segment_hits_box(c.x1, c.y1, c.x2, c.y2, min_x - c.radius, min_y - c.radius, max_x + c.radius, max_y + c.radius) for c in corridor]

	def mark_legs(self, route, clearance):
		pad = (int(clearance / self.res) + 3) * self.res
		min_x, min_y, max_x, max_y = self.extent
		return [segment_bounds_hit_box(route[i - 1][0], route[i - 1][1], route[i][0], route[i][1], min_x - pad, min_y - pad, max_x + pad, max_y + pad) for i in range(1, len(route))]

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
			self.watch_flags = None

		self.review_watched()

	def field_for(self, clearance, corridor, active=None, centreline=None, divergence_radius=0.0):
		with self.lock:
			if self.dist is None:
				return None
			dist = self.dist
			origin = self.origin
			res = self.res
			extent = self.extent

		field = CostField()
		field.adopt(res, origin[0], origin[1], dist, clearance, self.soft_radius, corridor, active, centreline, divergence_radius)
		return field, extent

	def review_watched(self):
		with self.lock:
			route = list(self.watched)
			clearance = self.watch_clearance
			if len(route) >= 2 and self.extent is not None and self.watch_flags is None:
				self.watch_flags = self.mark_legs(route, clearance)
			marks = self.watch_flags

		if len(route) < 2:
			return

		if clearance > self.soft_radius and not self.warned_clearance:
			self.warned_clearance = True
			rospy.logwarn("planner: clearance %.1fm exceeds soft_radius %.1fm, obstructions beyond the soft radius cannot be seen", clearance, self.soft_radius)

		snapshot = self.field_for(clearance, None)
		if snapshot is None:
			return

		field = snapshot[0]

		step = ROUTE_SAMPLE_CELLS * field.res
		tolerance = distance_quantum(clearance, self.soft_radius)
		pad_cells = int(clearance / field.res) + 1

		travelled = 0.0
		worst = float("inf")
		blocked_leg = -1
		blocked_at = 0.0

		for leg in range(1, len(route)):
			ax, ay = route[leg - 1]
			bx, by = route[leg]
			length = math.hypot(bx - ax, by - ay)

			if marks is not None and not marks[leg - 1]:
				bound = float("inf")
			else:
				bound = self.leg_clearance_bound(field, ax, ay, bx, by, pad_cells)

			if bound is not None and bound >= clearance:
				worst = min(worst, bound, self.soft_radius)
			else:
				samples = max(1, int(length / step))
				for s in range(samples + 1):
					t = s / samples
					gap = min(field.obstacle_distance_at(ax + t * (bx - ax), ay + t * (by - ay)), self.soft_radius)
					worst = min(worst, gap)
					if blocked_leg < 0 and gap < clearance - tolerance:
						blocked_leg = leg
						blocked_at = travelled + t * length

			travelled += length

		self.report(blocked_leg >= 0, blocked_leg, blocked_at, worst)

	def leg_clearance_bound(self, field, ax, ay, bx, by, pad_cells):
		rows, cols = field.dist.shape

		col0 = int((min(ax, bx) - field.origin_x) / field.res) - pad_cells
		col1 = int((max(ax, bx) - field.origin_x) / field.res) + pad_cells + 1
		row0 = int((min(ay, by) - field.origin_y) / field.res) - pad_cells
		row1 = int((max(ay, by) - field.origin_y) / field.res) + pad_cells + 1

		if col1 < 0 or col0 >= cols or row1 < 0 or row0 >= rows:
			return float("inf")

		window = field.dist[max(0, row0):min(rows, row1), max(0, col0):min(cols, col1)]
		if window.size == 0:
			return None

		return float(window.min())

	def report(self, blocked, leg, distance, clearance):
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
		try:
			self.solve(msg)
		except Exception:
			rospy.logerr("planner: request %d failed:\n%s", msg.request_id, traceback.format_exc())
			self.publish_reply(msg.request_id, PlanReply.INTERNAL_ERROR, [], False, 0.0, 0)

	def corridor_for(self, geofence, msg):
		if len(geofence) < 2:
			return None, None

		key = (tuple(geofence), msg.corridor_radius)

		with self.lock:
			if key != self.corridor_key:
				self.corridor_key = key
				self.corridor = corridor_from_polyline(geofence, msg.corridor_radius)
				self.corridor_flags = None

			if self.corridor_flags is None and self.extent is not None:
				self.corridor_flags = self.mark_capsules(self.corridor)

			return self.corridor, self.corridor_flags

	def solve(self, msg):
		corridor, active = self.corridor_for([(p.x, p.y) for p in msg.corridor], msg)
		leg = [(msg.start.x, msg.start.y), (msg.goal.x, msg.goal.y)]

		snapshot = self.field_for(msg.clearance, corridor, active, leg, msg.corridor_radius)
		if snapshot is None:
			rospy.logwarn_throttle(5.0, "planner: request %d arrived before any costmap" % msg.request_id)
			self.publish_reply(msg.request_id, PlanReply.NO_COSTMAP, [], False, 0.0, 0)
			return

		field, extent = snapshot

		started = time.time()

		goal_x, goal_y = msg.goal.x, msg.goal.y
		nudged = nudge_goal(field, goal_x, goal_y, msg.start.x, msg.start.y, 0.5 * msg.corridor_radius)
		if nudged:
			rospy.loginfo("planner: request %d goal was blocked, moved %.1fm to clear water", msg.request_id, math.hypot(nudged[0] - goal_x, nudged[1] - goal_y))
			goal_x, goal_y = nudged

			field.adopt_centreline([(msg.start.x, msg.start.y), (goal_x, goal_y)], msg.corridor_radius)

		raw = self.search.plan(field, msg.start.x, msg.start.y, goal_x, goal_y, msg.corridor_radius)
		path = smooth_path(field, raw, ROUTE_SAMPLE_CELLS * field.res, extent) if raw else []
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