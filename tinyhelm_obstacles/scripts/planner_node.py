#!/usr/bin/env python3
import threading
import time

import numpy as np
import rospy

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid

from coarse_heuristic import CoarseHeuristic
from cost_field import CostField, corridor_from_polyline, decode_distance
from theta_star import GOAL_IN_OBSTACLE, GOAL_OUTSIDE_CORRIDOR, NO_ROUTE, OK, START_TRAPPED, ThetaStar, smooth_path
from tinyhelm_obstacles.msg import PathStatus, PathWatch, PlanReply, PlanRequest

RESULT_CODES = {
	OK: PlanReply.OK,
	GOAL_IN_OBSTACLE: PlanReply.GOAL_IN_OBSTACLE,
	GOAL_OUTSIDE_CORRIDOR: PlanReply.GOAL_OUTSIDE_CORRIDOR,
	START_TRAPPED: PlanReply.START_TRAPPED,
	NO_ROUTE: PlanReply.NO_ROUTE,
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

		self.soft_radius = rospy.get_param("~soft_radius", 15.0)
		self.coarse_factor = rospy.get_param("~coarse_factor", 4)
		self.status_period = rospy.get_param("~status_period", 1.0)

		self.lock = threading.Lock()
		self.dist = None
		self.origin = (0.0, 0.0)
		self.res = 0.5

		self.watched = []
		self.watch_clearance = 0.0
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
		values = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

		# The costmap publishes the distance field with no hard inflation baked in, so clearance is
		# applied here and can follow the controller's reconfigurable corridor width
		dist = decode_distance(values, 0.0, self.soft_radius)

		with self.lock:
			self.dist = dist
			self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
			self.res = msg.info.resolution

		self.review_watched()

	def watch_callback(self, msg):
		with self.lock:
			self.watched = [(p.x, p.y) for p in msg.path]
			self.watch_clearance = msg.clearance
			self.last_blocked = None

		self.review_watched()

	def field_for(self, clearance, corridor):
		with self.lock:
			if self.dist is None:
				return None
			dist = self.dist
			origin = self.origin
			res = self.res

		field = CostField()
		field.adopt(res, origin[0], origin[1], dist, clearance, self.soft_radius, corridor)
		return field

	def review_watched(self):
		"""Walks the watched route and reports the first place it is obstructed. Sampling at half the
		grid resolution so a route cannot thread between samples past a cell it actually clips."""
		with self.lock:
			route = list(self.watched)
			clearance = self.watch_clearance

		if len(route) < 2:
			return

		field = self.field_for(clearance, None)
		if field is None:
			return

		step = 0.5 * self.res
		travelled = 0.0
		worst = float("inf")
		blocked_leg = -1
		blocked_at = 0.0

		for leg in range(1, len(route)):
			ax, ay = route[leg - 1]
			bx, by = route[leg]
			length = ((bx - ax) ** 2 + (by - ay) ** 2) ** 0.5
			samples = max(1, int(length / step))
			for s in range(samples + 1):
				t = s / samples
				gap = field.obstacle_distance_at(ax + t * (bx - ax), ay + t * (by - ay))
				worst = min(worst, gap)
				if blocked_leg < 0 and gap < clearance:
					blocked_leg = leg
					blocked_at = travelled + t * length

			travelled += length

		self.report(blocked_leg >= 0, blocked_leg, blocked_at, worst)

	def report(self, blocked, leg, distance, clearance):
		# Reported on every change, and repeated while obstructed so a monitor that came up late still
		# learns about an intrusion without needing the whole thing restated at grid rate
		now = rospy.Time.now()
		changed = blocked != self.last_blocked
		if not changed and not (blocked and (now - self.last_status).to_sec() >= self.status_period):
			return

		self.last_blocked = blocked
		self.last_status = now

		msg = PathStatus()
		msg.blocked = blocked
		msg.blocked_leg = leg
		msg.blocked_distance = distance
		msg.min_clearance = 0.0 if clearance == float("inf") else clearance
		self.status_pub.publish(msg)

	def request_callback(self, msg):
		corridor = corridor_from_polyline([(p.x, p.y) for p in msg.corridor], msg.corridor_radius) if len(msg.corridor) >= 2 else None

		field = self.field_for(msg.clearance, corridor)
		if field is None:
			rospy.logwarn_throttle(5.0, "planner: request %d arrived before any costmap" % msg.request_id)
			self.publish_reply(msg.request_id, PlanReply.NO_COSTMAP, [], False, 0.0, 0)
			return

		started = time.time()
		heuristic = CoarseHeuristic(field, msg.goal.x, msg.goal.y, factor=self.coarse_factor)
		raw = self.search.plan(field, msg.start.x, msg.start.y, msg.goal.x, msg.goal.y, msg.corridor_radius, heuristic=heuristic)
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
