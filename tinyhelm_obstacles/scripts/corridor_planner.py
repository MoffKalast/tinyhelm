import math
from cost_field import Capsule
from theta_star import ThetaStar, path_length, smooth_leg

OK = "ok"
WARN = "warn"
REPLAN = "replan"
ERROR = "error"


class CorridorPlanner:
	"""All planning logic and mission progress state, free of any ROS dependency. The
	node owns subscriptions, TF and message conversion and hands in the mission and
	tactical path as lists of (x, y, z) tuples plus a freshly built CostField each tick;
	this class decides whether the corridor is intruded and assembles corrections.

	status_cb(level, message) mirrors the MonitorStatus publications of the C++ node,
	log_info/log_warn mirror its rosconsole output; the node wires these to ROS."""

	def __init__(self, config, status_cb, log_info, log_warn):
		self.cfg = config
		self.status_cb = status_cb
		self.log_info = log_info
		self.log_warn = log_warn

		self.theta_star = ThetaStar()
		self.effective_inflate = config["inflate_radius"]

		self.mission = []
		self.tactical = []
		self.remaining = []
		self.next_wp_index = 0
		self.unreachable_counts = {}
		self.skipped = set()
		self.last_published = []

	def set_mission(self, waypoints):
		self.mission = list(waypoints)
		self.remaining = []
		self.next_wp_index = 0
		self.unreachable_counts.clear()
		self.skipped.clear()
		self.last_published = []
		if not self.mission:
			self.status_cb(OK, "No active mission.")

	def set_tactical(self, points):
		self.tactical = list(points)

	def update_remaining(self, rx, ry):
		"""Strategic waypoints are always visited in order (detours never skip them), so
		passing within reach radius of the next one is the ground truth for progress.
		Waypoints skipped as occupied can never be reached, so they count as passed once
		the vessel crosses the perpendicular at the waypoint along the outbound corridor
		direction."""
		while self.next_wp_index < len(self.mission):
			px, py, _ = self.mission[self.next_wp_index]
			if math.hypot(px - rx, py - ry) <= self.cfg["waypoint_reached_radius"]:
				self.next_wp_index += 1
				continue
			if self.next_wp_index in self.skipped and self.next_wp_index + 1 < len(self.mission):
				qx, qy, _ = self.mission[self.next_wp_index + 1]
				if (rx - px) * (qx - px) + (ry - py) * (qy - py) > 0.0:
					self.next_wp_index += 1
					continue
			break

		self.remaining = self.mission[self.next_wp_index:]

	def build_geofence(self, rx, ry):
		fence = []
		px, py = rx, ry
		for wx, wy, _ in self.remaining:
			leg = math.hypot(wx - px, wy - py)
			radius = min(self.cfg["max_detour"], max(self.cfg["min_detour"], leg * self.cfg["detour_leg_fraction"]))
			fence.append(Capsule(px, py, wx, wy, radius))
			px, py = wx, wy
		return fence

	def corridor_point_blocked(self, field, x, y):
		"""Trip threshold shared by the tactical corridor check and the correction
		validity check; corrections are planned with effective_inflate + res clearance,
		so the band between planning margin and trip threshold gives hysteresis against
		replan oscillation. Outside the loaded window the field reports infinite
		clearance, i.e. unseen space never trips."""
		return field.obstacle_distance_at(x, y) < self.effective_inflate - self.cfg["resolution"]

	def corridor_point_clear(self, field, x, y, margin):
		return field.obstacle_distance_at(x, y) >= margin

	def segment_blocked(self, field, ax, ay, bx, by):
		length = math.hypot(bx - ax, by - ay)
		steps = max(1, int(length / (self.cfg["resolution"] * 2.0)))
		for s in range(steps + 1):
			t = s / steps
			if self.corridor_point_blocked(field, ax + t * (bx - ax), ay + t * (by - ay)):
				return True
		return False

	def corridor_clear(self, field):
		for i in range(1, len(self.tactical)):
			x1, y1, _ = self.tactical[i - 1]
			x2, y2, _ = self.tactical[i]
			if self.segment_blocked(field, x1, y1, x2, y2):
				return False
		return True

	def published_still_valid(self, field, rx, ry):
		"""The active correction stays in force while the vessel is still on it and
		everything from the vessel's progress point to the end remains obstacle-free."""
		if len(self.last_published) < 2:
			return False

		seg = 1
		best = float("inf")
		best_t = 0.0
		for i in range(1, len(self.last_published)):
			x1, y1, _ = self.last_published[i - 1]
			x2, y2, _ = self.last_published[i]
			dx, dy = x2 - x1, y2 - y1
			len2 = dx * dx + dy * dy
			t = max(0.0, min(1.0, ((rx - x1) * dx + (ry - y1) * dy) / len2)) if len2 > 0.0 else 0.0
			d = math.hypot(rx - (x1 + t * dx), ry - (y1 + t * dy))
			if d < best:
				best, seg, best_t = d, i, t
		if best > self.cfg["waypoint_reached_radius"]:
			return False

		x1, y1, _ = self.last_published[seg - 1]
		x2, y2, _ = self.last_published[seg]
		px = x1 + best_t * (x2 - x1)
		py = y1 + best_t * (y2 - y1)
		for i in range(seg, len(self.last_published)):
			x2, y2, _ = self.last_published[i]
			if self.segment_blocked(field, px, py, x2, y2):
				return False
			px, py = x2, y2
		return True

	def find_rejoin(self, field, rx, ry):
		"""First clear point on the current strategic leg at or past the robot's
		projection. Clearance demands effective_inflate + res while the corridor check
		trips below effective_inflate - res, so a published rejoin path passes its own
		corridor check."""
		if self.next_wp_index == 0 or self.next_wp_index >= len(self.mission):
			return None
		ax, ay, _ = self.mission[self.next_wp_index - 1]
		bx, by, _ = self.mission[self.next_wp_index]
		dx, dy = bx - ax, by - ay
		length = math.hypot(dx, dy)
		if length < 2.0 * self.cfg["resolution"]:
			return None

		t = max(0.0, min(1.0, ((rx - ax) * dx + (ry - ay) * dy) / (length * length)))
		step = self.cfg["resolution"] / length
		margin = self.effective_inflate + self.cfg["resolution"]
		while t <= 1.0:
			x, y = ax + t * dx, ay + t * dy
			if self.corridor_point_clear(field, x, y, margin) and not field.lethal_at(x, y):
				return x, y
			t += step
		return None

	def find_clear_along(self, field, fx, fy, tx, ty):
		"""First clear point walking from (fx, fy) toward (tx, ty), used to project
		substitutes for an occupied waypoint onto its inbound and outbound corridors."""
		dx, dy = tx - fx, ty - fy
		length = math.hypot(dx, dy)
		if length < self.cfg["resolution"]:
			return None

		step = (self.cfg["resolution"] * 2.0) / length
		margin = self.effective_inflate + self.cfg["resolution"]
		t = step
		while t <= 1.0:
			x, y = fx + t * dx, fy + t * dy
			if self.corridor_point_clear(field, x, y, margin):
				return x, y
			t += step
		return None

	def plan_leg(self, field, sx, sy, gx, gy):
		leg = self.theta_star.plan(field, sx, sy, gx, gy, self.cfg["max_detour"])
		return smooth_leg(field, leg, self.cfg["resolution"] * 2.0) if leg else []

	def plan_corridor_leg(self, field, ax, ay, bx, by):
		"""Follows the strategic line a->b, detouring with Theta* around each blocked
		stretch and rejoining the line immediately after it, so deviation from the
		surveyed corridor stays minimal instead of cutting a taut diagonal to the far
		endpoint. Empty on failure."""
		dx, dy = bx - ax, by - ay
		length = math.hypot(dx, dy)
		margin = self.effective_inflate + self.cfg["resolution"]
		steps = max(1, int(math.ceil(length / (self.cfg["resolution"] * 2.0))))

		def at(i):
			t = i / steps
			return ax + t * dx, ay + t * dy

		def clear_at(i):
			x, y = at(i)
			return self.corridor_point_clear(field, x, y, margin)

		points = [(ax, ay)]
		i = 0
		while i < steps:
			if clear_at(i + 1):
				i += 1
				continue

			to = i + 1
			while to <= steps and not clear_at(to):
				to += 1
			start = at(i)
			goal = at(to) if to <= steps else (bx, by)
			detour = self.plan_leg(field, start[0], start[1], goal[0], goal[1])
			if not detour:
				return []
			for p in detour:
				self.append_point(points, p)
			i = to

		self.append_point(points, (bx, by))
		return points

	def append_point(self, points, p):
		if points and math.hypot(points[-1][0] - p[0], points[-1][1] - p[1]) < 1e-6:
			return
		points.append(p)

	def append_leg(self, out, leg, z):
		for x, y in leg:
			if out and math.hypot(out[-1][0] - x, out[-1][1] - y) < 1e-6:
				continue
			out.append((x, y, z))

	def strategic_index(self, remaining_index):
		return self.next_wp_index + remaining_index

	def bump_unreachable(self, remaining_index):
		"""Returns True once the failure has persisted long enough to declare the
		waypoint unreachable; the status is published only on the confirming transition."""
		si = self.strategic_index(remaining_index)
		count = self.unreachable_counts.get(si, 0) + 1
		self.unreachable_counts[si] = min(count, self.cfg["unreachable_cycles"])
		if count == self.cfg["unreachable_cycles"]:
			self.status_cb(ERROR, "Waypoint %d unreachable after repeated attempts, skipping." % si)
		return count >= self.cfg["unreachable_cycles"]

	def replan(self, field, rx, ry):
		"""Assembles a full correction through the remaining waypoints. Returns the
		correction as a list of (x, y, z) or None when withheld (mid-confirmation or no
		usable course); statuses are emitted through the callback along the way."""
		res = self.cfg["resolution"]
		out = []
		px, py = rx, ry
		on_line = False

		# Return to the current survey line first. The correction is consumed as a
		# revised plan, so its first pose is the anchor of the leg in progress, not a
		# goal to visit.
		rejoin = self.find_rejoin(field, rx, ry)
		if rejoin:
			jx, jy = rejoin
			back = self.plan_leg(field, rx, ry, jx, jy)
			direct = math.hypot(jx - rx, jy - ry)
			if back and (direct <= 2.0 * res or path_length(back) <= self.cfg["budget_factor"] * direct):
				self.append_leg(out, back, self.mission[self.next_wp_index][2])
				px, py = jx, jy
				on_line = True

		# Connect as many remaining waypoints as possible: individually unplannable ones
		# are skipped after confirmation instead of abandoning the whole correction, so
		# one blocked waypoint can't silence the planner while the vessel sails on
		any_skipped = False
		for wi in range(len(self.remaining)):
			gx, gy, z = self.remaining[wi]
			si = self.strategic_index(wi)

			if field.lethal_at(gx, gy):
				done, dropped = self.route_past_occupied(field, out, wi, px, py, on_line)
				if not done:
					return None
				if not dropped:
					px, py = out[-1][0], out[-1][1]
					on_line = True
				any_skipped = True
				continue

			# Legs that start on the strategic line hug it; the fallback direct plan
			# covers legs starting off-line (no rejoin found) or corridor legs that
			# failed to close
			leg = []
			if on_line or wi > 0:
				leg = self.plan_corridor_leg(field, px, py, gx, gy)
			if not leg:
				leg = self.plan_leg(field, px, py, gx, gy)

			direct = math.hypot(gx - px, gy - py)
			over_budget = leg and direct > 2.0 * res and path_length(leg) > self.cfg["budget_factor"] * direct

			if not leg or over_budget:
				# During confirmation keep the active correction untouched; once
				# confirmed, skip this waypoint and keep connecting the rest
				if not self.bump_unreachable(wi):
					if over_budget:
						self.status_cb(WARN, "Detour to waypoint %d exceeds budget, confirming..." % si)
					else:
						self.status_cb(WARN, "No path within geofence to waypoint %d, confirming..." % si)
					return None
				self.skipped.add(si)
				any_skipped = True
				continue

			self.append_leg(out, leg, z)
			out[-1] = self.remaining[wi]
			self.unreachable_counts.pop(si, None)
			self.skipped.discard(si)
			px, py = gx, gy
			on_line = True

		# A correction ending at the last plannable point is a safe stop; only one too
		# short to follow at all is withheld
		if len(out) < 2:
			self.status_cb(ERROR, "No usable corrected course, all remaining waypoints blocked.")
			return None

		self.last_published = out
		if any_skipped:
			self.status_cb(REPLAN, "Corrected course planned, occupied waypoint(s) skipped via corridor.")
		else:
			self.status_cb(REPLAN, "Corrected course planned around obstacles.")
		self.log_info("replanned %d poses through %d waypoints%s" % (len(out), len(self.remaining), " (skips)" if any_skipped else ""))
		return out

	def route_past_occupied(self, field, out, wi, px, py, on_line):
		"""Handles one occupied waypoint: after confirmation over several cycles it is
		substituted by projections onto its inbound and outbound corridors, so the
		vessel hugs the survey lines past the blockage instead of cutting a diagonal.
		Returns (done, dropped): done False means mid-confirmation, abort this replan;
		dropped True means the waypoint had to be abandoned with no substitute legs."""
		gx, gy, z = self.remaining[wi]
		si = self.strategic_index(wi)

		if si not in self.skipped:
			count = self.unreachable_counts.get(si, 0) + 1
			self.unreachable_counts[si] = count
			if count < self.cfg["unreachable_cycles"]:
				self.status_cb(WARN, "Waypoint %d occupied, confirming..." % si)
				return False, False
			self.skipped.add(si)
			self.log_warn("waypoint %d occupied, substituting corridor projections" % si)

		# Entry substitute: closest clear point to the waypoint on the inbound corridor.
		# A degenerate corridor (duplicated turn waypoint on and_return missions) falls
		# back to the current chain position if that is itself clear; a corridor blocked
		# along its whole length (wall running beside it) drops the waypoint entirely
		# and routes toward the next one.
		entry = self.find_clear_along(field, gx, gy, px, py)
		if entry is None:
			if self.corridor_point_clear(field, px, py, self.effective_inflate + self.cfg["resolution"]):
				entry = (px, py)
			else:
				self.log_warn("waypoint %d and its corridor are blocked, dropping it" % si)
				return True, True

		ex, ey = entry
		leg = self.plan_corridor_leg(field, px, py, ex, ey) if on_line else []
		if not leg:
			leg = self.plan_leg(field, px, py, ex, ey)
		if not leg:
			self.log_warn("no path toward occupied waypoint %d, dropping it" % si)
			return True, True
		self.append_leg(out, leg, z)
		px, py = ex, ey

		# Exit substitute: first clear point past the waypoint on the outbound corridor,
		# so the next leg starts clear and exactly on its own survey line
		if wi + 1 < len(self.remaining):
			qx, qy, _ = self.remaining[wi + 1]
			exit_point = self.find_clear_along(field, gx, gy, qx, qy)
			if exit_point:
				hop = self.plan_leg(field, px, py, exit_point[0], exit_point[1])
				if hop:
					self.append_leg(out, hop, z)

		return True, False
