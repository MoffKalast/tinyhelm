import math

WALK_RUNNING = "running"
WALK_DONE = "done"
WALK_ABANDONED = "abandoned"
WALK_WITHHELD = "withheld"

class MissionState:
	"""Everything the monitor knows about the mission it is watching, and nothing that needs a grid.

	Progress along the mission is deliberately monotonic: strategic waypoints are always visited in
	order, so passing within reach of the next one is the ground truth and being pushed back by wind
	or current does not un-visit it.

	The corridor is built fresh every time it is asked for, from the vessel's position now and the
	waypoints still to visit. It used to be cached at the moment a mission arrived, computed from the
	previous mission's remaining waypoints because the new one had not been applied yet, and never
	rebuilt afterwards. Building on demand removes the possibility rather than fixing an instance of
	it: there is nowhere for a stale tube to live."""

	def __init__(self, unreachable_cycles):
		self.unreachable_cycles = unreachable_cycles

		self.mission = []
		self.next_index = 0
		self.failures = {}
		self.skipped = set()

	def set_mission(self, waypoints):
		self.mission = list(waypoints)
		self.next_index = 0
		self.failures = {}
		self.skipped = set()

	def active(self):
		return len(self.mission) > 0 and self.next_index < len(self.mission)

	def passed(self, index, rx, ry):
		"""Whether the vessel has gone by mission[index].

		Measured along the leg that arrives at the waypoint and satisfied once the vessel's projection
		onto that leg reaches its far end, however far to the side it actually passed. Proximity to the
		waypoint cannot express that: miss one by more than a radius while detouring round an obstacle
		and progress stops for good, because every later test is then against a waypoint astern, which
		forward motion can never satisfy. One missed waypoint used to poison the whole mission.

		Oriented by the incoming leg and never by the outgoing one. A survey pattern doubles back on
		itself, so at a turn the next waypoint lies back the way we came, and an outgoing orientation
		would call the turn passed while the vessel was still running up to it."""
		if index >= len(self.mission):
			return False

		bx, by, _ = self.mission[index]

		if index > 0:
			ax, ay, _ = self.mission[index - 1]
			reach = 1.0
		elif len(self.mission) > 1:
			# The first waypoint is where the mission was anchored, at the vessel, so no leg arrives at
			# it. Being on the outbound side of it is as much as can be asked.
			ax, ay = bx, by
			bx, by, _ = self.mission[1]
			reach = 0.0
		else:
			return False

		dx, dy = bx - ax, by - ay
		length_sq = dx * dx + dy * dy
		if length_sq < 1e-9:
			return True

		return ((rx - ax) * dx + (ry - ay) * dy) / length_sq >= reach

	def update_progress(self, rx, ry):
		"""Returns True when the vessel has passed one more waypoint.

		One per call, deliberately. A pattern that crosses itself leaves the vessel beyond the
		perpendicular of waypoints it has no business having reached, and a loop here would run through
		all of them on the strength of a single coincidence. One at a time means a wrong advance costs
		one waypoint, and the next call has to earn the one after it."""
		if not self.passed(self.next_index, rx, ry):
			return False

		self.next_index += 1
		return True

	def remaining(self):
		return self.mission[self.next_index:]

	def corridor_polyline(self):
		"""The mission legs still to run, starting from the one currently being followed. The tube
		around this is the only thing bounding how far a correction may stray, and therefore also the
		only thing bounding how much space a search has to cover.

		Deliberately independent of where the vessel is. Anchoring the near end to the vessel let the
		tube sweep round as it manoeuvred, so it drifted a little further with every correction and
		never came back; worse, it followed the vessel into whichever side of an obstacle got picked
		first, so if that side turned out to be blocked there was no longer any corridor on the other
		side to find a way through."""
		first = max(0, self.next_index - 1)
		return [(x, y) for x, y, _ in self.mission[first:]]

	def leg_reference(self, index, ax, ay):
		"""The one mission leg a search is being asked to solve, as the two points it runs between.

		Per leg rather than the whole remaining mission because a survey pattern folds back on itself.
		The tube round the full polyline is a union, so rows closer together than the corridor radius
		merge into a single blob and bound the search far more loosely than intended; and distance to
		the full polyline reads a point on one row as being on course because it happens to be near
		the next row over. Neither is visible on a single outbound leg, which is why it went unnoticed.

		The first leg of a mission has no preceding waypoint, so the search's own start anchors it.
		Every other leg is anchored to the mission alone and so stays where the survey line is rather
		than following the vessel around."""
		if index <= 0:
			return [(ax, ay), (self.mission[0][0], self.mission[0][1])]

		return [(self.mission[index - 1][0], self.mission[index - 1][1]), (self.mission[index][0], self.mission[index][1])]

	def note_failure(self, index):
		"""Counts consecutive failures against a waypoint. Returns True once it has failed often
		enough to give up on, so a single unlucky search cannot drop a waypoint."""
		count = self.failures.get(index, 0) + 1
		self.failures[index] = count
		return count >= self.unreachable_cycles

	def note_success(self, index):
		self.failures.pop(index, None)
		self.skipped.discard(index)

	def give_up_on(self, index):
		self.skipped.add(index)

class ReplanWalk:
	"""Assembles one correction through the remaining waypoints, one search at a time.

	The old walk ran the whole thing inside a single call, which only worked because the search was a
	direct function call. With the search behind a topic pair this has to be a sequence of replies
	instead, so the walk holds its position between them and is advanced by accept().

	A waypoint that cannot be reached is confirmed over several attempts and then skipped rather than
	abandoning the whole correction, so one blocked waypoint mid-mission cannot silence the monitor
	while the vessel sails on."""

	def __init__(self, state, rx, ry):
		self.state = state
		self.targets = list(range(state.next_index, len(state.mission)))
		self.cursor = 0
		self.from_x = rx
		self.from_y = ry
		self.points = []
		self.any_skipped = False
		self.reasons = []

	def target_index(self):
		return self.targets[self.cursor] if self.cursor < len(self.targets) else None

	def pending_request(self):
		"""The leg to solve next, or None when the walk has run out of waypoints."""
		index = self.target_index()
		if index is None:
			return None

		gx, gy, _ = self.state.mission[index]
		return (self.from_x, self.from_y), (gx, gy), index

	def accept(self, reachable, path, reason):
		"""Folds one reply into the walk and reports what to do next.

		reachable False means this waypoint could not be reached this time; the walk either withholds
		the whole correction while it confirms that, or gives up on the waypoint and carries on to the
		next one. Withholding matters: publishing a correction that has quietly dropped a waypoint on
		the strength of one failed search is worse than publishing nothing."""
		index = self.target_index()
		if index is None:
			return WALK_DONE

		if reachable:
			self.append(path, self.state.mission[index][2])
			self.snap_last_to(index)
			self.state.note_success(index)
			self.from_x, self.from_y = self.state.mission[index][0], self.state.mission[index][1]
			self.cursor += 1
			return WALK_RUNNING if self.target_index() is not None else self.settle()

		if not self.state.note_failure(index):
			return WALK_WITHHELD

		self.state.give_up_on(index)
		self.any_skipped = True
		self.reasons.append((index, reason))

		# The vessel keeps its current position as the anchor for the next leg, so the correction runs
		# from where we are toward the next waypoint we can still reach rather than through the one we
		# just gave up on
		self.cursor += 1
		return WALK_RUNNING if self.target_index() is not None else self.settle()

	def settle(self):
		# A correction ending at the last waypoint we could reach is a safe course to steer. Only one
		# too short to follow at all is worth nothing.
		return WALK_DONE if len(self.points) >= 2 else WALK_ABANDONED

	def append(self, path, z):
		for x, y in path:
			if self.points and math.hypot(self.points[-1][0] - x, self.points[-1][1] - y) < 1e-6:
				continue
			self.points.append((x, y, z))

	def snap_last_to(self, index):
		"""Ends the leg on the waypoint itself rather than on the cell centre nearest to it, so
		quantisation cannot accumulate across chained legs and drift the route off the survey line."""
		if self.points:
			self.points[-1] = self.state.mission[index]

def drop_passed_legs(points, x, y):
	"""Trims legs the vessel has already run past. A search takes as long as it takes, so by the time
	a correction is ready its opening leg can be astern of us, and a controller has no way to tell a
	stale opening pose from a waypoint it is meant to visit. A leg counts as passed once the vessel's
	projection onto it reaches the far end."""
	if len(points) < 2:
		return points

	for i in range(len(points) - 1):
		ax, ay = points[i][0], points[i][1]
		bx, by = points[i + 1][0], points[i + 1][1]
		dx, dy = bx - ax, by - ay
		length_sq = dx * dx + dy * dy
		if length_sq < 1e-9:
			continue

		if ((x - ax) * dx + (y - ay) * dy) / length_sq < 1.0:
			return points[i:]

	return points[-2:]
