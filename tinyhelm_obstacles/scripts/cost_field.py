import math
import numpy as np

LETHAL = float("inf")

# How much a path pays for hugging the inflation boundary, as a multiple of the distance travelled.
# Not a parameter: above roughly this value the standoff a path settles at stops depending on it and
# is set by soft_radius alone, so exposing it only offered a way to make things worse. Two also caps
# the worst case segment cost at three times its length, which is what budget_factor already allows;
# a larger weight buys detours that budget_factor then throws away as unplannable.
SOFT_WEIGHT = 2.0

DIVERGENCE_WEIGHT = 2.0

def segment_distance(px, py, ax, ay, bx, by):
	dx = bx - ax
	dy = by - ay
	len2 = dx * dx + dy * dy
	t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / len2)) if len2 > 0.0 else 0.0
	return math.hypot(px - (ax + t * dx), py - (ay + t * dy))

class Capsule:
	"""Line segment with a radius, used as one link of the corridor tube around the strategic legs.
	distance() works on scalars and numpy arrays alike."""

	def __init__(self, x1, y1, x2, y2, radius):
		self.x1 = x1
		self.y1 = y1
		self.x2 = x2
		self.y2 = y2
		self.radius = radius

	def distance(self, px, py):
		dx = self.x2 - self.x1
		dy = self.y2 - self.y1
		len2 = dx * dx + dy * dy
		if len2 > 0.0:
			t = np.clip(((px - self.x1) * dx + (py - self.y1) * dy) / len2, 0.0, 1.0)
		else:
			t = 0.0

		return np.hypot(px - (self.x1 + t * dx), py - (self.y1 + t * dy))

	def contains(self, px, py):
		return self.distance(px, py) <= self.radius

	def contains_point(self, px, py):
		"""Scalar version. The array path goes through numpy for the rasteriser, which costs a couple
		of microseconds a call and is not worth paying on a per sample test inside the search."""
		dx = self.x2 - self.x1
		dy = self.y2 - self.y1
		len2 = dx * dx + dy * dy
		t = max(0.0, min(1.0, ((px - self.x1) * dx + (py - self.y1) * dy) / len2)) if len2 > 0.0 else 0.0
		ox = px - (self.x1 + t * dx)
		oy = py - (self.y1 + t * dy)
		return ox * ox + oy * oy <= self.radius * self.radius

def corridor_from_polyline(points, radius):
	"""One capsule per leg of the polyline. Both the planner and the marker overlay derive the tube
	from the same polyline this way, so what is drawn is what the search was actually held to."""
	return [Capsule(points[i - 1][0], points[i - 1][1], points[i][0], points[i][1], radius) for i in range(1, len(points))]

class CostField:
	"""Snapshot of the obstacle grid window turned into planning costs.

	A euclidean distance-to-obstacle field gives hard inflation (lethal within inflate_radius) and a
	soft penalty that falls to nothing at soft_radius, so paths keep standoff where they can and
	centre themselves between obstacles where they cannot. An optional corridor (union of capsules
	around the strategic legs) marks everything outside the allowed tube unplannable as well.

	Corridor violation is answerable on its own through outside_corridor_at, separately from the
	combined verdict cost_at gives. They used to be the same predicate, which made a waypoint merely
	outside its corridor indistinguishable from one sitting on a rock, and the caller would then
	route past it as though it were occupied.

	All world queries treat anything outside the field extent as free, which is what lets the
	planner treat unseen space beyond the loaded window as clear."""

	def __init__(self):
		self.res = 1.0
		self.origin_x = 0.0
		self.origin_y = 0.0
		self.size = 0
		self.inflate = 0.0
		self.soft = 0.0
		self.dist = None
		self.corridor_ok = None
		self.corridor = None
		self.centreline = None
		self.divergence_radius = 0.0
		self.divergence = None

	def adopt(self, resolution, origin_x, origin_y, dist, inflate_radius, soft_radius, corridor, centreline=None, divergence_radius=0.0):
		"""Takes a distance field computed elsewhere, which is how the planner consumes what the
		costmap publishes. Inflation and the soft falloff are applied here rather than baked into the
		grid, so the clearance can follow the controller's reconfigurable divergence band without the
		costmap having to know anything about the vessel."""
		self.res = resolution
		self.origin_x = origin_x
		self.origin_y = origin_y
		self.size = dist.shape[0]
		self.inflate = inflate_radius

		# Keeping soft strictly above inflate leaves the falloff a non-zero span to work over, so
		# the ramp needs no special case for a soft radius configured below the hard one
		self.soft = max(soft_radius, inflate_radius + resolution)
		self.dist = np.minimum(dist, self.soft)

		self.rasterise_corridor(corridor)
		self.adopt_centreline(centreline, divergence_radius)

	def adopt_centreline(self, centreline, divergence_radius):
		"""The line a path is meant to be running, kept as points rather than rasterised up front.
		Nothing here knows the mission: the planner is stateless and is handed one leg per request, so
		there is nothing long lived enough for precomputing to pay for. Cells are filled in as the
		search touches them.

		The cache cannot go stale. A field is built fresh from a locked snapshot for every request and
		is discarded with it, so it never outlives the polyline and the distances it was filled from,
		and there is no invalidation to get wrong.

		Deliberately the polyline alone rather than the capsules the corridor is built from. That union
		carries a disc at the vessel, and measuring deviation against the disc would read as zero
		exactly where the pull back onto the line is most needed."""
		self.centreline = list(centreline) if centreline and len(centreline) >= 2 else None
		self.divergence_radius = divergence_radius

		if self.centreline and divergence_radius > 0.0:
			self.divergence = np.full((self.size, self.size), -1.0, dtype=np.float32)
		else:
			self.divergence = None

	def rasterise_corridor(self, corridor):
		# Kept alongside the raster so the corridor can still be answered for outside the window
		self.corridor = corridor or None

		if not corridor:
			self.corridor_ok = np.ones((self.size, self.size), dtype=bool)
			return

		# Stamping each capsule's bounding box keeps lookups O(1)
		self.corridor_ok = np.zeros((self.size, self.size), dtype=bool)
		for capsule in corridor:
			x0, y0 = self.bound_cell(min(capsule.x1, capsule.x2) - capsule.radius, min(capsule.y1, capsule.y2) - capsule.radius)
			x1, y1 = self.bound_cell(max(capsule.x1, capsule.x2) + capsule.radius, max(capsule.y1, capsule.y2) + capsule.radius)
			cols, rows = np.meshgrid(np.arange(x0, x1 + 1), np.arange(y0, y1 + 1))
			wx = self.origin_x + (cols + 0.5) * self.res
			wy = self.origin_y + (rows + 0.5) * self.res
			self.corridor_ok[y0:y1 + 1, x0:x1 + 1] |= capsule.contains(wx, wy)

	def world_to_cell(self, x, y):
		"""Returns (col, row) or None when outside the field."""
		cx = math.floor((x - self.origin_x) / self.res)
		cy = math.floor((y - self.origin_y) / self.res)
		if cx < 0 or cy < 0 or cx >= self.size or cy >= self.size:
			return None

		return cx, cy

	def cell_to_world(self, cx, cy):
		return self.origin_x + (cx + 0.5) * self.res, self.origin_y + (cy + 0.5) * self.res

	def bound_cell(self, x, y):
		cx = max(0, min(self.size - 1, math.floor((x - self.origin_x) / self.res)))
		cy = max(0, min(self.size - 1, math.floor((y - self.origin_y) / self.res)))
		return cx, cy

	def soft_cost(self, distance):
		"""Dimensionless penalty per metre travelled: one at the inflation boundary, nothing at
		soft_radius, squared so the gradient bites hardest close in and barely perturbs the search
		far out. Convex and symmetric, which is what makes a path centre itself in a gap regardless
		of how the weight is set."""
		if distance <= self.inflate:
			return LETHAL
		if distance >= self.soft:
			return 0.0

		shortfall = (self.soft - distance) / (self.soft - self.inflate)
		return SOFT_WEIGHT * shortfall * shortfall

	def deviation_at(self, x, y):
		return min(segment_distance(x, y, self.centreline[i - 1][0], self.centreline[i - 1][1], self.centreline[i][0], self.centreline[i][1]) for i in range(1, len(self.centreline)))

	def divergence_cost(self, x, y):
		"""Dimensionless penalty per metre travelled: nothing on the line, DIVERGENCE_WEIGHT at the
		fence. Linear rather than squared, unlike the obstacle falloff, because a squared term goes
		flat near the line: a path drifts most of the way back and then stops caring and wanders. A
		constant gradient is what actually rejoins the course, and its steepness is what decides
		whether the return is a lazy sweep or a hard cut back."""
		if self.divergence is None:
			return 0.0

		return DIVERGENCE_WEIGHT * min(1.0, self.deviation_at(x, y) / self.divergence_radius)

	def divergence_cell(self, cx, cy):
		if self.divergence is None:
			return 0.0

		penalty = self.divergence[cy, cx]
		if penalty < 0.0:
			penalty = self.divergence_cost(*self.cell_to_world(cx, cy))
			self.divergence[cy, cx] = penalty

		return penalty

	def cost_cell(self, cx, cy):
		if not self.corridor_ok[cy, cx]:
			return LETHAL

		# Added only once the cell is known to be passable. Lethality travels as a sentinel compared
		# with equality all through the search, so adding anything to it would leave a value that is
		# still infinite but no longer tests as lethal.
		soft = self.soft_cost(self.dist[cy, cx])
		if soft == LETHAL:
			return LETHAL

		return soft + self.divergence_cell(cx, cy)

	def in_corridor(self, x, y):
		if not self.corridor:
			return True

		return any(capsule.contains_point(x, y) for capsule in self.corridor)

	def cost_at(self, x, y):
		cell = self.world_to_cell(x, y)
		if cell:
			return self.cost_cell(*cell)

		# Unseen space beyond the window counts as clear of obstacles, but the corridor still applies
		# out there. It is anchored to the mission rather than to the map, and a search allowed to slip
		# round the edge of the window would be left with no bound on it at all.
		if not self.in_corridor(x, y):
			return LETHAL

		return self.divergence_cost(x, y)

	def outside_corridor_at(self, x, y):
		cell = self.world_to_cell(x, y)
		if cell:
			return not self.corridor_ok[cell[1], cell[0]]

		return not self.in_corridor(x, y)

	def obstacle_distance_at(self, x, y):
		"""Clamped at soft_radius inside the field; unseen space outside it reads as infinitely
		clear, which is what keeps the planner from treating the window edge as a wall."""
		cell = self.world_to_cell(x, y)
		return self.dist[cell[1], cell[0]] if cell else LETHAL

# Published as a spec nav_msgs/OccupancyGrid so it renders in rviz as an ordinary costmap and needs
# no message of its own. The costmap_2d convention: 100 is definitely occupied, 0 is free. Unknown
# (-1) is never emitted, because unseen space is deliberately treated as clear.
COST_LETHAL = 100

def encode_cost(dist, inflate_radius, soft_radius):
	"""Clamped distance field to publishable 0..100 costs. Quantisation is finest close in, where
	the quadratic is steep and the cost actually matters, and coarsest out near soft_radius where the
	cost is nearly nothing either way."""
	soft = max(soft_radius, inflate_radius + 1e-6)
	shortfall = np.clip((soft - dist) / (soft - inflate_radius), 0.0, 1.0)
	graded = np.rint(shortfall * shortfall * (COST_LETHAL - 1))
	return np.where(dist <= inflate_radius, COST_LETHAL, graded).astype(np.int8)

def decode_distance(values, inflate_radius, soft_radius):
	"""Inverse of encode_cost. Kept adjacent to it deliberately: the two have to agree exactly, and a
	round trip test over the pair catches a disagreement that would otherwise surface as a planner
	that quietly believes obstacles are somewhere they are not."""
	soft = max(soft_radius, inflate_radius + 1e-6)
	graded = np.asarray(values, dtype=np.float64)
	shortfall = np.sqrt(np.clip(graded / (COST_LETHAL - 1), 0.0, 1.0))
	distance = soft - shortfall * (soft - inflate_radius)

	# Lethality travels as its own symbol, so decoding must never manufacture it: a graded cell that
	# lands exactly on the inflation boundary would come back lethal and quietly eat the margin the
	# corridor check relies on. Half a quantum of bias keeps every graded value strictly outside.
	margin = 0.5 * (soft - inflate_radius) / (COST_LETHAL - 1)
	distance = np.maximum(distance, inflate_radius + margin)

	return np.where(graded >= COST_LETHAL, 0.0, distance)
