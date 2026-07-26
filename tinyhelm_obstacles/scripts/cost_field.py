import math
import numpy as np
from scipy import ndimage

LETHAL = float("inf")

# How much a path pays for hugging the inflation boundary, as a multiple of the distance travelled.
# Not a parameter: above roughly this value the standoff a path settles at stops depending on it and
# is set by soft_radius alone, so exposing it only offered a way to make things worse. Two also caps
# the worst case segment cost at three times its length, which is what budget_factor already allows;
# a larger weight buys detours that budget_factor then throws away as unplannable.
SOFT_WEIGHT = 2.0

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

	Obstacle lethality and corridor violation are deliberately separate queries. They used to be the
	same predicate, which made a waypoint merely outside its corridor indistinguishable from one
	sitting on a rock, and the caller would then route past it as though it were occupied.

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

	def build(self, resolution, origin_x, origin_y, occupied, inflate_radius, soft_radius, corridor):
		"""occupied is a [row, col] = [y, x] boolean array; the field is square."""
		self.res = resolution
		self.origin_x = origin_x
		self.origin_y = origin_y
		self.size = occupied.shape[0]
		self.inflate = inflate_radius

		# Keeping soft strictly above inflate leaves the falloff a non-zero span to work over, so
		# the ramp needs no special case for a soft radius configured below the hard one
		self.soft = max(soft_radius, inflate_radius + resolution)

		if occupied.any():
			# Distances past soft_radius do not affect cost, so clamping there loses nothing and
			# bounds how far a changed cell can reach, which is what makes incremental updates local
			self.dist = np.minimum(ndimage.distance_transform_edt(~occupied) * resolution, self.soft)
		else:
			# An all-free window has no background to measure to and scipy's answer in that case is
			# an artefact of the array bounds rather than anything meaningful
			self.dist = np.full((self.size, self.size), self.soft, dtype=float)

		self.rasterise_corridor(corridor)

	def adopt(self, resolution, origin_x, origin_y, dist, inflate_radius, soft_radius, corridor):
		"""Takes a distance field computed elsewhere, which is how the planner consumes what the
		costmap publishes. Inflation and the soft falloff are applied here rather than baked into the
		grid, so the clearance can follow the controller's reconfigurable corridor width without the
		costmap having to know anything about the vessel."""
		self.res = resolution
		self.origin_x = origin_x
		self.origin_y = origin_y
		self.size = dist.shape[0]
		self.inflate = inflate_radius
		self.soft = max(soft_radius, inflate_radius + resolution)
		self.dist = np.minimum(dist, self.soft)

		self.rasterise_corridor(corridor)

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

	def cost_cell(self, cx, cy):
		if not self.corridor_ok[cy, cx]:
			return LETHAL

		return self.soft_cost(self.dist[cy, cx])

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
		return 0.0 if self.in_corridor(x, y) else LETHAL

	def lethal_at(self, x, y):
		"""Anything the search may not enter, for either reason."""
		return self.cost_at(x, y) == LETHAL

	def obstacle_lethal_at(self, x, y):
		cell = self.world_to_cell(x, y)
		return self.dist[cell[1], cell[0]] <= self.inflate if cell else False

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
