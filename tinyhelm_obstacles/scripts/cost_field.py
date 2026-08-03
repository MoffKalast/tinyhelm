import math
import numpy as np

from utils import segment_distance

LETHAL = float("inf")
SOFT_WEIGHT = 2.0
DIVERGENCE_WEIGHT = 1.6

def shortfall(distance, inflate_radius, soft_radius):
	return (soft_radius - distance) / (soft_radius - inflate_radius)

def soft_penalty(shortfall_value):
	return SOFT_WEIGHT * shortfall_value * shortfall_value

class Capsule:

	def __init__(self, x1, y1, x2, y2, radius):
		self.x1 = x1
		self.y1 = y1
		self.x2 = x2
		self.y2 = y2
		self.radius = radius

	def distance(self, px, py):
		return segment_distance(px, py, self.x1, self.y1, self.x2, self.y2)

	def contains(self, px, py):
		return self.distance(px, py) <= self.radius

def corridor_from_polyline(points, radius):
	"""One capsule per leg of the polyline. Both the planner and the marker overlay derive the tube
	from the same polyline this way, so what is drawn is what the search was actually held to."""
	return [Capsule(points[i - 1][0], points[i - 1][1], points[i][0], points[i][1], radius) for i in range(1, len(points))]

class CostField:

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
		self.res = resolution
		self.origin_x = origin_x
		self.origin_y = origin_y
		self.size = dist.shape[0]
		self.inflate = inflate_radius

		self.soft = max(soft_radius, inflate_radius + resolution)
		self.dist = np.minimum(dist, self.soft)

		self.rasterise_corridor(corridor)
		self.adopt_centreline(centreline, divergence_radius)

	def adopt_centreline(self, centreline, divergence_radius):

		self.centreline = list(centreline) if centreline and len(centreline) >= 2 else None
		self.divergence_radius = divergence_radius

		if self.centreline and divergence_radius > 0.0:
			self.divergence = np.full((self.size, self.size), -1.0, dtype=np.float32)
		else:
			self.divergence = None

	def rasterise_corridor(self, corridor):
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
		if distance <= self.inflate:
			return LETHAL
		if distance >= self.soft:
			return 0.0

		return soft_penalty(shortfall(distance, self.inflate, self.soft))

	def deviation_at(self, x, y):
		return min(segment_distance(x, y, self.centreline[i - 1][0], self.centreline[i - 1][1], self.centreline[i][0], self.centreline[i][1]) for i in range(1, len(self.centreline)))

	def divergence_cost(self, x, y):
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

		soft = self.soft_cost(self.dist[cy, cx])
		if soft == LETHAL:
			return LETHAL

		return soft + self.divergence_cell(cx, cy)

	def in_corridor(self, x, y):
		if not self.corridor:
			return True

		return any(capsule.contains(x, y) for capsule in self.corridor)

	def cost_at(self, x, y):
		cell = self.world_to_cell(x, y)
		if cell:
			return self.cost_cell(*cell)

		if not self.in_corridor(x, y):
			return LETHAL

		return self.divergence_cost(x, y)

	def outside_corridor_at(self, x, y):
		cell = self.world_to_cell(x, y)
		if cell:
			return not self.corridor_ok[cell[1], cell[0]]

		return not self.in_corridor(x, y)

	def obstacle_distance_at(self, x, y):
		cell = self.world_to_cell(x, y)
		return self.dist[cell[1], cell[0]] if cell else LETHAL

	def distance_gradient(self, x, y):
		east = self.obstacle_distance_at(x + self.res, y)
		west = self.obstacle_distance_at(x - self.res, y)
		north = self.obstacle_distance_at(x, y + self.res)
		south = self.obstacle_distance_at(x, y - self.res)
		if LETHAL in (east, west, north, south):
			return None

		dx = east - west
		dy = north - south
		length = math.hypot(dx, dy)
		if length <= 0.0:
			return None

		return float(dx / length), float(dy / length)

def nudge_legal(field, x, y):
	return field.cost_at(x, y) != LETHAL and field.obstacle_distance_at(x, y) > field.inflate + field.res

def nudge_direction(field, gx, gy, toward_x, toward_y):
	vx = toward_x - gx
	vy = toward_y - gy
	length = math.hypot(vx, vy)
	if length <= 0.0:
		return 1.0, 0.0

	vx /= length
	vy /= length

	gradient = field.distance_gradient(gx, gy)
	if gradient is None or gradient[0] * vx + gradient[1] * vy <= 0.0:
		return vx, vy

	return gradient

def nudge_nearest(field, gx, gy, toward_x, toward_y, max_distance):
	cell = field.world_to_cell(gx, gy)
	if cell is None:
		return None

	best = None
	best_distance = 0.0
	for radius in range(1, int(max_distance / field.res) + 1):
		for ox in range(-radius, radius + 1):
			for oy in range(-radius, radius + 1):
				if max(abs(ox), abs(oy)) != radius:
					continue

				x, y = field.cell_to_world(cell[0] + ox, cell[1] + oy)
				if math.hypot(x - gx, y - gy) > max_distance or not nudge_legal(field, x, y):
					continue

				distance = math.hypot(x - toward_x, y - toward_y)
				if best is None or distance < best_distance:
					best = (x, y)
					best_distance = distance

		if best is not None:
			return best

	return None

def nudge_goal(field, gx, gy, toward_x, toward_y, max_distance):
	if field.cost_at(gx, gy) != LETHAL:
		return None

	direction = nudge_direction(field, gx, gy, toward_x, toward_y)
	step = 0.5 * field.res

	for i in range(1, int(max_distance / step) + 1):
		x = gx + direction[0] * i * step
		y = gy + direction[1] * i * step
		if nudge_legal(field, x, y):
			return x, y

	return nudge_nearest(field, gx, gy, toward_x, toward_y, max_distance)

COST_LETHAL = 100

def encode_cost(dist, soft_radius):
	soft = max(soft_radius, 1e-6)
	graded = np.clip(shortfall(dist, 0.0, soft), 0.0, 1.0)
	return np.where(dist <= 0.0, COST_LETHAL, np.rint(graded * graded * (COST_LETHAL - 1))).astype(np.int8)

def distance_quantum(distance, soft_radius):
	soft = max(soft_radius, 1e-6)
	graded = min(1.0, max(0.0, shortfall(distance, 0.0, soft)))
	if graded <= 0.0:
		return soft

	return soft / (2.0 * graded * (COST_LETHAL - 1))

def decode_distance(values, soft_radius):
	soft = max(soft_radius, 1e-6)
	graded = np.asarray(values, dtype=np.float64)
	recovered = np.sqrt(np.clip(graded / (COST_LETHAL - 1), 0.0, 1.0))
	distance = soft - recovered * soft

	margin = 0.5 * soft / (COST_LETHAL - 1)
	distance = np.maximum(distance, margin)

	return np.where(graded >= COST_LETHAL, 0.0, distance)
