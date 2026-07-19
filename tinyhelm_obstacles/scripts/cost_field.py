import math
import numpy as np
from scipy import ndimage

LETHAL = float("inf")


class Capsule:
	"""Line segment with a radius, used as one link of the geofence tube around the
	strategic legs. distance() works on scalars and numpy arrays alike."""

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


class CostField:
	"""Snapshot of the obstacle grid window turned into planning costs: a euclidean
	distance-to-obstacle field gives hard inflation (lethal within inflate_radius) and
	a soft falloff out to soft_radius, and an optional geofence (union of capsules
	around the strategic legs) makes everything outside the allowed tube lethal too.
	All world-space queries treat anything outside the field extent as free, which is
	what lets the planner treat unseen space beyond the loaded window as clear."""

	def __init__(self):
		self.res = 1.0
		self.origin_x = 0.0
		self.origin_y = 0.0
		self.size = 0
		self.inflate = 0.0
		self.soft = 0.0
		self.soft_weight = 0.0
		self.dist = None
		self.fence_ok = None

	def build(self, resolution, origin_x, origin_y, occupied, inflate_radius, soft_radius, soft_weight, geofence):
		"""occupied is a [row, col] = [y, x] boolean array; the field is square."""
		self.res = resolution
		self.origin_x = origin_x
		self.origin_y = origin_y
		self.size = occupied.shape[0]
		self.inflate = inflate_radius
		self.soft = max(soft_radius, inflate_radius)
		self.soft_weight = soft_weight

		self.dist = ndimage.distance_transform_edt(~occupied) * resolution

		if not geofence:
			self.fence_ok = np.ones((self.size, self.size), dtype=bool)
			return

		# Rasterize the fence by stamping each capsule's bounding box, so cost lookups stay O(1)
		self.fence_ok = np.zeros((self.size, self.size), dtype=bool)
		for c in geofence:
			x0, y0 = self.bound_cell(min(c.x1, c.x2) - c.radius, min(c.y1, c.y2) - c.radius)
			x1, y1 = self.bound_cell(max(c.x1, c.x2) + c.radius, max(c.y1, c.y2) + c.radius)
			cols, rows = np.meshgrid(np.arange(x0, x1 + 1), np.arange(y0, y1 + 1))
			wx = self.origin_x + (cols + 0.5) * self.res
			wy = self.origin_y + (rows + 0.5) * self.res
			self.fence_ok[y0:y1 + 1, x0:x1 + 1] |= c.contains(wx, wy)

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

	def cost_cell(self, cx, cy):
		if not self.fence_ok[cy, cx]:
			return LETHAL
		d = self.dist[cy, cx]
		if d <= self.inflate:
			return LETHAL
		if d >= self.soft:
			return 0.0
		return self.soft_weight * (self.soft - d) / (self.soft - self.inflate)

	def cost_at(self, x, y):
		cell = self.world_to_cell(x, y)
		return self.cost_cell(*cell) if cell else 0.0

	def lethal_at(self, x, y):
		return self.cost_at(x, y) == LETHAL

	def obstacle_distance_at(self, x, y):
		cell = self.world_to_cell(x, y)
		return self.dist[cell[1], cell[0]] if cell else LETHAL
