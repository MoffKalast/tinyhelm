import math
import numpy as np


class Chunk:
	def __init__(self, cells):
		self.values = np.zeros((cells, cells), dtype=np.int8)
		self.last_decay = 0.0
		self.last_hit = 0.0


class ChunkedGrid:
	"""Unbounded occupancy map stored as fixed-size square chunks in a dict, keyed by
	integer chunk coordinates. Cells hold hit evidence in [0, 127] that decays toward 0
	with a configurable half-life. Decay is lazy: each chunk stores its last decay time
	and catches up on access, so idle chunks cost nothing per tick. maintain() discards
	chunks that leave the load radius or whose data would have fully decayed anyway."""

	def __init__(self, resolution, chunk_size_m, hit_delta, occupied_threshold, half_life_s):
		self.res = resolution
		self.cells = max(1, int(round(chunk_size_m / resolution)))
		self.chunk_m = self.cells * self.res
		self.hit_delta = hit_delta
		self.remove_delta = math.ceil(hit_delta / 2)
		self.occ_thresh = occupied_threshold
		self.half_life = half_life_s
		self.chunks = {}

	def full_decay_time(self):
		return 2.0 * self.half_life

	def key(self, x, y):
		return (math.floor(x / self.chunk_m), math.floor(y / self.chunk_m))

	def cell_in_chunk(self, x, y, k):
		row = math.floor(y / self.res) - k[1] * self.cells
		col = math.floor(x / self.res) - k[0] * self.cells
		return row, col

	def remove_hit(self, x, y, now):
		k = self.key(x, y)
		chunk = self.chunks.get(k)
		if chunk is None:
			return

		self.decay_chunk(chunk, now)
		row, col = self.cell_in_chunk(x, y, k)
		chunk.values[row, col] = max(0, int(chunk.values[row, col]) - self.remove_delta)

	def add_hit(self, x, y, now):
		k = self.key(x, y)
		chunk = self.chunks.get(k)
		if chunk is None:
			chunk = Chunk(self.cells)
			self.chunks[k] = chunk

		self.decay_chunk(chunk, now)
		row, col = self.cell_in_chunk(x, y, k)
		chunk.values[row, col] = min(127, int(chunk.values[row, col]) + self.hit_delta)
		chunk.last_hit = now

	def add_unreliable_hit(self, x, y, now):
		"""Like add_hit but filters likely noise: only refreshes cells that are already
		occupied, or empty cells directly adjacent to an occupied one, so confirmed
		obstacles hold and expand slightly while stray noise is dropped."""
		if self.occupied_at(x, y):
			self.add_hit(x, y, now)
			return

		neighbors = ((x + self.res, y), (x - self.res, y), (x, y + self.res), (x, y - self.res))
		if any(self.occupied_at(nx, ny) for nx, ny in neighbors):
			self.add_hit(x, y, now)

	def value_at(self, x, y):
		chunk = self.chunks.get(self.key(x, y))
		if chunk is None:
			return 0
		row, col = self.cell_in_chunk(x, y, self.key(x, y))
		return int(chunk.values[row, col])

	def occupied_at(self, x, y):
		return self.value_at(x, y) >= self.occ_thresh

	def clear(self):
		self.chunks.clear()

	def maintain(self, rx, ry, load_radius, now):
		"""Decays loaded chunks and discards chunks that left the load radius or whose
		data would have fully decayed since the last hit. Call once per tick."""
		for k in list(self.chunks.keys()):
			chunk = self.chunks[k]
			if now - chunk.last_hit >= self.full_decay_time():
				del self.chunks[k]
			elif self.chunk_distance(k, rx, ry) > load_radius:
				del self.chunks[k]
			else:
				self.decay_chunk(chunk, now)

	def chunk_distance(self, k, rx, ry):
		cx = (k[0] + 0.5) * self.chunk_m
		cy = (k[1] + 0.5) * self.chunk_m
		return max(abs(cx - rx), abs(cy - ry)) - 0.5 * self.chunk_m

	def decay_chunk(self, chunk, now):
		if self.half_life <= 0.0:
			return
		if chunk.last_decay == 0.0:
			chunk.last_decay = now
			return

		# Linear pull toward 0, scaled so a saturated cell empties in ~2 half-lives
		per_step = self.full_decay_time() / 127.0
		steps = int((now - chunk.last_decay) / per_step)
		if steps < 1:
			return
		chunk.last_decay += steps * per_step
		chunk.values = np.maximum(chunk.values.astype(np.int16) - steps, 0).astype(np.int8)

	def window(self, ox, oy, size):
		"""Raw cell values over a size x size window whose lower-left corner is at
		(ox, oy), assembled by slicing the overlapping chunks. ox and oy must be
		multiples of the resolution. Returned array is indexed [row, col] = [y, x]."""
		out = np.zeros((size, size), dtype=np.int8)
		gx0 = int(round(ox / self.res))
		gy0 = int(round(oy / self.res))

		for (kx, ky), chunk in self.chunks.items():
			cx0 = kx * self.cells
			cy0 = ky * self.cells
			x0 = max(gx0, cx0)
			x1 = min(gx0 + size, cx0 + self.cells)
			y0 = max(gy0, cy0)
			y1 = min(gy0 + size, cy0 + self.cells)
			if x0 >= x1 or y0 >= y1:
				continue
			out[y0 - gy0:y1 - gy0, x0 - gx0:x1 - gx0] = chunk.values[y0 - cy0:y1 - cy0, x0 - cx0:x1 - cx0]

		return out