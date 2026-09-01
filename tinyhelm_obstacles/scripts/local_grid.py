import math
import numpy as np
from scipy import ndimage

class LocalGrid:

	def __init__(self, resolution, size_cells, soft_radius, confirm_seconds=5.0, memory_seconds=15.0, grace_seconds=3.0, forget_ratio=2.0, confirm_period=1.0, scroll_hysteresis_cells=5):
		self.res = resolution
		self.size = int(size_cells)
		self.soft = soft_radius

		self.confirm = max(1, int(round(confirm_seconds / confirm_period)))
		self.memory = max(self.confirm + 1, int(round(memory_seconds / confirm_period)))
		self.grace = grace_seconds
		self.confirm_period = confirm_period

		self.forget = forget_ratio * confirm_period
		self.scroll_hysteresis = scroll_hysteresis_cells

		self.credit = np.zeros((self.size, self.size), dtype=np.int16)
		self.last_seen = np.full((self.size, self.size), -1e9, dtype=np.float64)
		self.last_credit = np.full((self.size, self.size), -1e9, dtype=np.float64)

		self.origin_x = 0.0
		self.origin_y = 0.0
		self.placed = False

		self.dist = np.full((self.size, self.size), self.soft, dtype=np.float32)
		self.pending = None

		self.elapsed = np.empty((self.size, self.size), dtype=np.float64)
		self.burned = np.empty((self.size, self.size), dtype=np.int16)
		self.active = np.empty((self.size, self.size), dtype=bool)
		self.remembered = np.empty((self.size, self.size), dtype=bool)
		self.active_rows = np.empty(self.size, dtype=bool)
		self.active_cols = np.empty(self.size, dtype=bool)

	def extent(self):
		return self.size * self.res

	def centre_origin(self, rx, ry):
		half = 0.5 * self.extent()
		return math.floor((rx - half) / self.res) * self.res, math.floor((ry - half) / self.res) * self.res

	def recentre(self, rx, ry):
		target_x, target_y = self.centre_origin(rx, ry)
		if not self.placed:
			self.origin_x, self.origin_y = target_x, target_y
			self.placed = True
			return True

		shift_x = int(round((target_x - self.origin_x) / self.res))
		shift_y = int(round((target_y - self.origin_y) / self.res))
		if max(abs(shift_x), abs(shift_y)) < self.scroll_hysteresis:
			return False

		if max(abs(shift_x), abs(shift_y)) >= self.size:
			self.clear()
			self.origin_x, self.origin_y = target_x, target_y
			self.placed = True
			return True

		self.credit = np.roll(self.credit, (-shift_y, -shift_x), axis=(0, 1))
		self.last_seen = np.roll(self.last_seen, (-shift_y, -shift_x), axis=(0, 1))
		self.last_credit = np.roll(self.last_credit, (-shift_y, -shift_x), axis=(0, 1))
		self.dist = np.roll(self.dist, (-shift_y, -shift_x), axis=(0, 1))

		self.blank_exposed(shift_x, shift_y)

		self.origin_x += shift_x * self.res
		self.origin_y += shift_y * self.res
		return True

	def blank_exposed(self, shift_x, shift_y):
		rows = slice(-shift_y, None) if shift_y > 0 else slice(None, -shift_y)
		cols = slice(-shift_x, None) if shift_x > 0 else slice(None, -shift_x)

		for region in ((rows, slice(None)) if shift_y else None, (slice(None), cols) if shift_x else None):
			if region is None:
				continue
			self.credit[region] = 0
			self.last_seen[region] = -1e9
			self.last_credit[region] = -1e9
			self.dist[region] = self.soft

	def clear(self):
		self.credit.fill(0)
		self.last_seen.fill(-1e9)
		self.last_credit.fill(-1e9)
		self.dist.fill(self.soft)
		self.pending = None

	def to_cells(self, xs, ys):
		"""World coordinates to unique in-bounds flat cell indices."""
		cols = np.floor((np.asarray(xs, dtype=np.float64) - self.origin_x) / self.res).astype(np.int64)
		rows = np.floor((np.asarray(ys, dtype=np.float64) - self.origin_y) / self.res).astype(np.int64)

		keep = (cols >= 0) & (cols < self.size) & (rows >= 0) & (rows < self.size)
		return np.unique(rows[keep] * self.size + cols[keep])

	def settled_credit(self, flat_index, now):
		"""Credit with elapsed forgetting already taken off, without touching stored state."""
		credit = self.credit.reshape(-1)[flat_index]
		silence = now - self.last_seen.reshape(-1)[flat_index] - self.grace
		burned = np.minimum(np.floor(np.maximum(0.0, silence) / self.forget), self.memory).astype(np.int16)
		return np.maximum(0, credit - burned)

	def observe(self, xs, ys, now, credit_delta):
		"""Applies one cloud. credit_delta of +1 is a detection, -1 a free space observation.
		Returns the touched region as (col0, row0, col1, row1) or None."""
		index = self.to_cells(xs, ys)
		if index.size == 0:
			return None

		if credit_delta < 0:
			index = index[now - self.last_seen.reshape(-1)[index] >= self.grace]
			if index.size == 0:
				return None

		credit = self.credit.reshape(-1)
		last_seen = self.last_seen.reshape(-1)
		last_credit = self.last_credit.reshape(-1)

		credit[index] = self.settled_credit(index, now)

		ready = index[now - last_credit[index] >= self.confirm_period]
		if ready.size:
			credit[ready] = np.clip(credit[ready] + credit_delta, 0, self.memory)
			last_credit[ready] = now

		if credit_delta > 0:
			last_seen[index] = now

		rows, cols = np.divmod(index, self.size)
		return int(cols.min()), int(rows.min()), int(cols.max()), int(rows.max())

	def observe_reliable(self, xs, ys, now):
		return self.observe(xs, ys, now, 1)

	def observe_free(self, xs, ys, now):
		return self.observe(xs, ys, now, -1)

	def observe_unreliable(self, xs, ys, now):
		index = self.to_cells(xs, ys)
		if index.size == 0:
			return None

		keep = index[self.beside_occupied(index)]
		if keep.size == 0:
			return None

		rows, cols = np.divmod(keep, self.size)
		return self.observe(self.origin_x + (cols + 0.5) * self.res, self.origin_y + (rows + 0.5) * self.res, now, 1)

	def beside_occupied(self, index):
		credit = self.credit.reshape(-1)
		rows, cols = np.divmod(index, self.size)
		last = self.size * self.size - 1

		near = credit[index] >= self.confirm
		near |= (cols > 0) & (credit[np.maximum(index - 1, 0)] >= self.confirm)
		near |= (cols < self.size - 1) & (credit[np.minimum(index + 1, last)] >= self.confirm)
		near |= (rows > 0) & (credit[np.maximum(index - self.size, 0)] >= self.confirm)
		near |= (rows < self.size - 1) & (credit[np.minimum(index + self.size, last)] >= self.confirm)

		return near

	def settle(self, now):
		steps = self.elapsed
		np.subtract(now, self.last_seen, out=steps)
		np.subtract(steps, self.grace, out=steps)
		np.maximum(steps, 0.0, out=steps)
		np.divide(steps, self.forget, out=steps)
		np.floor(steps, out=steps)

		np.greater(steps, 0.0, out=self.active)
		np.greater(self.credit, 0, out=self.remembered)
		np.logical_and(self.active, self.remembered, out=self.active)

		region = self.active_region()
		if region is None:
			return None

		np.copyto(self.burned, steps, where=self.active, casting="unsafe")
		np.subtract(self.credit, self.burned, out=self.credit, where=self.active)
		np.maximum(self.credit, 0, out=self.credit, where=self.active)

		np.multiply(steps, self.forget, out=steps)
		np.add(self.last_seen, steps, out=self.last_seen, where=self.active)

		return region

	def active_region(self):
		np.any(self.active, axis=1, out=self.active_rows)
		if not self.active_rows.any():
			return None

		np.any(self.active, axis=0, out=self.active_cols)

		row0 = int(np.argmax(self.active_rows))
		row1 = self.size - 1 - int(np.argmax(self.active_rows[::-1]))
		col0 = int(np.argmax(self.active_cols))
		col1 = self.size - 1 - int(np.argmax(self.active_cols[::-1]))

		return col0, row0, col1, row1

	def occupied(self):
		return self.credit >= self.confirm

	def mark(self, region):
		if region is None:
			return

		if self.pending is None:
			self.pending = region
			return

		self.pending = (min(self.pending[0], region[0]), min(self.pending[1], region[1]), max(self.pending[2], region[2]), max(self.pending[3], region[3]))

	def refresh_distance(self, force=False):
		if force:
			self.dist = np.minimum(self.distance_of(self.occupied()), self.soft)
			self.pending = None
			return True

		if self.pending is None:
			return False

		col0, row0, col1, row1 = self.pending
		self.pending = None

		pad = int(math.ceil(self.soft / self.res)) + 1

		wc0 = max(0, col0 - pad)
		wr0 = max(0, row0 - pad)
		wc1 = min(self.size - 1, col1 + pad)
		wr1 = min(self.size - 1, row1 + pad)

		rc0 = max(0, wc0 - pad)
		rr0 = max(0, wr0 - pad)
		rc1 = min(self.size - 1, wc1 + pad)
		rr1 = min(self.size - 1, wr1 + pad)

		if (rc1 - rc0 + 1) * (rr1 - rr0 + 1) > 0.5 * self.size * self.size:
			return self.refresh_distance(force=True)

		window = self.occupied()[rr0:rr1 + 1, rc0:rc1 + 1]
		local = np.minimum(self.distance_of(window), self.soft)

		self.dist[wr0:wr1 + 1, wc0:wc1 + 1] = local[wr0 - rr0:wr1 - rr0 + 1, wc0 - rc0:wc1 - rc0 + 1]
		return True

	def distance_of(self, occupied):
		if not occupied.any():
			return np.full(occupied.shape, self.soft, dtype=np.float32)

		return (ndimage.distance_transform_edt(~occupied) * self.res).astype(np.float32)