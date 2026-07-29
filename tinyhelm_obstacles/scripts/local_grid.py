import math
import numpy as np
from scipy import ndimage

class LocalGrid:
	"""One flat window of obstacle evidence around the vessel, scrolled as it moves. Information
	leaves at the trailing edge and is not recoverable, which is the accepted trade for never paying
	to maintain anything we have sailed away from.

	Evidence is measured in seconds observed rather than hits. A cell earns at most one credit per
	confirm_period no matter how many returns land in it, so a cell's credit is the length of time
	something has been sitting there, which is the only quantity that actually separates a piling
	from a workboat crossing our bow. Counting hits cannot: a ten metre hull at three knots puts as
	many returns into a cell as a jetty does, and no combination of increment and decay rate
	separates the two.

	Credit burns off only once observations have actually stopped, after grace_seconds of silence,
	so a structure glimpsed intermittently through swell and occlusion holds its evidence instead of
	losing it between sightings. Because credit is capped, how long anything survives after it stops
	being seen is bounded and proportional to how long it was really there.

	The distance field is maintained alongside the evidence rather than rebuilt, which is what keeps
	a 10Hz update off the critical path. Distances are clamped at soft_radius because
	nothing beyond it affects planning cost, and that clamp is also what makes the update local: a
	cell that changes can only influence distances within soft_radius of itself."""

	def __init__(self, resolution, size_cells, soft_radius, confirm_seconds=5.0, memory_seconds=15.0, grace_seconds=3.0, forget_ratio=2.0, confirm_period=1.0, scroll_hysteresis_cells=5):
		self.res = resolution
		self.size = int(size_cells)
		self.soft = soft_radius

		self.confirm = max(1, int(round(confirm_seconds / confirm_period)))
		self.memory = max(self.confirm + 1, int(round(memory_seconds / confirm_period)))
		self.grace = grace_seconds
		self.confirm_period = confirm_period

		# Seconds of silence needed to undo one second of observation. Held as a ratio rather than as
		# seconds per credit because credit is counted in confirm_period units: a fixed seconds per
		# credit silently stretches how long anything survives whenever confirm_period is shortened,
		# so tuning for a faster reaction would quietly buy a far longer smear.
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

		# Scratch for settle, which runs on every maintenance tick over the whole window and is the
		# only thing here that would otherwise allocate several grids a tick. Not reentrant, which
		# costs nothing: every caller already holds the map lock.
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
		"""Scrolls the window to follow the vessel, but only once it has moved several cells, so a
		vessel holding station or working back and forth over a boundary does not shed its trailing
		edge repeatedly. Returns True when the window actually moved."""
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

		# Rolling the distance field along with the evidence keeps it valid everywhere except the
		# strips that just came into view, which are refilled as unseen and therefore clear
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

		credit = self.credit.reshape(-1)
		last_seen = self.last_seen.reshape(-1)
		last_credit = self.last_credit.reshape(-1)

		# Fold in whatever has already been forgotten before adding to it, so a cell re-sighted after
		# a gap resumes from where it had decayed to rather than springing back
		credit[index] = self.settled_credit(index, now)

		ready = index[now - last_credit[index] >= self.confirm_period]
		if ready.size:
			credit[ready] = np.clip(credit[ready] + credit_delta, 0, self.memory)
			last_credit[ready] = now

		# Only a detection counts as a sighting. A free observation must not refresh last_seen, or it
		# would protect the very cell it is arguing against; leaving it alone lets ordinary staleness
		# run alongside the credit this already took off.
		if credit_delta > 0:
			last_seen[index] = now

		rows, cols = np.divmod(index, self.size)
		return int(cols.min()), int(rows.min()), int(cols.max()), int(rows.max())

	def observe_reliable(self, xs, ys, now):
		return self.observe(xs, ys, now, 1)

	def observe_free(self, xs, ys, now):
		return self.observe(xs, ys, now, -1)

	def observe_unreliable(self, xs, ys, now):
		"""Sustains evidence that already exists and lets it creep one cell outward, but cannot
		originate an obstacle on its own, so speckle is dropped instead of painted."""
		index = self.to_cells(xs, ys)
		if index.size == 0:
			return None

		keep = index[self.beside_occupied(index)]
		if keep.size == 0:
			return None

		rows, cols = np.divmod(keep, self.size)
		return self.observe(self.origin_x + (cols + 0.5) * self.res, self.origin_y + (rows + 0.5) * self.res, now, 1)

	def beside_occupied(self, index):
		"""Whether each given cell is itself occupied or touches an occupied cell edgewise. The same
		predicate a four connected dilation of the whole window answers, evaluated only where it is
		asked about. Neighbours off the edge of the window read as free, matching the zero border the
		dilation used; the clamping only keeps those reads in bounds and the mask discards them."""
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
		"""Materialises elapsed forgetting into stored credit. Returns the touched region, or None if
		nothing changed. Carrying the burned time onto last_seen rather than resetting it is what
		keeps this free of drift however often it is called."""
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

		# Masked so a cell blanked at the trailing edge, whose silence is nine orders of magnitude
		# larger than anything real, is never cast at all: unmasked it would overflow int16 and
		# resurrect the cell it had just cleared.
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
		"""Recomputes the clamped distance field over whatever has changed since the last call.

		Only obstacles within soft_radius can affect a cell, so the transform is run over the changed
		region grown by that much and written back to the changed region alone. Everything the window
		needs to answer correctly is inside that padding, which is what makes this exact rather than
		an approximation."""
		if force:
			self.dist = np.minimum(self.distance_of(self.occupied()), self.soft)
			self.pending = None
			return True

		if self.pending is None:
			return False

		col0, row0, col1, row1 = self.pending
		self.pending = None

		pad = int(math.ceil(self.soft / self.res)) + 1

		# A cell's distance changes if any obstacle within soft_radius of it changed, so the region
		# needing rewriting is the changed one grown by that much, not the changed one itself
		wc0 = max(0, col0 - pad)
		wr0 = max(0, row0 - pad)
		wc1 = min(self.size - 1, col1 + pad)
		wr1 = min(self.size - 1, row1 + pad)

		# And computing those correctly needs every obstacle within soft_radius of them in turn
		rc0 = max(0, wc0 - pad)
		rr0 = max(0, wr0 - pad)
		rc1 = min(self.size - 1, wc1 + pad)
		rr1 = min(self.size - 1, wr1 + pad)

		# A full sweep touches cells all round the vessel, so the padded window is most of the grid
		# anyway and the bookkeeping costs more than it saves
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
