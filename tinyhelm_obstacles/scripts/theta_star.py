import math
import heapq
from cost_field import LETHAL

NEIGHBOURS = [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)]

def path_length(points):
	return sum(math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1]) for i in range(1, len(points)))

def line_clear(field, a, b, step):
	"""True when no sample strictly between a and b is lethal, sampling every `step` metres."""
	length = math.hypot(b[0] - a[0], b[1] - a[1])
	steps = max(1, int(length / step))
	for s in range(1, steps):
		t = s / steps
		if field.lethal_at(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])):
			return False
	return True

def smooth_leg(field, leg, step):
	"""Theta* style string pulling: greedily link each point to the furthest point it has
	line of sight to, dropping everything in between, leaving one taut segment chain."""
	if len(leg) <= 2:
		return leg

	out = [leg[0]]
	i = 0
	while i < len(leg) - 1:
		j = len(leg) - 1
		while j > i + 1 and not line_clear(field, leg[i], leg[j], step):
			j -= 1
		out.append(leg[j])
		i = j
	return out

class ThetaStar:
	"""Any-angle Theta* on a virtual grid spanning the bounding box of start and goal plus
	a margin, at the cost field's resolution. Cost and lethality lookups delegate to the
	field's world-space queries, which return free outside the field extent, so planning
	is not clipped at the loaded window: unseen space is optimistically clear. Standard
	A* expansion on the 8-connected grid, but each expanded neighbour first tries to
	connect straight to its parent's parent if line of sight is free, which yields taut,
	any-angle paths. Segment cost = euclidean length plus the soft obstacle-proximity
	cost integrated along the segment, so paths keep standoff when they can. Sparse dicts
	instead of dense arrays keep the virtual grid free until actually explored."""

	# The heuristic is inflated a hair to break the equal-cost plateaus of free space,
	# which would otherwise make python-side A* crawl on long unobstructed legs
	def __init__(self, expansion_limit=100000, heuristic_weight=1.001):
		self.expansion_limit = expansion_limit
		self.heuristic_weight = heuristic_weight

	def plan(self, field, sx, sy, gx, gy, margin):
		"""Returns a list of (x, y) tuples, empty on failure. The start is nudged out of
		lethal space if sensor noise painted the vessel's own cell; a lethal goal fails."""
		res = field.res
		origin_x = math.floor((min(sx, gx) - margin) / res) * res
		origin_y = math.floor((min(sy, gy) - margin) / res) * res
		cols = int(math.ceil((max(sx, gx) + margin - origin_x) / res)) + 1
		rows = int(math.ceil((max(sy, gy) + margin - origin_y) / res)) + 1

		def to_world(cell):
			return origin_x + (cell[0] + 0.5) * res, origin_y + (cell[1] + 0.5) * res

		# All lookups quantize to the same virtual cells (both origins are snapped to
		# resolution multiples), so a per-cell memo is exact and collapses the millions
		# of repeated field queries a search makes into one per distinct cell
		cost_cache = {}

		def cell_cost(cell):
			c = cost_cache.get(cell)
			if c is None:
				c = field.cost_at(*to_world(cell))
				cost_cache[cell] = c
			return c

		def lethal(cell):
			return cell_cost(cell) == LETHAL

		start = (math.floor((sx - origin_x) / res), math.floor((sy - origin_y) / res))
		goal = (math.floor((gx - origin_x) / res), math.floor((gy - origin_y) / res))
		if lethal(goal):
			return []
		if lethal(start):
			start = self.nudge_free(lethal, start)
			if start is None:
				return []

		def heuristic(cell):
			return math.hypot(goal[0] - cell[0], goal[1] - cell[1]) * res * self.heuristic_weight

		g = {start: 0.0}
		parent = {start: start}
		closed = set()
		open_heap = [(heuristic(start), 0, start)]
		tiebreak = 1
		expansions = 0

		while open_heap:
			cur = heapq.heappop(open_heap)[2]
			if cur in closed:
				continue
			closed.add(cur)
			if cur == goal:
				break

			expansions += 1
			if expansions > self.expansion_limit:
				return []

			for ox, oy in NEIGHBOURS:
				nb = (cur[0] + ox, cur[1] + oy)
				if nb[0] < 0 or nb[1] < 0 or nb[0] >= cols or nb[1] >= rows:
					continue
				if nb in closed or lethal(nb):
					continue

				par = parent[cur]
				via_parent = g[par] + self.segment_cost(cell_cost, res, par, nb)
				if par != cur and via_parent < LETHAL and self.cell_line_of_sight(lethal, par, nb):
					ng = via_parent
					npar = par
				else:
					ng = g[cur] + self.segment_cost(cell_cost, res, cur, nb)
					npar = cur

				if ng < g.get(nb, LETHAL):
					g[nb] = ng
					parent[nb] = npar
					heapq.heappush(open_heap, (ng + heuristic(nb), tiebreak, nb))
					tiebreak += 1

		if goal not in parent:
			return []

		path = []
		cur = goal
		while True:
			path.append(to_world(cur))
			if parent[cur] == cur:
				break
			cur = parent[cur]
		path.reverse()
		path[0] = (sx, sy)
		path[-1] = (gx, gy)
		return path

	def cell_line_of_sight(self, lethal, a, b):
		steps = max(abs(b[0] - a[0]), abs(b[1] - a[1]))
		for i in range(1, steps):
			t = i / steps
			cell = (round(a[0] + t * (b[0] - a[0])), round(a[1] + t * (b[1] - a[1])))
			if lethal(cell):
				return False
		return True

	def segment_cost(self, cell_cost, res, a, b):
		"""Euclidean length plus the soft proximity cost integrated along the segment,
		sampled per crossed cell like the C++ version; LETHAL if any sample is lethal."""
		steps = max(1, abs(b[0] - a[0]), abs(b[1] - a[1]))
		length = math.hypot(b[0] - a[0], b[1] - a[1]) * res

		soft = 0.0
		for i in range(1, steps + 1):
			t = i / steps
			c = cell_cost((round(a[0] + t * (b[0] - a[0])), round(a[1] + t * (b[1] - a[1]))))
			if c == LETHAL:
				return LETHAL
			soft += c

		return length + soft * (length / steps)

	# Sensor noise can momentarily paint the vessel's own cell; escape to the nearest free cell
	def nudge_free(self, lethal, cell):
		for r in range(1, 9):
			for oy in range(-r, r + 1):
				for ox in range(-r, r + 1):
					if max(abs(ox), abs(oy)) != r:
						continue
					candidate = (cell[0] + ox, cell[1] + oy)
					if not lethal(candidate):
						return candidate
		return None
