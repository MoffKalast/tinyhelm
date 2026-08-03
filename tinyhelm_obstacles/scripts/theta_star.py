import math
import heapq

from cost_field import LETHAL
from utils import NEIGHBOURS

OK = "ok"
GOAL_IN_OBSTACLE = "goal_in_obstacle"
GOAL_OUTSIDE_CORRIDOR = "goal_outside_corridor"
START_TRAPPED = "start_trapped"
NO_ROUTE = "no_route"
UNREACHABLE_COARSE = "unreachable_coarse"

def integrated_cost(field, ax, ay, bx, by, step):
	min_x, max_x = min(ax, bx), max(ax, bx)
	min_y, max_y = min(ay, by), max(ay, by)
	field_max_x = field.origin_x + field.size * field.res
	field_max_y = field.origin_y + field.size * field.res

	if max_x < field.origin_x or min_x > field_max_x or max_y < field.origin_y or min_y > field_max_y:
		return math.hypot(bx - ax, by - ay)

	length = math.hypot(bx - ax, by - ay)
	samples = max(1, int(length / step))

	soft = 0.0
	for i in range(1, samples + 1):
		t = i / samples
		c = field.cost_at(ax + t * (bx - ax), ay + t * (by - ay))
		if c == LETHAL:
			return LETHAL
		soft += c

	return length * (1.0 + soft / samples)

def cumulative_cost(field, points, step):
	cumulative = [0.0]
	for i in range(1, len(points)):
		leg = integrated_cost(field, points[i - 1][0], points[i - 1][1], points[i][0], points[i][1], step)
		cumulative.append(LETHAL if leg == LETHAL else cumulative[-1] + leg)

	return cumulative

def smooth_path(field, points, step):
	#String pulling that respects the cost field. A shortcut is taken only when it is no more expensive than the stretch it replaces
	if len(points) <= 2:
		return list(points)

	cumulative = cumulative_cost(field, points, step)

	out = [points[0]]
	i = 0
	while i < len(points) - 1:
		j = len(points) - 1
		while j > i + 1:
			direct = integrated_cost(field, points[i][0], points[i][1], points[j][0], points[j][1], step)
			stretch = LETHAL if cumulative[i] == LETHAL else cumulative[j] - cumulative[i]
			if direct < LETHAL and direct <= stretch + 1e-9:
				break

			j -= 1

		out.append(points[j])
		i = j

	return out

class SearchGrid:

	def __init__(self, field, sx, sy, gx, gy, margin):
		self.field = field
		self.res = field.res

		self.origin_x = math.floor((min(sx, gx) - margin) / self.res) * self.res
		self.origin_y = math.floor((min(sy, gy) - margin) / self.res) * self.res

		self.cols = int(math.ceil((max(sx, gx) + margin - self.origin_x) / self.res)) + 1
		self.rows = int(math.ceil((max(sy, gy) + margin - self.origin_y) / self.res)) + 1

		self.cost_cache = {}

	def to_world(self, cell):
		return self.origin_x + (cell[0] + 0.5) * self.res, self.origin_y + (cell[1] + 0.5) * self.res

	def to_cell(self, x, y):
		return int(math.floor((x - self.origin_x) / self.res)), int(math.floor((y - self.origin_y) / self.res))

	def inside(self, cell):
		return 0 <= cell[0] < self.cols and 0 <= cell[1] < self.rows

	def cost(self, cell):
		c = self.cost_cache.get(cell)
		if c is None:
			x, y = self.to_world(cell)
			c = self.field.cost_at(x, y)
			self.cost_cache[cell] = c

		return c

	def lethal(self, cell):
		return self.cost(cell) == LETHAL

	def straight_length(self, a, b):
		return math.hypot(b[0] - a[0], b[1] - a[1]) * self.res

	def line_of_sight(self, a, b):
		steps = max(abs(b[0] - a[0]), abs(b[1] - a[1]))
		ax, ay = a
		dx, dy = b[0] - ax, b[1] - ay
		for i in range(1, steps):
			t = i / steps
			if self.lethal((int(ax + t * dx + 0.5), int(ay + t * dy + 0.5))):
				return False

		return True

	def segment_cost(self, a, b):
		ax, ay = a
		dx, dy = b[0] - ax, b[1] - ay
		steps = max(1, abs(dx), abs(dy))
		length = math.hypot(dx, dy) * self.res

		#Euclidean length plus the soft proximity cost integrated along the segment, sampled once per crossed cell. LETHAL if any sample is lethal.
		soft = 0.0
		for i in range(1, steps + 1):
			t = i / steps
			c = self.cost((int(ax + t * dx + 0.5), int(ay + t * dy + 0.5)))
			if c == LETHAL:
				return LETHAL
			soft += c

		return length * (1.0 + soft / steps)

	def nudge_free(self, cell, max_radius):
		for r in range(1, max_radius + 1):
			for oy in range(-r, r + 1):
				span = (-r, r) if abs(oy) != r else range(-r, r + 1)
				for ox in span:
					candidate = (cell[0] + ox, cell[1] + oy)
					if self.inside(candidate) and not self.lethal(candidate):
						return candidate
		return None

class ThetaStar:

	def __init__(self, nudge_radius=8):
		self.nudge_radius = nudge_radius

		self.last_expansions = 0
		self.reason = OK
		self.start_nudged = False

	def plan(self, field, sx, sy, gx, gy, margin, heuristic):
		self.last_expansions = 0
		self.reason = OK
		self.start_nudged = False

		grid = SearchGrid(field, sx, sy, gx, gy, margin)

		start = grid.to_cell(sx, sy)
		goal = grid.to_cell(gx, gy)

		if grid.lethal(goal):
			self.reason = GOAL_OUTSIDE_CORRIDOR if field.outside_corridor_at(gx, gy) else GOAL_IN_OBSTACLE
			return []

		if grid.lethal(start):
			nudged = grid.nudge_free(start, self.nudge_radius)
			if nudged is None:
				self.reason = START_TRAPPED
				return []
			start = nudged
			self.start_nudged = True

		start_x, start_y = grid.to_world(start)
		if heuristic.estimate(start_x, start_y) == LETHAL:
			self.reason = UNREACHABLE_COARSE
			return []

		came_from = self.search(grid, start, goal, heuristic)
		if came_from is None:
			self.reason = NO_ROUTE
			return []

		return self.reconstruct(grid, came_from, goal, sx, sy, gx, gy)

	def search(self, grid, start, goal, heuristic):
		g = {start: 0.0}
		parent = {start: start}
		closed = set()

		start_x, start_y = grid.to_world(start)
		open_heap = [(heuristic.estimate(start_x, start_y), 0.0, 0, start)]
		counter = 1
		expansions = 0

		while open_heap:
			current = heapq.heappop(open_heap)[3]
			if current in closed:
				continue

			closed.add(current)
			if current == goal:
				self.last_expansions = expansions
				return parent

			expansions += 1

			for ox, oy in NEIGHBOURS:
				neighbour = (current[0] + ox, current[1] + oy)
				if not grid.inside(neighbour) or neighbour in closed or grid.lethal(neighbour):
					continue

				cost = g[current] + grid.segment_cost(current, neighbour)
				via = current

				ancestor = parent[current]
				if ancestor != current and g[ancestor] + grid.straight_length(ancestor, neighbour) < cost:
					jump = g[ancestor] + grid.segment_cost(ancestor, neighbour)
					if jump < cost and grid.line_of_sight(ancestor, neighbour):
						cost = jump
						via = ancestor

				if cost >= g.get(neighbour, LETHAL):
					continue

				nx, ny = grid.to_world(neighbour)
				estimate = heuristic.estimate(nx, ny)
				if estimate == LETHAL:
					continue

				g[neighbour] = cost
				parent[neighbour] = via
				heapq.heappush(open_heap, (cost + estimate, -cost, counter, neighbour))
				counter += 1

		self.last_expansions = expansions
		return None

	def reconstruct(self, grid, parent, goal, sx, sy, gx, gy):
		points = []
		current = goal
		while True:
			points.append(grid.to_world(current))
			if parent[current] == current:
				break
			current = parent[current]

		points.reverse()

		if not self.start_nudged:
			points[0] = (sx, sy)
		points[-1] = (gx, gy)

		return points
