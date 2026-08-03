import math
import heapq
import numpy as np

from cost_field import shortfall, soft_penalty
from utils import NEIGHBOURS

OCTILE_OVERSHOOT = 1.0 + math.sqrt(2.0) - 2.0 / math.sqrt(2.0) + 1e-9

class CoarseHeuristic:

	def __init__(self, field, gx, gy, factor=4):
		self.factor = factor
		self.res = field.res * factor
		self.origin_x = field.origin_x
		self.origin_y = field.origin_y
		self.goal_x = gx
		self.goal_y = gy

		blocked, soft = self.downsample(field)
		self.rows, self.cols = blocked.shape

		goal = self.to_cell(gx, gy)
		self.goal_inside = self.inside(goal)

		self.cost_to_goal = self.dijkstra(blocked, soft, goal) if self.goal_inside else {}
		self.trust_unreachable = self.goal_inside and bool(self.cost_to_goal) and not self.reached_edge()

	def downsample(self, field):
		occupied = field.dist <= field.inflate
		n = occupied.shape[0] // self.factor
		usable = self.factor * n

		blocks = occupied[:usable, :usable].reshape(n, self.factor, n, self.factor)
		blocked = blocks.all(axis=(1, 3))

		graded = np.clip(shortfall(field.dist[:usable, :usable], field.inflate, field.soft), 0.0, 1.0)
		penalty = np.where(occupied[:usable, :usable], np.inf, soft_penalty(graded))
		soft = penalty.reshape(n, self.factor, n, self.factor).min(axis=(1, 3))

		inside = field.corridor_ok[:usable, :usable].reshape(n, self.factor, n, self.factor).any(axis=(1, 3))
		blocked |= ~inside

		return blocked, soft

	def reached_edge(self):
		last_col = self.cols - 1
		last_row = self.rows - 1
		return any(cx == 0 or cy == 0 or cx == last_col or cy == last_row for cx, cy in self.cost_to_goal)

	def to_cell(self, x, y):
		return int(math.floor((x - self.origin_x) / self.res)), int(math.floor((y - self.origin_y) / self.res))

	def inside(self, cell):
		return 0 <= cell[0] < self.cols and 0 <= cell[1] < self.rows

	def dijkstra(self, blocked, soft, goal):
		if blocked[goal[1], goal[0]]:
			return {}

		cost = {}
		queue = [(0.0, goal)]
		while queue:
			distance, cell = heapq.heappop(queue)
			if cell in cost:
				continue

			cost[cell] = distance
			here = soft[cell[1], cell[0]]
			for ox, oy in NEIGHBOURS:
				neighbour = (cell[0] + ox, cell[1] + oy)
				if not self.inside(neighbour):
					continue
				if neighbour in cost or blocked[neighbour[1], neighbour[0]]:
					continue

				cheapest = min(here, soft[neighbour[1], neighbour[0]])
				step = math.hypot(ox, oy) * self.res * (1.0 + cheapest)
				heapq.heappush(queue, (distance + step, neighbour))

		return cost

	def estimate(self, x, y):
		cell = self.to_cell(x, y)
		if self.inside(cell):
			distance = self.cost_to_goal.get(cell)
			if distance is not None:
				return distance / OCTILE_OVERSHOOT
			if self.trust_unreachable:
				return float("inf")

		return math.hypot(self.goal_x - x, self.goal_y - y)
