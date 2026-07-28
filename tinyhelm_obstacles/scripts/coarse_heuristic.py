import math
import heapq
import numpy as np

from cost_field import SOFT_WEIGHT

NEIGHBOURS = ((-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1))

# An eight connected grid can only approximate a straight line, overestimating it by at most this
# factor, so dividing by it keeps the estimate admissible
OCTILE_OVERSHOOT = 1.0 + math.sqrt(2.0) - 2.0 / math.sqrt(2.0) + 1e-9

class CoarseHeuristic:
	"""Cost to the goal precomputed by Dijkstra over a downsampled copy of the window, used as the
	heuristic for the full resolution search. Plain euclidean distance is a very weak bound here
	because segment cost carries the soft proximity penalty on top of length, so the fine search ends
	up expanding most of the corridor; this gives it something informed to aim at.

	Everything about the coarse layer errs optimistic, because a heuristic that overestimates makes
	the search wrong rather than slow:

	  - a coarse cell counts as blocked only when every fine cell inside it is blocked, so a row of
	    marina piles stays passable instead of merging into a wall
	  - clearance is not applied here at all: it is already in the field this is built from
	  - only geometric length and the cheapest soft penalty in each cell are accumulated
	  - the octile overshoot is divided back out

	The layer only covers the costmap window, and the search deliberately does not stop there: unseen
	space beyond the window is treated as clear everywhere else in the stack, and a leg longer than
	the window has its goal outside it altogether. So anything the coarse grid holds no cell for falls
	back to the straight line rather than being called unreachable, and an unreachable verdict is only
	trusted when the corridor lies wholly inside the window."""

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

		# Seeding Dijkstra with a goal outside the grid indexed straight off the end of the array,
		# which only happens once a leg reaches past the edge of the window
		self.cost_to_goal = self.dijkstra(blocked, soft, goal) if self.goal_inside else {}

		# A cell the optimistic layer cannot reach really is unreachable, but only if nothing could
		# reach the goal from outside the window either. Asking whether the corridor touches the edge
		# is far too blunt for that: a fifty metre leg with a twenty metre tube already touches the
		# edge of a modest window. What matters is whether the goal's own reachable set does, since a
		# route can only arrive from outside the window by entering that set through the border.
		#
		# The set has to be non-empty to be worth trusting. Dijkstra returns nothing at all when the
		# goal's own coarse cell is blocked, and an empty set reaches no edge, so without this a layer
		# that knows nothing would answer unreachable for every cell in the window and every request
		# against it would die before searching.
		self.trust_unreachable = self.goal_inside and bool(self.cost_to_goal) and not self.reached_edge()

	def downsample(self, field):
		occupied = field.dist <= field.inflate
		n = occupied.shape[0] // self.factor
		usable = self.factor * n

		blocks = occupied[:usable, :usable].reshape(n, self.factor, n, self.factor)

		# all() rather than any(): a coarse cell is only impassable when nothing inside it is free
		blocked = blocks.all(axis=(1, 3))

		# Nothing is dilated here. occupied is already the clearance-inflated lethal set, the same
		# predicate soft_cost returns LETHAL for, so growing it again by field.inflate would charge the
		# vessel its clearance twice. At a coarse cell no smaller than the clearance that is a whole
		# extra ring, which is enough to seal the vessel off from the goal's reachable set while there
		# is still open water either side of the obstacle at full resolution, and the fine search would
		# then be told the leg is impossible without ever expanding a node.
		#
		# Cheapest soft penalty available anywhere inside each coarse cell. Taking the minimum keeps
		# the estimate below whatever a fine path through there would really pay, while still telling
		# the search that threading a narrow gap costs more than open water. Without this the estimate
		# ignores the soft penalty entirely and stays a factor of three too low.
		shortfall = np.clip((field.soft - field.dist[:usable, :usable]) / (field.soft - field.inflate), 0.0, 1.0)
		penalty = np.where(occupied[:usable, :usable], np.inf, SOFT_WEIGHT * shortfall * shortfall)
		soft = penalty.reshape(n, self.factor, n, self.factor).min(axis=(1, 3))

		# The fine search cannot leave the corridor either, so flooding the whole window is wasted
		# work. any() again keeps it optimistic: a coarse cell counts as inside if any part of it is.
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
