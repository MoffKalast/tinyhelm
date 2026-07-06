#pragma once

#include <vector>
#include <queue>
#include <cmath>
#include "cost_field.h"

namespace tinyhelm {

struct PlanPoint { double x, y; };

// Any-angle Theta* over a CostField. Standard A* expansion on the 8-connected grid, but each
// expanded neighbour first tries to connect straight to its parent's parent if line of sight
// is free, which yields taut, any-angle paths. Segment cost = euclidean length plus the soft
// obstacle-proximity cost integrated along the segment, so paths keep standoff when they can.
class ThetaStar {
public:
	// Returns an empty path on failure. Start and goal are snapped inside the field; the goal
	// is rejected as unreachable if it lies in lethal space.
	std::vector<PlanPoint> plan(const CostField& field, double sx, double sy, double gx, double gy) {
		int scx, scy, gcx, gcy;
		if (!field.worldToCell(sx, sy, scx, scy) || !field.worldToCell(gx, gy, gcx, gcy)) return {};
		if (field.lethal(gcx, gcy)) return {};
		if (field.lethal(scx, scy) && !nudgeFree(field, scx, scy)) return {};

		int n = field.size();
		std::vector<float> g(n * n, std::numeric_limits<float>::max());
		std::vector<int> parent(n * n, -1);
		std::vector<uint8_t> closed(n * n, 0);
		auto idx = [n](int x, int y) { return y * n + x; };

		using QE = std::pair<float, int>;
		std::priority_queue<QE, std::vector<QE>, std::greater<QE>> open;
		int start = idx(scx, scy), goal = idx(gcx, gcy);
		g[start] = 0.0f;
		parent[start] = start;
		open.push({heuristic(field, scx, scy, gcx, gcy), start});

		while (!open.empty()) {
			int cur = open.top().second;
			open.pop();
			if (closed[cur]) continue;
			closed[cur] = 1;
			if (cur == goal) break;
			int cx = cur % n, cy = cur / n;

			for (int oy = -1; oy <= 1; oy++) {
				for (int ox = -1; ox <= 1; ox++) {
					if (ox == 0 && oy == 0) continue;
					int nx = cx + ox, ny = cy + oy;
					if (!field.inBounds(nx, ny) || closed[idx(nx, ny)] || field.lethal(nx, ny)) continue;
					int nb = idx(nx, ny);

					int par = parent[cur];
					float ng;
					int npar;
					float via_parent = g[par] + segmentCost(field, par % n, par / n, nx, ny);
					if (par != cur && via_parent < std::numeric_limits<float>::max() && lineOfSight(field, par % n, par / n, nx, ny)) {
						ng = via_parent;
						npar = par;
					} else {
						ng = g[cur] + segmentCost(field, cx, cy, nx, ny);
						npar = cur;
					}

					if (ng < g[nb]) {
						g[nb] = ng;
						parent[nb] = npar;
						open.push({ng + heuristic(field, nx, ny, gcx, gcy), nb});
					}
				}
			}
		}

		if (parent[goal] < 0) return {};

		std::vector<PlanPoint> path;
		for (int cur = goal; ; cur = parent[cur]) {
			double wx, wy;
			field.cellToWorld(cur % n, cur / n, wx, wy);
			path.push_back({wx, wy});
			if (parent[cur] == cur) break;
		}
		std::reverse(path.begin(), path.end());
		path.front() = {sx, sy};
		path.back() = {gx, gy};
		return path;
	}

	static double pathLength(const std::vector<PlanPoint>& path) {
		double len = 0.0;
		for (size_t i = 1; i < path.size(); i++) len += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
		return len;
	}

private:
	float heuristic(const CostField& f, int x1, int y1, int x2, int y2) const {
		return (float)(std::hypot(x2 - x1, y2 - y1) * f.resolution());
	}

	bool lineOfSight(const CostField& f, int x1, int y1, int x2, int y2) const {
		int steps = std::max(std::abs(x2 - x1), std::abs(y2 - y1));
		for (int i = 1; i < steps; i++) {
			double t = (double)i / steps;
			int x = (int)std::lround(x1 + t * (x2 - x1));
			int y = (int)std::lround(y1 + t * (y2 - y1));
			if (f.lethal(x, y)) return false;
		}
		return true;
	}

	float segmentCost(const CostField& f, int x1, int y1, int x2, int y2) const {
		double dx = (x2 - x1) * f.resolution(), dy = (y2 - y1) * f.resolution();
		double len = std::hypot(dx, dy);
		int steps = std::max(1, std::max(std::abs(x2 - x1), std::abs(y2 - y1)));
		float soft = 0.0f;
		for (int i = 1; i <= steps; i++) {
			double t = (double)i / steps;
			int x = (int)std::lround(x1 + t * (x2 - x1));
			int y = (int)std::lround(y1 + t * (y2 - y1));
			float c = f.cost(x, y);
			if (c == CostField::LETHAL) return std::numeric_limits<float>::max();
			soft += c;
		}
		return (float)len + soft * (float)(len / steps);
	}

	// Sensor noise can momentarily paint the vessel's own cell; escape to the nearest free cell
	bool nudgeFree(const CostField& f, int& cx, int& cy) const {
		for (int r = 1; r <= 8; r++) {
			for (int oy = -r; oy <= r; oy++) {
				for (int ox = -r; ox <= r; ox++) {
					if (std::max(std::abs(ox), std::abs(oy)) != r) continue;
					if (f.inBounds(cx + ox, cy + oy) && !f.lethal(cx + ox, cy + oy)) {
						cx += ox;
						cy += oy;
						return true;
					}
				}
			}
		}
		return false;
	}
};

}
