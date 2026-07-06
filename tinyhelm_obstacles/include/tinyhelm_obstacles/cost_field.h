#pragma once

#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include "decay_grid.h"

namespace tinyhelm {

struct Capsule {
	double x1, y1, x2, y2, radius;

	double distance(double px, double py) const {
		double dx = x2 - x1, dy = y2 - y1;
		double len2 = dx * dx + dy * dy;
		double t = len2 > 0.0 ? std::max(0.0, std::min(1.0, ((px - x1) * dx + (py - y1) * dy) / len2)) : 0.0;
		double cx = x1 + t * dx, cy = y1 + t * dy;
		return std::hypot(px - cx, py - cy);
	}

	bool contains(double px, double py) const { return distance(px, py) <= radius; }
};

// Snapshot of a DecayGrid turned into planning costs: a BFS distance-to-obstacle field
// gives hard inflation (lethal within inflate_radius) and a soft falloff out to soft_radius,
// and an optional geofence (union of capsules around the strategic legs) makes everything
// outside the allowed tube lethal too.
class CostField {
public:
	static constexpr float LETHAL = std::numeric_limits<float>::infinity();

	void build(const DecayGrid& grid, double inflate_radius, double soft_radius, double soft_weight, const std::vector<Capsule>& geofence) {
		res_ = grid.resolution();
		size_ = grid.size();
		origin_x_ = grid.originX();
		origin_y_ = grid.originY();
		dist_.assign(size_ * size_, std::numeric_limits<float>::max());
		fence_ok_.assign(size_ * size_, geofence.empty() ? 1 : 0);
		inflate_ = inflate_radius;
		soft_ = std::max(soft_radius, inflate_radius);
		soft_weight_ = soft_weight;

		std::queue<int> frontier;
		for (int y = 0; y < size_; y++) {
			for (int x = 0; x < size_; x++) {
				if (grid.occupied(x, y)) {
					dist_[y * size_ + x] = 0.0f;
					frontier.push(y * size_ + x);
				}
			}
		}

		float step = (float)res_;
		float diag = step * 1.41421356f;
		while (!frontier.empty()) {
			int idx = frontier.front();
			frontier.pop();
			int x = idx % size_, y = idx / size_;
			float d = dist_[idx];
			if (d > soft_) continue;
			for (int oy = -1; oy <= 1; oy++) {
				for (int ox = -1; ox <= 1; ox++) {
					if (ox == 0 && oy == 0) continue;
					int nx = x + ox, ny = y + oy;
					if (nx < 0 || ny < 0 || nx >= size_ || ny >= size_) continue;
					float nd = d + ((ox && oy) ? diag : step);
					if (nd < dist_[ny * size_ + nx]) {
						dist_[ny * size_ + nx] = nd;
						frontier.push(ny * size_ + nx);
					}
				}
			}
		}

		// Rasterize the geofence by stamping each capsule's bounding box, so cost lookups stay O(1)
		for (const auto& c : geofence) {
			int x0, y0, x1, y1;
			boundCell(std::min(c.x1, c.x2) - c.radius, std::min(c.y1, c.y2) - c.radius, x0, y0);
			boundCell(std::max(c.x1, c.x2) + c.radius, std::max(c.y1, c.y2) + c.radius, x1, y1);
			for (int y = y0; y <= y1; y++) {
				for (int x = x0; x <= x1; x++) {
					if (fence_ok_[y * size_ + x]) continue;
					double wx, wy;
					cellToWorld(x, y, wx, wy);
					if (c.contains(wx, wy)) fence_ok_[y * size_ + x] = 1;
				}
			}
		}
	}

	int size() const { return size_; }
	double resolution() const { return res_; }
	double originX() const { return origin_x_; }
	double originY() const { return origin_y_; }

	bool worldToCell(double x, double y, int& cx, int& cy) const {
		cx = (int)std::floor((x - origin_x_) / res_);
		cy = (int)std::floor((y - origin_y_) / res_);
		return cx >= 0 && cy >= 0 && cx < size_ && cy < size_;
	}

	void cellToWorld(int cx, int cy, double& x, double& y) const {
		x = origin_x_ + (cx + 0.5) * res_;
		y = origin_y_ + (cy + 0.5) * res_;
	}

	bool inBounds(int cx, int cy) const { return cx >= 0 && cy >= 0 && cx < size_ && cy < size_; }
	float obstacleDistance(int cx, int cy) const { return dist_[cy * size_ + cx]; }

	float cost(int cx, int cy) const {
		if (!fence_ok_[cy * size_ + cx]) return LETHAL;
		float d = dist_[cy * size_ + cx];
		if (d <= inflate_) return LETHAL;
		if (d >= soft_) return 0.0f;
		return soft_weight_ * (float)((soft_ - d) / (soft_ - inflate_));
	}

	bool lethal(int cx, int cy) const { return cost(cx, cy) == LETHAL; }

	// World-space queries treat anything outside the field extent as free
	float costAt(double x, double y) const {
		int cx, cy;
		return worldToCell(x, y, cx, cy) ? cost(cx, cy) : 0.0f;
	}

	bool lethalAt(double x, double y) const { return costAt(x, y) == LETHAL; }

	float obstacleDistanceAt(double x, double y) const {
		int cx, cy;
		return worldToCell(x, y, cx, cy) ? dist_[cy * size_ + cx] : std::numeric_limits<float>::max();
	}

private:
	void boundCell(double x, double y, int& cx, int& cy) const {
		cx = std::max(0, std::min(size_ - 1, (int)std::floor((x - origin_x_) / res_)));
		cy = std::max(0, std::min(size_ - 1, (int)std::floor((y - origin_y_) / res_)));
	}

	double res_ = 1.0, origin_x_ = 0.0, origin_y_ = 0.0;
	int size_ = 0;
	double inflate_ = 0.0, soft_ = 0.0, soft_weight_ = 0.0;
	std::vector<float> dist_;
	std::vector<uint8_t> fence_ok_;
};

}
