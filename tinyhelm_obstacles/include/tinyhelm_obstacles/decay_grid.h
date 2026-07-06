#pragma once

#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>

namespace tinyhelm {

// Axis-aligned occupancy grid in the fixed frame. The origin is always snapped to whole
// multiples of the resolution, so recentering shifts the data by whole cells and the world
// position of surviving cells never changes. Cells hold log-odds in [-127, 127] that decay
// toward 0 (unknown) with a configurable half-life. Cells scrolled off the edge can be
// spilled into a coarser DecayGrid so distant detail degrades instead of vanishing.
class DecayGrid {
public:
	DecayGrid(double resolution, int size, int8_t hit_delta, int8_t miss_delta, int8_t occupied_threshold, double half_life_s)
		: res_(resolution), size_(size), hit_delta_(hit_delta), miss_delta_(miss_delta), occ_thresh_(occupied_threshold), half_life_(half_life_s),
		  origin_ix_(-size / 2), origin_iy_(-size / 2), decay_accum_(0.0), logodds_(size * size, 0), statics_(size * size, 0) {}

	double resolution() const { return res_; }
	int size() const { return size_; }
	double originX() const { return origin_ix_ * res_; }
	double originY() const { return origin_iy_ * res_; }
	int8_t occupiedThreshold() const { return occ_thresh_; }

	bool worldToCell(double x, double y, int& cx, int& cy) const {
		cx = (int)std::floor(x / res_) - origin_ix_;
		cy = (int)std::floor(y / res_) - origin_iy_;
		return cx >= 0 && cy >= 0 && cx < size_ && cy < size_;
	}

	void cellToWorld(int cx, int cy, double& x, double& y) const {
		x = (origin_ix_ + cx + 0.5) * res_;
		y = (origin_iy_ + cy + 0.5) * res_;
	}

	int8_t at(int cx, int cy) const { return logodds_[cy * size_ + cx]; }
	bool isStatic(int cx, int cy) const { return statics_[cy * size_ + cx] != 0; }
	bool occupied(int cx, int cy) const { return isStatic(cx, cy) || at(cx, cy) >= occ_thresh_; }

	void addHit(double x, double y) { bump(x, y, hit_delta_); }
	void addMiss(double x, double y) { bump(x, y, (int8_t)(-miss_delta_)); }

	void setStatic(double x, double y, bool value) {
		int cx, cy;
		if (worldToCell(x, y, cx, cy)) statics_[cy * size_ + cx] = value ? 1 : 0;
	}

	void clear() {
		std::fill(logodds_.begin(), logodds_.end(), 0);
		std::fill(statics_.begin(), statics_.end(), 0);
	}

	// Linear pull toward unknown, scaled so a saturated cell reaches 0 in ~2 half-lives.
	void decay(double dt) {
		if (half_life_ <= 0.0) return;
		decay_accum_ += 127.0 * dt / (2.0 * half_life_);
		int step = (int)decay_accum_;
		if (step < 1) return;
		decay_accum_ -= step;
		for (auto& v : logodds_) {
			if (v > 0) v = (int8_t)std::max(0, v - step);
			else if (v < 0) v = (int8_t)std::min(0, v + step);
		}
	}

	// Shift so (x, y) lands near the center whenever it drifts past a quarter of the extent.
	// Occupied cells (and statics) that fall off the edge are max-pooled into `spill`.
	void recenter(double x, double y, DecayGrid* spill) {
		int cx = (int)std::floor(x / res_);
		int cy = (int)std::floor(y / res_);
		int dx = cx - (origin_ix_ + size_ / 2);
		int dy = cy - (origin_iy_ + size_ / 2);
		if (std::abs(dx) < size_ / 4 && std::abs(dy) < size_ / 4) return;
		shift(dx, dy, spill);
	}

private:
	void bump(double x, double y, int8_t delta) {
		int cx, cy;
		if (!worldToCell(x, y, cx, cy)) return;
		int v = (int)logodds_[cy * size_ + cx] + delta;
		logodds_[cy * size_ + cx] = (int8_t)std::max(-127, std::min(127, v));
	}

	void shift(int dx, int dy, DecayGrid* spill) {
		std::vector<int8_t> new_logodds(size_ * size_, 0);
		std::vector<uint8_t> new_statics(size_ * size_, 0);
		for (int y = 0; y < size_; y++) {
			for (int x = 0; x < size_; x++) {
				int8_t v = logodds_[y * size_ + x];
				uint8_t s = statics_[y * size_ + x];
				if (v == 0 && s == 0) continue;
				int nx = x - dx, ny = y - dy;
				if (nx >= 0 && ny >= 0 && nx < size_ && ny < size_) {
					new_logodds[ny * size_ + nx] = v;
					new_statics[ny * size_ + nx] = s;
				} else if (spill && (s || v >= occ_thresh_)) {
					double wx, wy;
					cellToWorld(x, y, wx, wy);
					spill->absorb(wx, wy, v, s != 0);
				}
			}
		}
		logodds_.swap(new_logodds);
		statics_.swap(new_statics);
		origin_ix_ += dx;
		origin_iy_ += dy;
	}

	void absorb(double x, double y, int8_t value, bool is_static) {
		int cx, cy;
		if (!worldToCell(x, y, cx, cy)) return;
		int idx = cy * size_ + cx;
		logodds_[idx] = std::max(logodds_[idx], value);
		if (is_static) statics_[idx] = 1;
	}

	double res_;
	int size_;
	int8_t hit_delta_, miss_delta_, occ_thresh_;
	double half_life_;
	int origin_ix_, origin_iy_;
	double decay_accum_;
	std::vector<int8_t> logodds_;
	std::vector<uint8_t> statics_;
};

}
