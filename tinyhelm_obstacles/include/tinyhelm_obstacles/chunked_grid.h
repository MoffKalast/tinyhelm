#pragma once

#include <unordered_map>
#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>

namespace tinyhelm {

// Unbounded occupancy map stored as fixed-size chunks in a hash map. Chunks near the vessel
// are "loaded": maintain() applies decay to them every call and they back the fine planning
// window. Chunks that drift outside the load radius are summarized into a per-chunk coarse
// binary mask (the global obstacle map used for planning beyond the loaded area) while their
// full-resolution data is retained, so returning to an area restores full precision. A chunk
// is discarded outright once enough time has passed since its last hit that every cell would
// have decayed to zero anyway; statics never decay and pin their chunk forever. Decay is
// lazy: each chunk stores its last decay time and catches up on access, so cold chunks cost
// nothing per tick.
class ChunkedGrid {
public:
	ChunkedGrid(double resolution, double chunk_size_m, double mask_resolution, int8_t hit_delta, int8_t miss_delta, int8_t occupied_threshold, double half_life_s)
		: res_(resolution), cells_(std::max(1, (int)std::round(chunk_size_m / resolution))), chunk_m_(cells_ * resolution),
		  mask_cells_(std::min(8, std::max(1, (int)std::round(chunk_m_ / mask_resolution)))), mask_res_(chunk_m_ / mask_cells_),
		  hit_delta_(hit_delta), miss_delta_(miss_delta), occ_thresh_(occupied_threshold), half_life_(half_life_s) {}

	double resolution() const { return res_; }
	double chunkSize() const { return chunk_m_; }
	double maskResolution() const { return mask_res_; }
	int8_t occupiedThreshold() const { return occ_thresh_; }
	size_t chunkCount() const { return chunks_.size(); }
	size_t maskCount() const { return masks_.size(); }

	void addHit(double x, double y, double now) { bump(x, y, hit_delta_, now); }
	void addMiss(double x, double y, double now) { bump(x, y, (int8_t)(-miss_delta_), now); }

	// Walks the ray from (x0, y0) to stop_short metres before (x1, y1), nudging positive
	// cells toward unknown by ray_delta. Never pushes below 0: a swept beam erodes stale
	// evidence of presence but does not accumulate evidence of absence. Caches the current
	// chunk so long rays don't pay a hash lookup per cell.
	void rayMiss(double x0, double y0, double x1, double y1, int8_t ray_delta, double stop_short, double now) {
		double dx = x1 - x0, dy = y1 - y0;
		double len = std::hypot(dx, dy) - stop_short;
		if (len <= 0.0) return;
		int steps = (int)(len / res_);
		long long cached_key = 0;
		Chunk* cached = nullptr;
		for (int i = 0; i <= steps; i++) {
			double t = i * res_ / std::hypot(dx, dy);
			double x = x0 + t * dx, y = y0 + t * dy;
			long long k = key(x, y);
			if (!cached || k != cached_key) {
				auto it = chunks_.find(k);
				cached = it == chunks_.end() ? nullptr : &it->second;
				cached_key = k;
				if (cached) decayChunk(*cached, now);
			}
			if (!cached) continue;
			int8_t& v = cached->lo[cellIndex(x, y)];
			if (v > 0) {
				v = (int8_t)std::max(0, v - ray_delta);
				masks_.erase(k);
			}
		}
	}

	void setStatic(double x, double y, bool value) {
		Chunk& c = chunkAt(key(x, y));
		int idx = cellIndex(x, y);
		if (value && !c.st[idx]) c.static_count++;
		if (!value && c.st[idx]) c.static_count--;
		c.st[idx] = value ? 1 : 0;
		masks_.erase(key(x, y));
	}

	void clear() {
		chunks_.clear();
		masks_.clear();
	}

	// Decays loaded chunks, refreshes masks for chunks leaving the load radius, and discards
	// chunks (and expired masks) whose data would have fully decayed. Call once per tick.
	void maintain(double rx, double ry, double load_radius, double now) {
		for (auto it = chunks_.begin(); it != chunks_.end();) {
			Chunk& c = it->second;
			if (c.static_count == 0 && now - c.last_hit >= fullDecayTime()) {
				masks_.erase(it->first);
				it = chunks_.erase(it);
				continue;
			}
			if (chunkDistance(it->first, rx, ry) <= load_radius) {
				decayChunk(c, now);
				masks_.erase(it->first);
			} else if (masks_.find(it->first) == masks_.end()) {
				decayChunk(c, now);
				masks_[it->first] = computeMask(c);
			}
			++it;
		}
		for (auto it = masks_.begin(); it != masks_.end();) {
			if (it->second.static_bits == 0 && now >= it->second.expiry) it = masks_.erase(it); else ++it;
		}
	}

	// Live full-resolution queries; valid for loaded chunks (decayed by maintain)
	int8_t at(double x, double y) const {
		auto it = chunks_.find(key(x, y));
		return it == chunks_.end() ? 0 : it->second.lo[cellIndex(x, y)];
	}

	bool isStatic(double x, double y) const {
		auto it = chunks_.find(key(x, y));
		return it != chunks_.end() && it->second.st[cellIndex(x, y)] != 0;
	}

	bool occupiedAt(double x, double y) const {
		auto it = chunks_.find(key(x, y));
		if (it == chunks_.end()) return false;
		int idx = cellIndex(x, y);
		return it->second.st[idx] != 0 || it->second.lo[idx] >= occ_thresh_;
	}

	// Coarse (mask-resolution) occupancy: masked chunks answer from their bitfield, loaded
	// chunks scan the live cells under the mask cell, absent chunks are free
	bool occupiedCoarseAt(double x, double y) const {
		long long k = key(x, y);
		auto mit = masks_.find(k);
		if (mit != masks_.end()) {
			int b = maskBit(x, y);
			return ((mit->second.bits | mit->second.static_bits) >> b & 1) != 0;
		}
		auto cit = chunks_.find(k);
		if (cit == chunks_.end()) return false;
		return maskCellOccupied(cit->second, maskBit(x, y));
	}

	bool occupiedCoarseNear(double x, double y, double radius) const {
		for (double my = y - radius; my <= y + radius + mask_res_; my += mask_res_) {
			for (double mx = x - radius; mx <= x + radius + mask_res_; mx += mask_res_) {
				if (occupiedCoarseAt(mx, my)) return true;
			}
		}
		return false;
	}

private:
	struct Chunk {
		std::vector<int8_t> lo;
		std::vector<uint8_t> st;
		double last_decay = 0.0, last_hit = 0.0;
		int static_count = 0;
	};

	struct Mask {
		uint64_t bits = 0, static_bits = 0;
		double expiry = 0.0;
	};

	double fullDecayTime() const { return 2.0 * half_life_; }

	static long long pack(int kx, int ky) { return ((long long)kx << 32) ^ (uint32_t)ky; }
	long long key(double x, double y) const { return pack((int)std::floor(x / chunk_m_), (int)std::floor(y / chunk_m_)); }

	int cellIndex(double x, double y) const {
		int gx = (int)std::floor(x / res_), gy = (int)std::floor(y / res_);
		int kx = (int)std::floor(x / chunk_m_), ky = (int)std::floor(y / chunk_m_);
		return (gy - ky * cells_) * cells_ + (gx - kx * cells_);
	}

	int maskBit(double x, double y) const {
		int mx = (int)std::floor(x / mask_res_), my = (int)std::floor(y / mask_res_);
		int kx = (int)std::floor(x / chunk_m_), ky = (int)std::floor(y / chunk_m_);
		return (my - ky * mask_cells_) * mask_cells_ + (mx - kx * mask_cells_);
	}

	double chunkDistance(long long k, double rx, double ry) const {
		double cx = ((int)(k >> 32) + 0.5) * chunk_m_, cy = ((int)(k & 0xffffffffLL) + 0.5) * chunk_m_;
		return std::max(std::fabs(cx - rx), std::fabs(cy - ry)) - 0.5 * chunk_m_;
	}

	Chunk& chunkAt(long long k) {
		auto it = chunks_.find(k);
		if (it != chunks_.end()) return it->second;
		Chunk& c = chunks_[k];
		c.lo.assign(cells_ * cells_, 0);
		c.st.assign(cells_ * cells_, 0);
		return c;
	}

	void bump(double x, double y, int8_t delta, double now) {
		long long k = key(x, y);
		Chunk& c = chunkAt(k);
		decayChunk(c, now);
		int idx = cellIndex(x, y);
		int v = (int)c.lo[idx] + delta;
		c.lo[idx] = (int8_t)std::max(-127, std::min(127, v));
		if (delta > 0) c.last_hit = now;
		masks_.erase(k);
	}

	void decayChunk(Chunk& c, double now) {
		if (half_life_ <= 0.0) return;
		if (c.last_decay == 0.0) { c.last_decay = now; return; }
		double per_step = fullDecayTime() / 127.0;
		int step = (int)((now - c.last_decay) / per_step);
		if (step < 1) return;
		c.last_decay += step * per_step;
		for (auto& v : c.lo) {
			if (v > 0) v = (int8_t)std::max(0, v - step);
			else if (v < 0) v = (int8_t)std::min(0, v + step);
		}
	}

	bool maskCellOccupied(const Chunk& c, int bit) const {
		int sub = cells_ / mask_cells_;
		int mx = bit % mask_cells_, my = bit / mask_cells_;
		for (int y = my * sub; y < (my + 1) * sub; y++) {
			for (int x = mx * sub; x < (mx + 1) * sub; x++) {
				int idx = y * cells_ + x;
				if (c.st[idx] != 0 || c.lo[idx] >= occ_thresh_) return true;
			}
		}
		return false;
	}

	Mask computeMask(const Chunk& c) const {
		Mask m;
		int sub = cells_ / mask_cells_;
		for (int my = 0; my < mask_cells_; my++) {
			for (int mx = 0; mx < mask_cells_; mx++) {
				bool occ = false, sta = false;
				for (int y = my * sub; y < (my + 1) * sub && !(occ && sta); y++) {
					for (int x = mx * sub; x < (mx + 1) * sub; x++) {
						int idx = y * cells_ + x;
						if (c.lo[idx] >= occ_thresh_) occ = true;
						if (c.st[idx] != 0) sta = true;
					}
				}
				uint64_t b = 1ULL << (my * mask_cells_ + mx);
				if (occ) m.bits |= b;
				if (sta) m.static_bits |= b;
			}
		}
		m.expiry = c.last_hit + fullDecayTime();
		return m;
	}

	double res_;
	int cells_;
	double chunk_m_;
	int mask_cells_;
	double mask_res_;
	int8_t hit_delta_, miss_delta_, occ_thresh_;
	double half_life_;
	std::unordered_map<long long, Chunk> chunks_;
	std::unordered_map<long long, Mask> masks_;
};

}
