#include <cstdio>
#include <cassert>
#include "tinyhelm_obstacles/decay_grid.h"
#include "tinyhelm_obstacles/cost_field.h"
#include "tinyhelm_obstacles/theta_star.h"

using namespace tinyhelm;

int main() {
	DecayGrid fine(0.25, 512, 32, 12, 48, 60.0);
	DecayGrid coarse(1.0, 1024, 32, 12, 48, 120.0);

	// Wall from (20,-15) to (20,15) with a real gap only around the edges
	for (double y = -15.0; y <= 15.0; y += 0.2) {
		for (int i = 0; i < 3; i++) fine.addHit(20.0, y);
	}
	int cx, cy;
	assert(fine.worldToCell(20.0, 0.0, cx, cy) && fine.occupied(cx, cy));

	std::vector<Capsule> fence = {{0, 0, 40, 0, 30.0}};
	CostField field;
	field.build(fine, 1.5, 5.0, 2.0, fence);
	assert(field.lethalAt(20.0, 0.0));
	assert(field.lethalAt(20.0, 1.0));
	assert(!field.lethalAt(5.0, 0.0));
	assert(field.lethalAt(5.0, 35.0)); // outside geofence tube

	ThetaStar planner;
	auto path = planner.plan(field, 0.0, 0.0, 40.0, 0.0);
	assert(!path.empty());
	double len = ThetaStar::pathLength(path);
	printf("path around wall: %zu pts, %.1fm (direct 40m)\n", path.size(), len);
	assert(len > 45.0 && len < 80.0);
	for (const auto& p : path) assert(!field.lethalAt(p.x, p.y));

	// Goal inside the wall must fail
	assert(planner.plan(field, 0.0, 0.0, 20.0, 0.0).empty());

	// Tight geofence that forbids going around the wall must fail
	CostField tight;
	tight.build(fine, 1.5, 5.0, 2.0, {{0, 0, 40, 0, 10.0}});
	assert(planner.plan(tight, 0.0, 0.0, 40.0, 0.0).empty());

	// Decay pulls the wall back to unknown, then the direct path opens
	for (int i = 0; i < 130; i++) fine.decay(1.0);
	CostField after;
	after.build(fine, 1.5, 5.0, 2.0, fence);
	auto direct = planner.plan(after, 0.0, 0.0, 40.0, 0.0);
	printf("after decay: %.1fm\n", ThetaStar::pathLength(direct));
	assert(ThetaStar::pathLength(direct) < 41.0);

	// Statics don't decay
	fine.setStatic(10.0, 0.0, true);
	for (int i = 0; i < 300; i++) fine.decay(1.0);
	assert(fine.worldToCell(10.0, 0.0, cx, cy) && fine.occupied(cx, cy));

	// Recenter far away spills occupied cells into the coarse layer and keeps alignment
	fine.addHit(10.0, 0.0);
	fine.addHit(10.0, 0.0);
	fine.recenter(300.0, 0.0, &coarse);
	assert(coarse.worldToCell(10.0, 0.0, cx, cy) && coarse.occupied(cx, cy));
	double before_x = coarse.originX();
	assert(std::fmod(before_x, 1.0) == 0.0 && std::fmod(fine.originX(), 0.25) == 0.0);

	// Consistency: plan with effective clearance (inflate + corridor), then validate the
	// output like the corridor monitor does; it must never reject its own path
	DecayGrid g2(0.25, 512, 32, 12, 48, 60.0);
	for (double y = -15.0; y <= 15.0; y += 0.2)
		for (int i = 0; i < 3; i++) g2.addHit(20.0, y);
	double eff = 1.5 + 4.0;
	CostField f2;
	f2.build(g2, eff, 8.0, 2.0, {{0, 0, 40, 0, 40.0}});
	auto p2 = planner.plan(f2, 0.0, 0.0, 40.0, 0.0);
	assert(!p2.empty());
	for (size_t i = 1; i < p2.size(); i++) {
		double len = std::hypot(p2[i].x - p2[i-1].x, p2[i].y - p2[i-1].y);
		int steps = std::max(1, (int)(len / (0.25 * 2.0)));
		for (int s = 0; s <= steps; s++) {
			double t = (double)s / steps;
			double x = p2[i-1].x + t * (p2[i].x - p2[i-1].x), y = p2[i-1].y + t * (p2[i].y - p2[i-1].y);
			int cx, cy;
			assert(f2.worldToCell(x, y, cx, cy));
			assert(f2.obstacleDistance(cx, cy) >= eff - 0.25);
		}
	}
	printf("planned path validates clear at effective clearance %.1fm (%.1fm long)\n", eff, ThetaStar::pathLength(p2));

	printf("all planning header tests passed\n");
	return 0;
}
