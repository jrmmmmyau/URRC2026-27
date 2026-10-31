#include "astar.hpp"
#include <cstdio>
#include <cassert>
static int8_t enc(int byte) { return static_cast<int8_t>(static_cast<uint8_t>(byte)); }
int main() {
    int w = 100, h = 100; float res = 0.05f;
    costmap::Params p;   // blocked radius 0.6 m = 12 cells
    // --- costmap ---
    std::vector<int8_t> data(w * h, 0);
    data[50 + w * 50] = enc(254);
    for (int x = 0; x < 20; x++) data[x + w * 5] = -1;
    data[80 + w * 80] = enc(100);
    auto cm = costmap::build(data, w, h, res, p);
    printf("lethal(254 wrapped) blocked=%d (1)\n", cm.blocked[50 + w * 50]);
    printf("11 cells blocked=%d (1)  13 cells blocked=%d (0)\n", cm.blocked[61 + w * 50], cm.blocked[63 + w * 50]);
    printf("unknown: unknown=%d blocked=%d extra=%.2f (1 0 3.00)\n", cm.unknown[3 + w * 5], cm.blocked[3 + w * 5], cm.extra[3 + w * 5]);
    printf("gray(100) extra=%.3f (1.212)\n", cm.extra[80 + w * 80]);
    printf("gray 200 (<254) below thresh? decode=%d\n", costmap::decode_cell(enc(200)));
    // --- A*: wall at x=50 with a gap at y 80..95, unknown region on the right ---
    std::vector<int8_t> m(w * h, 0);
    for (int y = 0; y < 80; y++) m[50 + w * y] = enc(254);
    for (int y = 0; y < h; y++) for (int x = 70; x < 100; x++) m[x + w * y] = -1;
    auto cm2 = costmap::build(m, w, h, res, p);
    astar::Params ap;
    auto r = astar::plan(cm2, {10, 10}, {90, 10}, ap);   // goal inside unknown
    printf("plan found=%d len=%zu waypoints=%zu msg='%s'\n", r.found, r.path.size(), r.waypoint_indices.size(), r.message.c_str());
    bool ok = r.found;
    for (auto &c : r.path) if (!cm2.traversable(c.x, c.y)) ok = false;
    printf("all path cells traversable: %d\n", ok);
    auto r2 = astar::plan(cm2, {10, 10}, {50, 10}, ap);  // goal in wall
    printf("goal in wall -> found=%d msg='%s'\n", r2.found, r2.message.c_str());
    auto r3 = astar::plan(cm2, {45, 10}, {10, 10}, ap);  // start inside clearance zone of wall
    printf("start in clearance -> found=%d snapped=%d\n", r3.found, r3.start_snapped);
    costmap::Params np = p; np.allow_unknown = false;
    auto cm3 = costmap::build(m, w, h, res, np);
    auto r4 = astar::plan(cm3, {10, 10}, {90, 10}, ap);
    printf("unknown disallowed -> found=%d\n", r4.found);
    // waypoints on straight line
    std::vector<int8_t> e(w * h, 0);
    auto r5 = astar::plan(costmap::build(e, w, h, res, p), {5, 5}, {5, 60}, ap);
    printf("straight: len=%zu waypoints=%zu (expect 56, 2)\n", r5.path.size(), r5.waypoint_indices.size());
}
