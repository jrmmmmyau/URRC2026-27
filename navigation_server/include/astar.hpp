#pragma once
// ROS-free A* over a costmap::Costmap (8-connected, turn penalty, heap-based open list).
#include "costmap.hpp"

#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace astar {

struct Cell { int x; int y; };

struct Params {
    float turn_cost = 10.0f;        // added when the move direction changes
    int max_start_snap_cells = 40;  // how far to look for a free start if the robot sits in a blocked cell
};

struct Result {
    bool found = false;
    std::string message;
    Cell start_used{0, 0};
    bool start_snapped = false;
    std::vector<Cell> path;
    std::vector<size_t> waypoint_indices;
};

// If c is blocked, move it to the nearest traversable cell within max_r (ring search).
inline bool snap_to_traversable(const costmap::Costmap &cm, Cell &c, int max_r)
{
    if (cm.traversable(c.x, c.y)) return true;
    for (int r = 1; r <= max_r; r++) {
        bool found = false;
        float best = std::numeric_limits<float>::max();
        Cell best_c{0, 0};
        for (int dx = -r; dx <= r; dx++) {
            for (int dy = -r; dy <= r; dy++) {
                if (std::max(std::abs(dx), std::abs(dy)) != r) continue;
                if (!cm.traversable(c.x + dx, c.y + dy)) continue;
                float d = float(dx * dx + dy * dy);
                if (d < best) { best = d; best_c = {c.x + dx, c.y + dy}; found = true; }
            }
        }
        if (found) { c = best_c; return true; }
    }
    return false;
}

// Corner cells: start, every cell where the move direction changes, and the goal.
inline std::vector<size_t> extract_waypoints(const std::vector<Cell> &path)
{
    std::vector<size_t> wp;
    if (path.empty()) return wp;
    wp.push_back(0);
    for (size_t i = 1; i + 1 < path.size(); i++) {
        int dx1 = path[i].x - path[i - 1].x, dy1 = path[i].y - path[i - 1].y;
        int dx2 = path[i + 1].x - path[i].x, dy2 = path[i + 1].y - path[i].y;
        if (dx1 != dx2 || dy1 != dy2) wp.push_back(i);
    }
    if (path.size() > 1) wp.push_back(path.size() - 1);
    return wp;
}

inline Result plan(const costmap::Costmap &cm, Cell start, Cell goal, const Params &p)
{
    Result res;
    if (!cm.in_bounds(start.x, start.y) || !cm.in_bounds(goal.x, goal.y)) {
        res.message = "start or goal is outside the map";
        return res;
    }
    Cell original_start = start;
    if (!snap_to_traversable(cm, start, p.max_start_snap_cells)) {
        res.message = "start is enclosed by obstacles / clearance zone";
        return res;
    }
    res.start_used = start;
    res.start_snapped = (start.x != original_start.x || start.y != original_start.y);
    if (!cm.traversable(goal.x, goal.y)) {
        res.message = "goal is blocked (inside an obstacle or the robot clearance zone)";
        return res;
    }

    const int w = cm.width;
    const size_t n = size_t(w) * cm.height;
    const float INF = std::numeric_limits<float>::infinity();
    const float SQRT2 = 1.41421356f;

    std::vector<float> g(n, INF);
    std::vector<int> parent(n, -1);
    std::vector<int8_t> dirx(n, 0), diry(n, 0);
    std::vector<uint8_t> closed(n, 0);

    using QE = std::pair<float, int>;
    std::priority_queue<QE, std::vector<QE>, std::greater<QE>> open;
    auto heuristic = [&](int x, int y) { return std::hypot(float(goal.x - x), float(goal.y - y)); };

    const int s_idx = start.x + w * start.y;
    g[s_idx] = 0.0f;
    open.push({heuristic(start.x, start.y), s_idx});

    while (!open.empty()) {
        const int idx = open.top().second;
        open.pop();
        if (closed[idx]) continue;
        closed[idx] = 1;
        const int cx = idx % w, cy = idx / w;

        if (cx == goal.x && cy == goal.y) {
            for (int i = idx; i != -1; i = parent[i]) res.path.push_back({i % w, i / w});
            std::reverse(res.path.begin(), res.path.end());
            res.waypoint_indices = extract_waypoints(res.path);
            res.found = true;
            return res;
        }

        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;
                const int nx = cx + dx, ny = cy + dy;
                if (!cm.traversable(nx, ny)) continue;
                if (dx != 0 && dy != 0 && !(cm.traversable(cx + dx, cy) && cm.traversable(cx, cy + dy))) continue;
                const int nidx = nx + w * ny;
                if (closed[nidx]) continue;

                float step = ((dx != 0 && dy != 0) ? SQRT2 : 1.0f) + cm.extra_cost(nx, ny);
                if ((dirx[idx] != 0 || diry[idx] != 0) && (dx != dirx[idx] || dy != diry[idx])) step += p.turn_cost;

                const float ng = g[idx] + step;
                if (ng < g[nidx]) {
                    g[nidx] = ng;
                    parent[nidx] = idx;
                    dirx[nidx] = static_cast<int8_t>(dx);
                    diry[nidx] = static_cast<int8_t>(dy);
                    open.push({ng + heuristic(nx, ny), nidx});
                }
            }
        }
    }
    res.message = "no path found (open list exhausted)";
    return res;
}

}  // namespace astar
