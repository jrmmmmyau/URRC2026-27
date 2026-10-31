#pragma once
// Shared costmap builder for the A* planner and the map verifier.
// No ROS dependency: pass in OccupancyGrid fields (data, width, height, resolution).
//
// Cell encoding (OccupancyGrid.data is int8_t, so grayscale bytes arrive wrapped):
//   raw -1        -> UNKNOWN
//   any other raw -> unsigned byte 0..254, 0 = free, larger = more occupied
// (so a fully occupied cell must be sent as 254, not 255)
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace costmap {

constexpr int UNKNOWN = -1;

inline int decode_cell(int8_t raw)
{
    return raw == -1 ? UNKNOWN : static_cast<int>(static_cast<uint8_t>(raw));
}

struct Params {
    float robot_radius = 2.5f;       // m, circumscribed radius of the footprint
    float safety_margin = 0.1f;      // m, extra clearance on top of the radius
    float inflation_radius = 1.5f;   // m from nearest lethal cell; no cost beyond this
    float inflation_scale = 10.0f;   // step cost at the edge of the blocked zone
    float cost_decay = 3.0f;         // 1/m, how fast inflation cost falls off
    int occupied_threshold = 165;    // decoded value >= this is lethal (0..254 scale)
    float gray_weight = 2.0f;        // step cost added at value == occupied_threshold
    bool allow_unknown = true;       // plan through unknown cells (optimistic)
    float unknown_cost = 3.0f;       // extra step cost for entering unknown cells
};

struct Costmap {
    int width = 0, height = 0;
    float resolution = 0.0f;
    std::vector<uint8_t> blocked;    // 1 = cannot enter
    std::vector<uint8_t> unknown;    // 1 = originally unknown
    std::vector<float> extra;        // added step cost (gray + inflation + unknown)

    bool in_bounds(int x, int y) const { return x >= 0 && y >= 0 && x < width && y < height; }
    bool traversable(int x, int y) const { return in_bounds(x, y) && !blocked[x + size_t(width) * y]; }
    float extra_cost(int x, int y) const { return extra[x + size_t(width) * y]; }
    bool is_unknown(int x, int y) const { return unknown[x + size_t(width) * y] != 0; }
};

namespace detail {
constexpr float INF = 1e20f;

// Felzenszwalb & Huttenlocher 1D squared-distance transform.
inline void edt_1d(const float *f, int n, float *d, int *v, float *z)
{
    int k = 0;
    v[0] = 0;
    z[0] = -INF;
    z[1] = INF;
    for (int q = 1; q < n; q++) {
        float s;
        while (true) {
            s = ((f[q] + float(q) * q) - (f[v[k]] + float(v[k]) * v[k])) / (2.0f * q - 2.0f * v[k]);
            if (s <= z[k] && k > 0) k--; else break;
        }
        k++;
        v[k] = q;
        z[k] = s;
        z[k + 1] = INF;
    }
    k = 0;
    for (int q = 0; q < n; q++) {
        while (z[k + 1] < q) k++;
        float dq = float(q - v[k]);
        d[q] = dq * dq + f[v[k]];
    }
}
}  // namespace detail

// Squared distance (cells^2) from every cell to the nearest lethal cell.
inline std::vector<float> distance_to_lethal(const std::vector<int8_t> &data, int w, int h, int occ_thresh)
{
    using detail::INF;
    const int m = std::max(w, h);
    std::vector<float> g(size_t(w) * h), tmp(m), out(m), z(m + 1);
    std::vector<int> v(m);
    for (size_t i = 0; i < g.size(); i++) g[i] = (decode_cell(data[i]) >= occ_thresh) ? 0.0f : INF;

    for (int x = 0; x < w; x++) {
        for (int y = 0; y < h; y++) tmp[y] = g[x + size_t(w) * y];
        detail::edt_1d(tmp.data(), h, out.data(), v.data(), z.data());
        for (int y = 0; y < h; y++) g[x + size_t(w) * y] = out[y];
    }
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) tmp[x] = g[x + size_t(w) * y];
        detail::edt_1d(tmp.data(), w, out.data(), v.data(), z.data());
        for (int x = 0; x < w; x++) g[x + size_t(w) * y] = out[x];
    }
    return g;
}

inline Costmap build(const std::vector<int8_t> &data, int w, int h, float res, const Params &p)
{
    Costmap cm;
    cm.width = w; cm.height = h; cm.resolution = res;
    cm.blocked.assign(size_t(w) * h, 0);
    cm.unknown.assign(size_t(w) * h, 0);
    cm.extra.assign(size_t(w) * h, 0.0f);

    std::vector<float> d2 = distance_to_lethal(data, w, h, p.occupied_threshold);
    const float hard = p.robot_radius + p.safety_margin;

    for (size_t i = 0; i < data.size(); i++) {
        const int v = decode_cell(data[i]);
        const float dist = (d2[i] >= detail::INF * 0.5f) ? detail::INF : std::sqrt(d2[i]) * res;

        if (v >= p.occupied_threshold || dist < hard) { cm.blocked[i] = 1; continue; }

        float c = 0.0f;
        if (v == UNKNOWN) {
            cm.unknown[i] = 1;
            if (!p.allow_unknown) { cm.blocked[i] = 1; continue; }
            c += p.unknown_cost;
        } else {
            c += p.gray_weight * float(v) / float(p.occupied_threshold);
        }
        if (dist < p.inflation_radius) {
            c += p.inflation_scale * std::exp(-p.cost_decay * (dist - hard));
        }
        cm.extra[i] = c;
    }
    return cm;
}

}  // namespace costmap
