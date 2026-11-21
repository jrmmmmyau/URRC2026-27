#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace perception {

struct CameraIntrinsics {
  double fx;
  double fy;
  double cx;
  double cy;
};

struct DepthSettings {
  int pixel_step = 4;
  double row_fraction = 0.5;
  double max_range = 8.0;
};

struct ObservationPoint {
  double x;
  double y;
  double z;
};

inline void validate_settings(const DepthSettings & settings)
{
  if (settings.pixel_step <= 0 || !std::isfinite(settings.row_fraction) ||
    settings.row_fraction < 0.0 || settings.row_fraction > 1.0 ||
    !std::isfinite(settings.max_range) || settings.max_range <= 0.0)
  {
    throw std::invalid_argument("Invalid depth sampling settings.");
  }
}

// The reader supplies depth in meters. Sampling order and X/Z calculations
// preserve the original mapping implementation for valid calibration.
template<typename DepthReader>
std::vector<ObservationPoint> sample_depth(
  int width, int height, const CameraIntrinsics & camera,
  const DepthSettings & settings, DepthReader read_depth)
{
  validate_settings(settings);
  if (width <= 0 || height <= 0) {
    throw std::invalid_argument("Depth image dimensions must be positive.");
  }
  if (!std::isfinite(camera.fx) || camera.fx <= 0.0 ||
    !std::isfinite(camera.fy) || camera.fy <= 0.0 ||
    !std::isfinite(camera.cx) || !std::isfinite(camera.cy))
  {
    throw std::invalid_argument("Camera intrinsics must be finite with positive focal lengths.");
  }

  const int row = std::clamp(
    static_cast<int>(settings.row_fraction * height), 0, height - 1);
  std::vector<ObservationPoint> points;
  const auto step = static_cast<std::size_t>(settings.pixel_step);
  const auto columns = static_cast<std::size_t>(width);
  points.reserve((columns - 1) / step + 1);
  for (std::size_t column = 0; column < columns; column += step) {
    const int u = static_cast<int>(column);
    const double z = read_depth(row, u);
    if (!std::isfinite(z) || z <= 0.0 || z > settings.max_range) {
      continue;
    }
    const double x = (u - camera.cx) * z / camera.fx;
    const double y = (row - camera.cy) * z / camera.fy;
    if (std::isfinite(x) && std::isfinite(y)) {
      points.push_back({x, y, z});
    }
  }
  return points;
}

}  // namespace perception
