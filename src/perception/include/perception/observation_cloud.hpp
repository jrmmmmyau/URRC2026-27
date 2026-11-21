#pragma once

#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

#include "perception/depth_processing.hpp"

namespace perception {

// Standard PointCloud2 schema: little-endian FLOAT64 x/y/z at offsets 0/8/16.
// Templating the message allows the wire layout to be tested without ROS.
template<typename Header, typename Cloud>
void write_observation_cloud(
  const std::vector<ObservationPoint> & points, const Header & header, Cloud & cloud)
{
  static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559,
    "Observation transport requires IEEE 754 binary64.");
  constexpr std::uint32_t point_step = 24;
  if (points.size() > std::numeric_limits<std::uint32_t>::max() / point_step) {
    throw std::length_error("Observation cloud exceeds PointCloud2 row size.");
  }

  cloud.header = header;
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = point_step;
  cloud.row_step = cloud.width * point_step;
  cloud.fields.resize(3);
  const char * names[] = {"x", "y", "z"};
  for (std::uint32_t axis = 0; axis < 3; ++axis) {
    auto & field = cloud.fields[axis];
    field.name = names[axis];
    field.offset = axis * 8;
    field.datatype = field.FLOAT64;
    field.count = 1;
  }
  cloud.data.resize(cloud.row_step);
  std::size_t offset = 0;
  for (const auto & point : points) {
    for (double value : {point.x, point.y, point.z}) {
      std::uint64_t bits;
      std::memcpy(&bits, &value, sizeof(bits));
      for (unsigned int byte = 0; byte < 8; ++byte) {
        cloud.data[offset++] = static_cast<std::uint8_t>(bits >> (8 * byte));
      }
    }
  }
}

}  // namespace perception
