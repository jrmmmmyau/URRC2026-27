#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "perception/observation_cloud.hpp"


struct Stamp {std::int32_t sec = 1; std::uint32_t nanosec = 0;};
struct Header {Stamp stamp; std::string frame_id = "map";};
struct Field {
  static constexpr std::uint8_t FLOAT64 = 8;
  std::string name;
  std::uint32_t offset = 0;
  std::uint8_t datatype = 0;
  std::uint32_t count = 0;
};
struct Cloud {
  Header header;
  std::uint32_t height = 0, width = 0, point_step = 0, row_step = 0;
  bool is_bigendian = false, is_dense = false;
  std::vector<Field> fields;
  std::vector<std::uint8_t> data;
};
using namespace perception;
int checks = 0;
void require(bool yes, const std::string & why) {++checks; if (!yes) {throw std::runtime_error(why);}}
void near(double a, double b, const std::string & why) {require(std::abs(a-b) < 1e-10, why);}
template<typename F> void invalid(F f) {
  bool rejected = false;
  try {f();} catch (const std::invalid_argument &) {rejected = true;}
  require(rejected, "Invalid input was accepted");
}

void test_sampling_and_encoding()
{
  const CameraIntrinsics camera{100, 200, 2, 1};
  const std::vector<double> depths{0, -1, 2, 8, 8.1,
    std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()};
  const auto points = sample_depth(7, 3, camera, {1, 0.5, 8},
    [&](int row, int col) {require(row == 1, "Wrong sampling row"); return depths[col];});
  require(points.size() == 2, "Invalid or out-of-range depths were retained");
  near(points[0].x, 0, "Centered optical X");
  near(points[0].z, 2, "Depth order changed");
  near(points[1].x, 0.08, "Off-center optical X");
  near(points[1].y, 0, "Centered optical Y");
  near(points[1].z, 8, "Maximum depth boundary lost");
  std::vector<int> columns;
  const auto sampled = sample_depth(9, 3, camera, {4, 1, 8},
    [&](int row, int col) {require(row == 2, "Bottom row not clamped"); columns.push_back(col); return 1.;});
  require(columns == std::vector<int>({0,4,8}), "Sample stride/order changed");
  near(sampled[0].y, 0.005, "Vertical optical coordinate");
  const auto empty = sample_depth(1, 1, camera, {}, [](int, int) {return 0.;});
  require(empty.empty(), "Invalid depth invented a point");
  for (const auto & settings : std::vector<DepthSettings>{
    {0,0.5,8}, {-1,0.5,8}, {4,-0.1,8}, {4,1.1,8}, {4,0.5,0},
    {4,std::numeric_limits<double>::quiet_NaN(),8}, {4,0.5,std::numeric_limits<double>::infinity()}})
  {invalid([&] {sample_depth(1,1,camera,settings,[](int,int) {return 1.;});});}
  for (const auto & intrinsics : std::vector<CameraIntrinsics>{
    {0,200,2,1}, {100,0,2,1}, {100,200,std::numeric_limits<double>::quiet_NaN(),1}})
  {invalid([&] {sample_depth(1,1,intrinsics,{},[](int,int) {return 1.;});});}
  invalid([&] {sample_depth(0,1,camera,{},[](int,int) {return 1.;});});

  const double precise = std::nextafter(1.,2.);
  Cloud cloud;
  const Header header{{123,456}, "camera_optical"};
  write_observation_cloud(std::vector<ObservationPoint>{{precise,-precise,0.3}}, header, cloud);
  require(cloud.header.frame_id == header.frame_id && cloud.header.stamp.sec == 123 && cloud.header.stamp.nanosec == 456,
    "Encoding lost frame or measurement time");
  require(cloud.width == 1 && cloud.height == 1 && cloud.point_step == 24 && cloud.row_step == 24 &&
    cloud.data.size() == 24 && !cloud.is_bigendian && cloud.is_dense, "Wrong cloud layout");
  for (std::size_t axis=0; axis<3; ++axis) {
    require(cloud.fields[axis].name == std::string(1,"xyz"[axis]) &&
      cloud.fields[axis].offset == axis*8 && cloud.fields[axis].datatype == 8 &&
      cloud.fields[axis].count == 1, "Wrong field schema");
    std::uint64_t bits=0;
    for (unsigned byte=0; byte<8; ++byte) {bits |= std::uint64_t(cloud.data[axis*8+byte]) << (8*byte);}
    double decoded;
    std::memcpy(&decoded,&bits,sizeof(bits));
    const double expected[] = {precise,-precise,0.3};
    require(decoded == expected[axis], "FLOAT64 encoding lost precision");
  }
  write_observation_cloud({},header,cloud);
  require(cloud.width == 0 && cloud.row_step == 0 && cloud.data.empty(), "Empty cloud retained stale points");
}

int main()
{
  try {
    test_sampling_and_encoding();
    std::cout << "PASS: sampling, filtering, calibration checks and FLOAT64 encoding ("
              << checks << " assertions).\n";
    return 0;
  } catch (const std::exception & e) {std::cerr << "FAIL: " << e.what() << '\n'; return 1;}
}
