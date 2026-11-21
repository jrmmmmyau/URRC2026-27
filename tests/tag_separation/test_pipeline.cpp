// Local callback comparison harness. These types are test doubles, not ROS/tf2.
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#define RCLCPP_WARN_THROTTLE(...) ((void)0)
#define RCLCPP_INFO_THROTTLE(...) ((void)0)

struct Stamp {std::int32_t sec = 0; std::uint32_t nanosec = 0;};
struct Header {Stamp stamp; std::string frame_id;};
struct Vector3 {double x = 0, y = 0, z = 0;};
struct Quaternion {double x = 0, y = 0, z = 0, w = 0;};
namespace geometry_msgs::msg {
struct Transform {Vector3 translation; Quaternion rotation;};
struct TransformStamped {Header header; std::string child_frame_id; Transform transform;};
struct Pose {Vector3 position; Quaternion orientation;};
struct PoseWithCovariance {Pose pose; std::array<double, 36> covariance{};};
struct PoseWithCovarianceStamped {Header header; PoseWithCovariance pose;};
}  // namespace geometry_msgs::msg
namespace apriltag_msgs::msg {
struct Detection {std::int64_t id;};
struct AprilTagDetectionArray {
  using SharedPtr = std::shared_ptr<AprilTagDetectionArray>;
  using ConstSharedPtr = std::shared_ptr<const AprilTagDetectionArray>;
  Header header;
  std::vector<Detection> detections;
};
}  // namespace apriltag_msgs::msg
namespace perception::msg {
struct TagObservation {std::int64_t id; geometry_msgs::msg::Transform transform;};
struct TagObservationArray {
  using ConstSharedPtr = std::shared_ptr<const TagObservationArray>;
  Header header;
  std::vector<TagObservation> observations;
};
}  // namespace perception::msg
namespace rclcpp {
struct Time {explicit Time(Stamp value) : stamp(value) {} Stamp stamp;};
struct Duration {
  double seconds;
  static Duration from_seconds(double seconds) {return {seconds};}
};
}  // namespace rclcpp

// A unit-quaternion rigid-transform model supplies arithmetic only. Both
// callbacks run against this same model; real tf2 integration is a ROS check.
Quaternion multiply(Quaternion a, Quaternion b)
{
  return {a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
    a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z};
}
Quaternion conjugate(Quaternion q) {return {-q.x, -q.y, -q.z, q.w};}
Vector3 rotate(Quaternion q, Vector3 v)
{
  const auto out = multiply(multiply(q, {v.x, v.y, v.z, 0}), conjugate(q));
  return {out.x, out.y, out.z};
}
namespace tf2 {
struct TransformException : std::runtime_error {using std::runtime_error::runtime_error;};
struct Transform {
  Vector3 translation{};
  Quaternion rotation{0, 0, 0, 1};
  Transform inverse() const {
    const auto q = conjugate(rotation);
    return {rotate(q, {-translation.x, -translation.y, -translation.z}), q};
  }
  Transform operator*(const Transform & other) const {
    const auto v = rotate(rotation, other.translation);
    return {{translation.x + v.x, translation.y + v.y, translation.z + v.z},
      multiply(rotation, other.rotation)};
  }
};
void fromMsg(const geometry_msgs::msg::Transform & msg, Transform & out)
{out = {msg.translation, msg.rotation};}
void toMsg(const Transform & transform, geometry_msgs::msg::Pose & out)
{out = {transform.translation, transform.rotation};}
}  // namespace tf2

using Key = std::tuple<std::string, std::string, std::int32_t, std::uint32_t>;
struct Query {Key key; double timeout;};
struct Buffer {
  std::map<Key, geometry_msgs::msg::TransformStamped> values;
  std::vector<Query> queries;
  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target, const std::string & source,
    rclcpp::Time time, rclcpp::Duration timeout)
  {
    Key key{target, source, time.stamp.sec, time.stamp.nanosec};
    queries.push_back({key, timeout.seconds});
    const auto found = values.find(key);
    if (found == values.end()) {throw tf2::TransformException("Unavailable test transform");}
    return found->second;
  }
};
template<typename Message> struct Publisher {
  std::vector<Message> messages;
  void publish(const Message & message) {messages.push_back(message);}
};
struct LocalizerState {
  std::string map_frame_ = "map", base_frame_ = "base_link", tag_prefix_ = "tag_";
  double pos_var_ = 0.0025, ang_var_ = 0.0025, move_threshold_ = 0.1;
  std::unordered_map<std::int64_t, tf2::Transform> tag_in_map_;
  std::unique_ptr<Buffer> tf_buffer_ = std::make_unique<Buffer>();
  std::shared_ptr<Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>> pub_ =
    std::make_shared<Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>>();
  geometry_msgs::msg::PoseWithCovarianceStamped last_pose_;
  bool has_last_pose_ = false;
};
struct ObserverState {
  std::string output_frame_ = "base_link", tag_prefix_ = "tag_";
  double tf_timeout_ = 0.1;
  std::unique_ptr<Buffer> tf_buffer_ = std::make_unique<Buffer>();
  std::shared_ptr<Publisher<perception::msg::TagObservationArray>> publisher_ =
    std::make_shared<Publisher<perception::msg::TagObservationArray>>();
};

#include "callbacks.hpp"

void require(bool condition, const std::string & message)
{if (!condition) {throw std::runtime_error(message);}}
void near(double a, double b, const std::string & message)
{require(std::abs(a-b) < 1e-12, message);}
std::array<double, 7> components(const geometry_msgs::msg::Pose & p)
{return {p.position.x, p.position.y, p.position.z,
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w};}
void compare_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & a,
  const geometry_msgs::msg::PoseWithCovarianceStamped & b)
{
  require(a.header.frame_id == b.header.frame_id && a.header.stamp.sec == b.header.stamp.sec &&
    a.header.stamp.nanosec == b.header.stamp.nanosec, "Pose header changed");
  require(components(a.pose.pose) == components(b.pose.pose), "Pose values changed");
  require(a.pose.covariance == b.pose.covariance, "Covariance changed");
}
geometry_msgs::msg::Transform make_transform(double x, double y = 0, double z = 0,
  double roll = 0, double pitch = 0, double yaw = 0)
{
  const Quaternion qx{std::sin(roll/2), 0, 0, std::cos(roll/2)};
  const Quaternion qy{0, std::sin(pitch/2), 0, std::cos(pitch/2)};
  const Quaternion qz{0, 0, std::sin(yaw/2), std::cos(yaw/2)};
  return {{x, y, z}, multiply(multiply(qz, qy), qx)};
}

int frames = 0;
struct Pipeline {
  ReferenceLocalizer old;
  CurrentLocalizer current;
  CurrentObserver observer;
  void field(std::int64_t id, const geometry_msgs::msg::Transform & value) {
    tf2::Transform transform;
    tf2::fromMsg(value, transform);
    old.tag_in_map_[id] = current.tag_in_map_[id] = transform;
  }
  void run(const std::vector<std::int64_t> & ids,
    const std::map<std::int64_t, geometry_msgs::msg::Transform> & available,
    Stamp stamp = {123, 456789})
  {
    auto detections = std::make_shared<apriltag_msgs::msg::AprilTagDetectionArray>();
    detections->header = {stamp, "camera_color_optical_frame"};
    old.tf_buffer_->values.clear();
    old.tf_buffer_->queries.clear();
    observer.tf_buffer_->values.clear();
    observer.tf_buffer_->queries.clear();
    for (const auto id : ids) {detections->detections.push_back({id});}
    for (const auto & [id, transform] : available) {
      const auto name = observer.tag_prefix_ + std::to_string(id);
      Key key{observer.output_frame_, name, stamp.sec, stamp.nanosec};
      // Deliberately different TF result time: output must retain detection time.
      geometry_msgs::msg::TransformStamped value{{{999, 0}, observer.output_frame_}, name, transform};
      old.tf_buffer_->values[key] = observer.tf_buffer_->values[key] = value;
    }
    old.on_detections(detections);
    observer.on_detections(detections);
    const auto & output = observer.publisher_->messages.back();
    require(output.header.frame_id == observer.output_frame_, "Wrong observation frame");
    require(output.header.stamp.sec == stamp.sec && output.header.stamp.nanosec == stamp.nanosec,
      "Detection timestamp was replaced by TF or processing time");
    require(observer.tf_buffer_->queries.size() == ids.size(), "Detection query count changed");
    std::size_t emitted = 0;
    for (std::size_t i = 0; i < ids.size(); ++i) {
      const auto & query = observer.tf_buffer_->queries[i];
      require(query.key == Key(observer.output_frame_, observer.tag_prefix_ + std::to_string(ids[i]),
        stamp.sec, stamp.nanosec), "TF direction, prefix, order, or timestamp changed");
      require(query.timeout == observer.tf_timeout_, "TF wait changed");
      const auto found = available.find(ids[i]);
      if (found != available.end()) {
        require(emitted < output.observations.size(), "Successful observation lost");
        const auto & observation = output.observations[emitted++];
        require(observation.id == ids[i], "Tag ID order changed");
        require(components({observation.transform.translation, observation.transform.rotation}) ==
          components({found->second.translation, found->second.rotation}), "TF precision changed");
      }
    }
    require(emitted == output.observations.size(), "Failed lookup emitted an observation");
    current.on_observations(std::make_shared<perception::msg::TagObservationArray>(output));
    require(old.pub_->messages.size() == current.pub_->messages.size(), "Pose output count changed");
    for (std::size_t i = 0; i < old.pub_->messages.size(); ++i) {
      compare_pose(old.pub_->messages[i], current.pub_->messages[i]);
    }
    require(old.has_last_pose_ == current.has_last_pose_, "Pose history validity changed");
    compare_pose(old.last_pose_, current.last_pose_);
    ++frames;
  }
};

void test_pose_and_history()
{
  Pipeline p;
  p.field(0, make_transform(10));
  p.run({0}, {{0, make_transform(2)}});
  near(p.current.pub_->messages.back().pose.pose.position.x, 8, "Translation result is wrong");
  p.run({0}, {{0, make_transform(1.95)}});  // Small move accepted.
  p.run({0}, {{0, make_transform(1.0)}});   // Jump skipped, but last_pose still updated.
  require(p.current.pub_->messages.size() == 2, "Jump unexpectedly published");
  near(p.current.last_pose_.pose.pose.position.x, 9, "Rejected jump did not update history");
  p.run({0}, {{0, make_transform(1.0)}});
  require(p.current.pub_->messages.size() == 3, "Repeated pose after jump was not accepted");
  const auto covariance = p.current.pub_->messages.back().pose.covariance;
  for (std::size_t i = 0; i < covariance.size(); ++i) {
    near(covariance[i], i % 7 == 0 ? 0.0025 : 0.0, "Wrong covariance");
  }
  Pipeline rotated;
  rotated.field(0, make_transform(10, 20, 0, 0, 0, std::acos(-1.0)/2));
  rotated.run({0}, {{0, make_transform(2)}});
  const auto pose = rotated.current.pub_->messages.back().pose.pose;
  near(pose.position.x, 10, "Rotation X is wrong");
  near(pose.position.y, 18, "Rotation Y is wrong");
  near(pose.orientation.z, std::sqrt(0.5), "Rotation quaternion is wrong");
}

void test_inputs_and_lookup()
{
  Pipeline p;
  p.field(0, make_transform(0));
  p.field(1, make_transform(1));
  p.run({}, {});
  p.run({0}, {});
  p.run({99}, {{99, make_transform(1)}});
  require(p.current.pub_->messages.empty(), "Empty/missing/unknown input produced a pose");
  p.run({0, 1, 0, 99, 2}, {{0, make_transform(1)}, {1, make_transform(2)}, {99, make_transform(7)}});
  p.run({0}, {{0, make_transform(1)}}, {0, 0});
  p.run({0}, {{0, make_transform(1.01)}}, {124, 42});
  p.observer.tag_prefix_ = p.old.tag_prefix_ = "custom_";
  p.observer.output_frame_ = p.old.base_frame_ = p.current.base_frame_ = "robot_body";
  p.observer.tf_timeout_ = 0.025;
  p.run({0}, {{0, make_transform(std::nextafter(1.0, 2.0))}});

  const auto count = p.current.pub_->messages.size();
  const auto last = p.current.last_pose_;
  auto wrong = std::make_shared<perception::msg::TagObservationArray>(p.observer.publisher_->messages.back());
  for (const std::string frame : {"camera_optical", ""}) {
    wrong->header.frame_id = frame;
    p.current.on_observations(wrong);
    require(p.current.pub_->messages.size() == count, "Wrong frame produced a pose");
    compare_pose(last, p.current.last_pose_);
  }
}

void test_sequences()
{
  Pipeline p;
  p.field(0, make_transform(3, -2, 0.5, 0.1, -0.2, 0.3));
  p.field(1, make_transform(-1, 4, 2, -0.4, 0.2, -0.1));
  std::mt19937 random(240527);
  std::uniform_real_distribution<double> value(-2, 2);
  for (int i = 0; i < 200; ++i) {
    const auto sample = make_transform(value(random), value(random), value(random),
      value(random), value(random), value(random));
    std::map<std::int64_t, geometry_msgs::msg::Transform> available{{0, sample}};
    if (i % 4 != 0) {available[1] = make_transform(0.1*i, 0, 1, 0.1, -0.1, 0.2);}
    p.run({0, 99, 1, 0}, available, {i + 1, static_cast<std::uint32_t>(i * 100)});
  }
}

int main()
{
  try {
    test_pose_and_history();
    test_inputs_and_lookup();
    test_sequences();
    std::cout << "PASS: " << frames << " frames match original pose outputs and localization history.\n"
              << "PASS: observation IDs/order/precision, TF query arguments and missing-TF handling.\n"
              << "PASS: translation, rotation, unknown/repeated tags, zero/nonzero times, covariance, jumps, and frame rejection.\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
