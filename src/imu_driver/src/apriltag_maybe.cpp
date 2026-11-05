#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// Turns AprilTag detections into "where is the robot in the world".
//
//   map -> base_link  =  (map -> tag)  *  (base_link -> tag)^-1
//
//   map -> tag        : known, from the YAML (where we put each tag)
//   base_link -> tag  : measured, looked up from tf2
//                       (camera mount TF + what apriltag_ros saw)
class TagLocalizer : public rclcpp::Node
{
public:
  TagLocalizer() : Node("tag_localizer")
  {
    // ---- Settings (from the YAML) ----
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    tag_prefix_ = declare_parameter<std::string>("tag_frame_prefix", "tag36h11:");

    const double pos_std = declare_parameter<double>("position_stddev", 0.05);  // meters
    const double ang_std = declare_parameter<double>("angle_stddev", 0.05);     // radians
    pos_var_ = pos_std * pos_std;
    ang_var_ = ang_std * ang_std;

    // ---- Where each tag is in the world (one list entry per tag) ----
    const auto ids = declare_parameter<std::vector<int64_t>>("tag_ids", std::vector<int64_t>{});
    const auto xs = declare_parameter<std::vector<double>>("tag_x", std::vector<double>{});
    const auto ys = declare_parameter<std::vector<double>>("tag_y", std::vector<double>{});
    const auto zs = declare_parameter<std::vector<double>>("tag_z", std::vector<double>{});
    const auto rolls = declare_parameter<std::vector<double>>("tag_roll", std::vector<double>{});
    const auto pitches = declare_parameter<std::vector<double>>("tag_pitch", std::vector<double>{});
    const auto yaws = declare_parameter<std::vector<double>>("tag_yaw", std::vector<double>{});

    const size_t n = ids.size();
    if (xs.size() != n || ys.size() != n || zs.size() != n ||
      rolls.size() != n || pitches.size() != n || yaws.size() != n)
    {
    RCLCPP_WARN(get_logger(), "All tag_* lists in the YAML must have the same length.");
      throw std::runtime_error("bad tag map");
    }

    for (size_t i = 0; i < n; ++i) {
      tf2::Quaternion q;
      q.setRPY(rolls[i], pitches[i], yaws[i]);
      tag_in_map_[ids[i]] = tf2::Transform(q, tf2::Vector3(xs[i], ys[i], zs[i]));
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu tag(s) into the map.", n);

    // ---- tf2: lets us ask "where is X relative to Y?" ----
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---- Input and output ----
    sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "detections", 10,
      [this](const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
        on_detections(msg);
      });

    pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("tag_pose", 10);
  }

private:
  void on_detections(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    for (const auto & det : msg->detections) {
      // Skip tags we don't know the world position of
      const auto it = tag_in_map_.find(det.id);
      if (it == tag_in_map_.end()) {
        continue;
      }
      const tf2::Transform & map_to_tag = it->second;

      // Step 1: ask tf2 where the tag is relative to the robot
      const std::string tag_frame = tag_prefix_ + std::to_string(det.id);
      geometry_msgs::msg::TransformStamped base_to_tag_msg;
      try {
        base_to_tag_msg = tf_buffer_->lookupTransform(
          base_frame_, tag_frame, rclcpp::Time(msg->header.stamp),
          rclcpp::Duration::from_seconds(0.1));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Can't look up %s -> %s: %s", base_frame_.c_str(), tag_frame.c_str(), ex.what());
        continue;
      }
      tf2::Transform base_to_tag;
      tf2::fromMsg(base_to_tag_msg.transform, base_to_tag);

      // Step 2: map -> base_link = (map -> tag) * (base_link -> tag)^-1
      const tf2::Transform map_to_base = map_to_tag * base_to_tag.inverse();

      // Step 3: publish the robot's pose in the world
      geometry_msgs::msg::PoseWithCovarianceStamped out;
      out.header.stamp = msg->header.stamp;
      out.header.frame_id = map_frame_;
      tf2::toMsg(map_to_base, out.pose.pose);

      // How much to trust this (6x6 grid, stored row by row; diagonal only)
      out.pose.covariance[0] = pos_var_;    // x
      out.pose.covariance[7] = pos_var_;    // y
      out.pose.covariance[14] = pos_var_;   // z
      out.pose.covariance[21] = ang_var_;   // roll
      out.pose.covariance[28] = ang_var_;   // pitch
      out.pose.covariance[35] = ang_var_;   // yaw

      pub_->publish(out);
    }
  }

  std::string map_frame_, base_frame_, tag_prefix_;
  double pos_var_{0.0}, ang_var_{0.0};
  std::unordered_map<int64_t, tf2::Transform> tag_in_map_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TagLocalizer>());
  rclcpp::shutdown();
  return 0;
}