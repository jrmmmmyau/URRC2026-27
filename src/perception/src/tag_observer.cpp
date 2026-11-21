#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "perception/msg/tag_observation_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace perception {

class TagObserver : public rclcpp::Node {
public:
  TagObserver() : Node("tag_observer")
  {
    output_frame_ = declare_parameter<std::string>("output_frame", "base_link");
    tag_prefix_ = declare_parameter<std::string>("tag_frame_prefix", "tag_");
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.1);
    if (output_frame_.empty() || !std::isfinite(tf_timeout_) || tf_timeout_ < 0.0) {
      throw std::invalid_argument("Set a nonempty output_frame and a finite nonnegative tf_timeout.");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    publisher_ = create_publisher<perception::msg::TagObservationArray>("tag/observations", 10);
    subscription_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "detections", 10,
      [this](apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr msg) {
        on_detections(msg);
      });
  }

private:
  void on_detections(const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr & msg)
  {
    perception::msg::TagObservationArray out;
    out.header = msg->header;
    out.header.frame_id = output_frame_;
    out.observations.reserve(msg->detections.size());
    for (const auto & detection : msg->detections) {
      const std::string tag_frame = tag_prefix_ + std::to_string(detection.id);
      try {
        // Preserve the original lookup direction, measurement time, and wait.
        // As before, a zero timestamp requests TF's latest available transform.
        const auto transform = tf_buffer_->lookupTransform(
          output_frame_, tag_frame, rclcpp::Time(msg->header.stamp),
          rclcpp::Duration::from_seconds(tf_timeout_));
        perception::msg::TagObservation observation;
        observation.id = detection.id;
        observation.transform = transform.transform;
        out.observations.push_back(observation);
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Can't look up %s -> %s: %s", output_frame_.c_str(), tag_frame.c_str(), error.what());
      }
    }
    publisher_->publish(out);
  }

  std::string output_frame_;
  std::string tag_prefix_;
  double tf_timeout_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr subscription_;
  rclcpp::Publisher<perception::msg::TagObservationArray>::SharedPtr publisher_;
};

}  // namespace perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception::TagObserver>());
  rclcpp::shutdown();
  return 0;
}
