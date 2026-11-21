#include <cstdint>
#include <exception>
#include <memory>
#include <stdexcept>

#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/core.hpp"
#include "perception/depth_processing.hpp"
#include "perception/observation_cloud.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace perception {

class DepthObserver : public rclcpp::Node {
public:
  DepthObserver() : Node("depth_observer")
  {
    settings_.pixel_step = declare_parameter<int>("pixel_step", 4);
    settings_.row_fraction = declare_parameter<double>("row_fraction", 0.5);
    settings_.max_range = declare_parameter<double>("max_range", 8.0);
    validate_settings(settings_);

    const auto qos = rclcpp::SensorDataQoS();
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("depth/points", qos);
    calibration_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "depth/camera_info", qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {calibration_ = msg;});
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "depth/image", qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_depth(msg);});
    RCLCPP_INFO(get_logger(), "Waiting for matching depth images and camera calibration.");
  }

private:
  void on_depth(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if (!calibration_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for camera calibration.");
      return;
    }
    if (msg->header.frame_id.empty() ||
      calibration_->header.frame_id != msg->header.frame_id ||
      calibration_->width != msg->width || calibration_->height != msg->height)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Depth image and calibration must have matching dimensions and optical frame.");
      return;
    }
    if (msg->encoding != "16UC1" && msg->encoding != "32FC1") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Unsupported depth encoding: %s", msg->encoding.c_str());
      return;
    }

    try {
      const auto image = cv_bridge::toCvShare(msg, msg->encoding);
      const cv::Mat & depth = image->image;
      if (depth.empty() || depth.channels() != 1 ||
        (depth.type() != CV_16UC1 && depth.type() != CV_32FC1))
      {
        throw std::invalid_argument("Depth image is empty or has an unsupported layout.");
      }
      const CameraIntrinsics camera{
        calibration_->k[0], calibration_->k[4], calibration_->k[2], calibration_->k[5]};
      const auto points = sample_depth(
        depth.cols, depth.rows, camera, settings_,
        [&depth](int row, int column) -> double {
          if (depth.type() == CV_16UC1) {
            return static_cast<double>(depth.at<std::uint16_t>(row, column)) / 1000.0;
          }
          return static_cast<double>(depth.at<float>(row, column));
        });
      sensor_msgs::msg::PointCloud2 cloud;
      write_observation_cloud(points, msg->header, cloud);
      publisher_->publish(cloud);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Skipping depth image: %s", error.what());
    }
  }

  DepthSettings settings_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr calibration_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr calibration_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

}  // namespace perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception::DepthObserver>());
  rclcpp::shutdown();
  return 0;
}
