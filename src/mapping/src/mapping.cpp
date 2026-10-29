#include "mapping/mapping.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <utility>
#include <limits>

#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/core.hpp"
#include "rclcpp/qos.hpp"

namespace mapping
{

Mapping::Mapping()
: Node("mapping"),
  fx_(0.0),
  cx_(0.0),
  camera_info_received_(false),
  resolution_(0.05),
  max_range_(8.0),
  width_(200),
  height_(200),
  origin_x_(-5.0),
  origin_y_(0.0),
  free_update_(std::log(0.2 / 0.8)),
  occupied_update_(std::log(0.7 / 0.3)),
  log_odds_limit_(5.0),
  pixel_step_(4),
  row_fraction_(0.5)
{
    const auto sensor_qos = rclcpp::SensorDataQoS();

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/depth/image_rect_raw",
        sensor_qos,
        std::bind(
            &Mapping::depth_callback,
            this,
            std::placeholders::_1));

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera/depth/camera_info",
        sensor_qos,
        std::bind(
            &Mapping::camera_info_callback,
            this,
            std::placeholders::_1));

    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/occupancy_grid",
        rclcpp::QoS(1).transient_local());

    log_odds_.assign(width_ * height_, 0.0);
    observed_.assign(width_ * height_, false);

    RCLCPP_INFO(
        get_logger(),
        "Mapping node started. Waiting for depth images and camera info.");
}

void Mapping::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    fx_ = msg->k[0];
    cx_ = msg->k[2];

    camera_info_received_ = (fx_ > 0.0);
}

void Mapping::depth_callback(
    const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (!camera_info_received_) {
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(get_logger(), "Depth conversion failed: %s", e.what());
        return;
    }

    const cv::Mat & depth_image = cv_ptr->image;

    if (depth_image.empty() || depth_image.channels() != 1) {
        RCLCPP_WARN(get_logger(), "Received an invalid depth image.");
        return;
    }

    const int row = std::clamp(
        static_cast<int>(row_fraction_ * depth_image.rows),
        0,
        depth_image.rows - 1);

    std::vector<std::pair<double, double>> measurements;

    for (int u = 0; u < depth_image.cols; u += pixel_step_) {

        double z = 0.0;

        if (depth_image.type() == CV_16UC1) {
            const uint16_t depth_mm =
                depth_image.at<uint16_t>(row, u);

            z = depth_mm / 1000.0;
        }
        else if (depth_image.type() == CV_32FC1) {
            z = depth_image.at<float>(row, u);
        }
        else {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Unsupported depth image encoding.");
            return;
        }

        if (!std::isfinite(z) || z <= 0.0 || z > max_range_) {
            continue;
        }

        const double x = (u - cx_) * z / fx_;
        const double y = z;

        measurements.emplace_back(x, y);
    }

    update_grid(measurements);

    publish_grid();
}

void Mapping::publish_grid()
{
    nav_msgs::msg::OccupancyGrid grid;

    grid.header.stamp = now();
    grid.header.frame_id = "scan_plane";

    grid.info.resolution = static_cast<float>(resolution_);
    grid.info.width = static_cast<uint32_t>(width_);
    grid.info.height = static_cast<uint32_t>(height_);

    grid.info.origin.position.x = origin_x_;
    grid.info.origin.position.y = origin_y_;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(width_ * height_, -1);

    for (int i = 0; i < width_ * height_; ++i) {
        if (!observed_[i]) {
            grid.data[i] = -1;
            continue;
        }

        const double probability =
            1.0 / (1.0 + std::exp(-log_odds_[i]));

        grid.data[i] = static_cast<int8_t>(
            std::clamp(std::lround(probability * 100.0), 0L, 100L));
    }

    grid_pub_->publish(grid);
}

void mapping::Mapping::update_grid(
    const std::vector<std::pair<double, double>>& measurements)
{

    for (int row = 0; row < height_; ++row) {
        for (int col = 0; col < width_; ++col) {

            const int index = row * width_ + col;

            const double cell_x = origin_x_ + (col + 0.5) * resolution_;

            const double cell_y = origin_y_ + (row + 0.5) * resolution_;

            double best_probability = -1.0;
            double best_angle_error = std::numeric_limits<double>::infinity();

            for (const auto& measurement : measurements) {

                const double measured_x = measurement.first;
                const double measured_y = measurement.second;

                const double probability = inverse_sensor_model(
                    cell_x, cell_y, measured_x, measured_y);

                if (probability < 0.0) {
                    continue;
                }

                const double angle_error = std::abs(std::atan2(
                    cell_x * measured_y - cell_y * measured_x,
                    cell_x * measured_x + cell_y * measured_y));

                if (angle_error < best_angle_error) {
                    best_angle_error = angle_error;
                    best_probability = probability;
                }
            }

            if (best_probability < 0.0) {
                continue;
            }

            const double measurement_log_odds = std::log(
                best_probability / (1.0 - best_probability));

            log_odds_[index] += measurement_log_odds;

            log_odds_[index] = std::clamp(
                log_odds_[index],
                -log_odds_limit_,
                log_odds_limit_);

            observed_[index] = true;
        }
    }
}

double mapping::Mapping::inverse_sensor_model(double cell_x, double cell_y, double measured_x, double measured_y)
{
    const double range = std::hypot(measured_x, measured_y);

    if (range <= 0.0) {
        return -1.0;
    }

    const double along_ray = (cell_x * measured_x + cell_y * measured_y) / range;

    const double cross = cell_x * measured_y - cell_y * measured_x;

    const double perpendicular_distance = std::abs(cross) / range;

    const double beam_width = std::max(
        resolution_ / 2.0,
        range * pixel_step_ / (2.0 * fx_));

    const double alpha = 2.0 * resolution_;

    if (along_ray < 0.0 || along_ray > range + alpha / 2.0 || perpendicular_distance > beam_width)
    {
        return -1.0;
    }

    if (std::abs(range - along_ray) <= alpha / 2.0) {
        return 0.7;
    }

    if (along_ray < range - alpha / 2.0) {
        return 0.3;
    }

    return -1.0;
}

}  // namespace mapping

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mapping::Mapping>());
    rclcpp::shutdown();
    return 0;
}