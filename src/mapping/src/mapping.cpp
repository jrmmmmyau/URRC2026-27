#include "mapping/mapping.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>

#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/core.hpp"
#include "rclcpp/qos.hpp"

namespace mapping {

Mapping::Mapping()
: Node("mapping"),
  fx_(0.0),
  cx_(0.0),
  resolution_(0.05),
  max_range_(8.0),
  width_(200),
  height_(200),
  origin_x_(-5.0),
  origin_y_(0.0),
  max_confidence_(20.0),
  pixel_step_(4),
  row_fraction_(0.5),
  map_(width_ * height_)
{
    const auto sensor_qos = rclcpp::SensorDataQoS();

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/depth/image_rect_raw", sensor_qos,
        std::bind(&Mapping::depth_callback, this, std::placeholders::_1));

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera/depth/camera_info", sensor_qos,
        std::bind(&Mapping::camera_info_callback, this, std::placeholders::_1));

    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/occupancy_grid", rclcpp::QoS(1).transient_local());

    RCLCPP_INFO(get_logger(), "Mapping started. Waiting for depth image and camera info.");
}

void Mapping::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    fx_ = msg->k[0];
    cx_ = msg->k[2];
}

void Mapping::depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (fx_ <= 0.0) {
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
        return;
    }

    if (depth_image.type() != CV_16UC1 && depth_image.type() != CV_32FC1) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000, "Unsupported depth image format.");
        return;
    }

    const int row = std::clamp(
        static_cast<int>(row_fraction_ * depth_image.rows),
        0, depth_image.rows - 1);

    for (int u = 0; u < depth_image.cols; u += pixel_step_) {
        double z;
        if (depth_image.type() == CV_16UC1) {
            z = static_cast<double>(depth_image.at<uint16_t>(row, u)) / 1000.0;
        } else {
            z = static_cast<double>(depth_image.at<float>(row, u));
        }

        if (!std::isfinite(z) || z <= 0.0 || z > max_range_) {
            continue;
        }

        const double x = (u - cx_) * z / fx_;
        const double y = z;
        trace_ray(x, y);
    }

    publish_grid();
}

void Mapping::fuse_cell(int row, int col, double observed_danger)
{
    if (row < 0 || row >= height_ || col < 0 || col >= width_) {
        return;
    }

    MapCell & cell = map_[row * width_ + col];
    cell.danger =
        (cell.confidence * cell.danger + observed_danger) / (cell.confidence + 1.0);
    cell.confidence = std::min(cell.confidence + 1.0, max_confidence_);
}

void Mapping::trace_ray(double end_x, double end_y)
{
    // Camera is at (0, 0). Clip rays to the fixed map bounds.
    const double map_min_x = origin_x_;
    const double map_min_y = origin_y_;
    const double map_max_x = origin_x_ + width_ * resolution_;
    const double map_max_y = origin_y_ + height_ * resolution_;

    const bool endpoint_inside =
        end_x >= map_min_x && end_x < map_max_x &&
        end_y >= map_min_y && end_y < map_max_y;

    double scale = 1.0;
    if (end_x > 0.0 && end_x >= map_max_x) {
        scale = std::min(scale, std::nextafter(map_max_x, map_min_x) / end_x);
    } else if (end_x < 0.0 && end_x < map_min_x) {
        scale = std::min(scale, map_min_x / end_x);
    }
    if (end_y > 0.0 && end_y >= map_max_y) {
        scale = std::min(scale, std::nextafter(map_max_y, map_min_y) / end_y);
    }

    const double target_x = end_x * scale;
    const double target_y = end_y * scale;
    const int end_col = std::clamp(
        static_cast<int>(std::floor((target_x - origin_x_) / resolution_)), 0, width_ - 1);
    const int end_row = std::clamp(
        static_cast<int>(std::floor((target_y - origin_y_) / resolution_)), 0, height_ - 1);

    int col = static_cast<int>(std::floor(-origin_x_ / resolution_));
    int row = static_cast<int>(std::floor(-origin_y_ / resolution_));

    const int step_x = (end_x > 0.0) ? 1 : (end_x < 0.0 ? -1 : 0);
    const int step_y = (end_y > 0.0) ? 1 : (end_y < 0.0 ? -1 : 0);

    const double infinity = std::numeric_limits<double>::infinity();
    const double t_delta_x = step_x ? resolution_ / std::abs(end_x) : infinity;
    const double t_delta_y = step_y ? resolution_ / std::abs(end_y) : infinity;

    const double next_x = origin_x_ + (col + (step_x > 0 ? 1 : 0)) * resolution_;
    const double next_y = origin_y_ + (row + (step_y > 0 ? 1 : 0)) * resolution_;
    double t_max_x = step_x ? next_x / end_x : infinity;
    double t_max_y = step_y ? next_y / end_y : infinity;

    // 2D DDA: move to whichever grid boundary the ray reaches next.
    while (row >= 0 && row < height_ && col >= 0 && col < width_) {
        if (row == end_row && col == end_col) {
            fuse_cell(row, col, endpoint_inside ? 100.0 : 0.0);
            break;
        }

        fuse_cell(row, col, 0.0);

        if (t_max_x < t_max_y) {
            col += step_x;
            t_max_x += t_delta_x;
        } else if (t_max_y < t_max_x) {
            row += step_y;
            t_max_y += t_delta_y;
        } else {
            col += step_x;
            row += step_y;
            t_max_x += t_delta_x;
            t_max_y += t_delta_y;
        }
    }
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
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(map_.size());
    for (size_t i = 0; i < map_.size(); ++i) {
        grid.data[i] = (map_[i].confidence == 0.0)
            ? -1
            : static_cast<int8_t>(std::clamp(std::lround(map_[i].danger), 0L, 100L));
    }
    grid_pub_->publish(grid);
}

}  // namespace mapping

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mapping::Mapping>());
    rclcpp::shutdown();
    return 0;
}
