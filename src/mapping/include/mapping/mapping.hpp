#pragma once

#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace mapping {

struct MapCell {
    double danger = 0.0;
    double confidence = 0.0;
};

class Mapping : public rclcpp::Node {
public:
    Mapping();

private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void trace_ray(double end_x, double end_y);
    void fuse_cell(int row, int col, double observed_danger);
    void publish_grid();

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

    double fx_;
    double cx_;
    double resolution_;
    double max_range_;
    int width_;
    int height_;
    double origin_x_;
    double origin_y_;
    double max_confidence_;
    int pixel_step_;
    double row_fraction_;

    std::vector<MapCell> map_;
};

}  // namespace mapping
