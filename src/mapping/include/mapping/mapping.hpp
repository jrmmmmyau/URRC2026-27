#ifndef MAPPING__MAPPING_HPP_
#define MAPPING__MAPPING_HPP_

#include <memory>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace mapping
{

class Mapping : public rclcpp::Node
{
public:
    Mapping();

private:
    void camera_info_callback(
        const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    void point_cloud_callback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void depth_callback(
        const sensor_msgs::msg::Image::SharedPtr msg);
    
    void update_grid(const std::vector<int8_t>& evidence);

    void publish_grid();

    double inverse_sensor_model(
        double cell_x,
        double cell_y,
        double measured_x,
        double measured_y);


    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

    double fx_;
    double cx_;
    bool camera_info_received_;

    double resolution_;
    double max_range_;
    int width_;
    int height_;
    double origin_x_;
    double origin_y_;

    std::vector<double> log_odds_;
    std::vector<bool> observed_;

    double free_update_;
    double occupied_update_;
    double log_odds_limit_;

    int pixel_step_;
    double row_fraction_;
};

}  // namespace mapping

#endif  