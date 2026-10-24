#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/core.hpp"

class DepthGridNode : public rclcpp::Node
{
public:
    DepthGridNode()
    : Node("depth_grid_node")
    {
        // straight from echo camera info
        fx_ = 385.47924804685;
        fy_ = 385.47924804685;
        cx_ = 323.978668218906;
        cy_ = 241.0266876220703;

        resolution_ = 0.05;
        x_min_ = -1.0;
        z_min_ = 0.0;
        width_ = 40;
        height_ = 80;

        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/depth/image_rect_raw",
            rclcpp::SensorDataQoS(),
            std::bind(
                &DepthGridNode::depthCallback,
                this,
                std::placeholders::_1));

        grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/occupancy_grid", 10);

        RCLCPP_INFO(
            get_logger(),
            "Depth occupancy grid node started");
    }

private:
    void depthCallback(
        const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, "16UC1");
        } catch (const cv_bridge::Exception & e) {
            RCLCPP_ERROR(
                get_logger(),
                "could not convert depth image: %s",
                e.what());
            return;
        }

        const cv::Mat & depth = cv_ptr->image;
        if (depth.empty()) {
            return;
        }

        std::vector<int8_t> grid(
            static_cast<size_t>(width_) * height_, -1);
            for (int v = 0; v < depth.rows; v += 4) {
                const uint16_t * row =
                    depth.ptr<uint16_t>(v);

                for (int u = 0; u < depth.cols; u += 4) {
                    const uint16_t raw_depth = row[u];

                    if (raw_depth == 0) {
                        continue;
                    }

                    const double z = static_cast<double>(raw_depth) / 1000.0;

                    if (!std::isfinite(z) || z < 0.15 || z > 4.0) {
                        continue;
                    }

                    const double x = (static_cast<double>(u) - cx_) * z / fx_;

                    const int col = static_cast<int>(
                        std::floor((x - x_min_) / resolution_));
                    const int grid_row = static_cast<int>(
                        std::floor((z - z_min_) / resolution_));

                    if (col < 0 || col >= width_ || 
                        grid_row < 0 || grid_row >= height_) {
                        continue;
                    }

                    grid[
                        static_cast<size_t>(grid_row) * width_ + col
                    ] = 100;
                }
            }

            nav_msgs::msg::OccupancyGrid out;
            out.header.stamp = msg->header.stamp;
            out.header.frame_id = "camera_depth_optical_frame";

            out.info.resolution = resolution_;
            out.info.width = width_;
            out.info.height = height_;

            out.info.origin.position.x = x_min_;
            out.info.origin.position.y = z_min_;
            out.info.origin.position.z = 0.0;
            out.info.origin.orientation.w = 1.0;

            out.data = std::move(grid);

            grid_pub_->publish(out);

    }

    double fx_, fy_, cx_, cy_;
    double resolution_, x_min_, z_min_;
    int width_, height_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
        depth_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        grid_pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<DepthGridNode>());

    rclcpp::shutdown();
    return 0;
}

