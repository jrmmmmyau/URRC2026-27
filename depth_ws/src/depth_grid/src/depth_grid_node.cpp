#include <algorithm>
#include <utility>
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
        x_min_ = -2.0;
        z_min_ = 0.0;
        width_ = 80;
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

    void markRay(std::vector<int8_t> & grid, int x0, int y0, int x1, int y1)
    {
	const int dx = std::abs(x1 - x0);
	const int dy = std::abs(y1 - y0);

	const int sx = (x0 < x1) ? 1 : -1;
	const int sy = (y0 < y1) ? 1 : -1;

	int error = dx - dy;
	int x = x0;
	int y = y0;

	while (x != x1 || y != y1) 
	{
	    if (x >= 0 && x < width_ && y >= 0 && y < height_)
	    {
		const size_t index = static_cast<size_t>(y) * width_ + x;
	
		if (grid[index] == -1) 
		{
		    grid[index] = 100;
		}
	    }

	    const int twice_error = 2 * error;
	    if (twice_error > -dy) {
	    	error -= dy;
		x += sx;
	    }

	    if (twice_error < dx) {
	    	error += dx;
		y += sy;
	    }
	}
    }

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

	if (depth.empty()) return;

   


        std::vector<int8_t> grid(static_cast<size_t>(width_) * height_, -1);
   
        //RCLCPP_INFO(get_logger(), "checkpoint 2: grid init");

    	// camera origin projected onto depth grid
	const int origin_col = static_cast<int>(std::floor((0.0 - x_min_) / resolution_));

	const int origin_row = static_cast<int>(std::floor((0.0 - z_min_) / resolution_));

	const double x_max = x_min_ + width_ * resolution_;
	const double z_max = z_min_ + height_ * resolution_;


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

		    if (x < x_min_ || x >= x_max ||
			z < z_min_ || z >= z_max) {
		    	continue;
		    }

                    const int col = static_cast<int>(
                        std::floor((x - x_min_) / resolution_));
                    const int grid_row = static_cast<int>(
                        std::floor((z - z_min_) / resolution_));

                    if (col < 0 || col >= width_ || 
                        grid_row < 0 || grid_row >= height_) {
                        continue;
                    }

                    markRay(grid, origin_col, origin_row, col, grid_row);

		    const size_t endpoint = static_cast<size_t>(grid_row) * width_ + col;
		    
		    // map dist to grayscale
		    const int shade = static_cast<int>(
				    std::clamp((z / 4.0) * 100.0, 0.0, 100.0)
		    );
		    grid[endpoint] = static_cast<int8_t>(shade);
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

