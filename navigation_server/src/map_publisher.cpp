#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

class MapPublisher : public rclcpp::Node
{
public:
  MapPublisher()
  : Node("map_publisher")
  {
    // Load the PGM map
    std::ifstream image_file(
      "/home/wajdar/ros2_ws/URRC2026-27/navigation_server/maps/map2.pgm",
      std::ios::binary);
    if (!image_file) {
      throw std::runtime_error("Could not open map2.pgm");
    }

    bool is_v2 = false;
    std::string magic = next_token(image_file, is_v2);
    if (magic != "P2" && magic != "P5") {
      throw std::runtime_error("Unsupported PGM format");
    }
    const int width = std::stoi(next_token(image_file, is_v2));
    const int height = std::stoi(next_token(image_file, is_v2));
    const int max_value = std::stoi(next_token(image_file, is_v2));
    std::vector<int> pixels(static_cast<size_t>(width) * static_cast<size_t>(height));

    if (magic == "P2") {
      for (int &pixel : pixels) {
        pixel = std::stoi(next_token(image_file, is_v2));
      }
    } else {
      image_file.get();
      if (max_value < 256) {
        for (int &pixel : pixels) {
          pixel = static_cast<unsigned char>(image_file.get());
        }
      } else {
        for (int &pixel : pixels) {
          const int high = static_cast<unsigned char>(image_file.get());
          const int low = static_cast<unsigned char>(image_file.get());
          pixel = (high << 8) | low;
        }
      }
    }

    RCLCPP_INFO(get_logger(), "Loaded map: %d x %d", width, height);

    // Create the OccupancyGrid message
    map_.header.frame_id = "map";

    // Map metadata
    map_.info.resolution = 1.0;
    map_.info.width = width;
    map_.info.height = height;

    // Origin of the map
    map_.info.origin.position.x = 0.0;
    map_.info.origin.position.y = 0.0;
    map_.info.origin.position.z = 0.0;

    map_.info.origin.orientation.w = 1.0;

    // Convert pixels to occupancy values
    map_.data.clear();
    map_.data.reserve(pixels.size());

    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        const int pixel = pixels[static_cast<size_t>(x) +
          static_cast<size_t>(height - 1 - y) * static_cast<size_t>(width)];

        if (is_v2 && pixel == 255) {
          map_.data.push_back(-1);
        } else {
          const double pixel_max = is_v2 ? 254.0 : static_cast<double>(max_value);
          const int occupancy = static_cast<int>(std::lround(
            100.0 * (pixel_max - pixel) / pixel_max));
          map_.data.push_back(static_cast<int8_t>(occupancy));
        }
      }
    }

    RCLCPP_INFO(get_logger(), "Created occupancy grid with %zu cells", map_.data.size());

    // Publisher
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    // Publish the map periodically
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0), [this]() {publish_map();});
  }

private:
  static std::string next_token(std::istream &stream, bool &is_v2)
  {
    std::string token;
    while (stream >> token) {
      if (!token.empty() && token[0] == '#') {
        std::string ignored;
        std::getline(stream, ignored);
        if (ignored.find("UR_MAP_V2:") != std::string::npos) {
          is_v2 = true;
        }
        continue;
      }
      return token;
    }
    throw std::runtime_error("Unexpected end of PGM file");
  }

  void publish_map()
  {
    map_.header.stamp = get_clock()->now();
    publisher_->publish(map_);
  }

  nav_msgs::msg::OccupancyGrid map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapPublisher>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
