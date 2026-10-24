#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <thread>
#include <sstream>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>

class TestMapPublisher : public rclcpp::Node
{
public:
    TestMapPublisher()
    : Node("test_map_publisher")
    {
        map_publisher_ =
            create_publisher<nav_msgs::msg::OccupancyGrid>(
                "/map",
                rclcpp::QoS(1).transient_local().reliable());

        goal_publisher_ =
            create_publisher<geometry_msgs::msg::Pose2D>(
                "/goal",
                rclcpp::QoS(1).transient_local().reliable());

        try
        {
            package_share_directory_ =
                ament_index_cpp::get_package_share_directory(
                    "navigation_server");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Could not find navigation_server package: %s",
                e.what());
            throw;
        }

        RCLCPP_INFO(get_logger(), "Test map publisher ready.");
        RCLCPP_INFO(
            get_logger(),
            "Maps are loaded from: %s/maps",
            package_share_directory_.c_str());
        RCLCPP_INFO(get_logger(), "Command format: map1 200 200");
        RCLCPP_INFO(get_logger(), "Type 'help' for commands.");

        input_thread_ = std::thread(
            &TestMapPublisher::input_loop,
            this);
    }

    ~TestMapPublisher()
    {
        running_ = false;

        if (input_thread_.joinable())
        {
            input_thread_.detach();
        }
    }

private:
    static std::string next_token(
        std::istream &stream,
        bool &is_v2)
    {
        std::string token;

        while (stream >> token)
        {
            if (!token.empty() && token[0] == '#')
            {
                std::string comment;
                std::getline(stream, comment);

                if (comment.find("UR_MAP_V2:") != std::string::npos)
                {
                    is_v2 = true;
                }

                continue;
            }

            return token;
        }

        throw std::runtime_error("Unexpected end of PGM file");
    }

    nav_msgs::msg::OccupancyGrid load_map(
        const std::string &map_name)
    {
        const std::string map_path =
            package_share_directory_ +
            "/maps/" +
            map_name +
            ".pgm";

        RCLCPP_INFO(
            get_logger(),
            "Loading %s",
            map_path.c_str());

        std::ifstream image_file(map_path, std::ios::binary);

        if (!image_file)
        {
            throw std::runtime_error("Could not open " + map_path);
        }

        bool is_v2 = false;

        const std::string magic = next_token(image_file, is_v2);

        if (magic != "P2" && magic != "P5")
        {
            throw std::runtime_error(
                "Unsupported PGM format: " + magic);
        }

        const int width = std::stoi(next_token(image_file, is_v2));
        const int height = std::stoi(next_token(image_file, is_v2));
        const int max_value = std::stoi(next_token(image_file, is_v2));

        if (width <= 0 || height <= 0)
        {
            throw std::runtime_error("Invalid PGM dimensions.");
        }

        if (max_value <= 0 || max_value > 255)
        {
            throw std::runtime_error(
                "Only 8-bit PGM maps are supported.");
        }

        std::vector<int> pixels(
            static_cast<size_t>(width) *
            static_cast<size_t>(height));

        if (magic == "P2")
        {
            for (int &pixel : pixels)
            {
                pixel = std::stoi(next_token(image_file, is_v2));

                if (pixel < 0 || pixel > max_value)
                {
                    throw std::runtime_error(
                        "PGM pixel value is out of range.");
                }
            }
        }
        else
        {
            // P5 uses binary pixels. Consume the whitespace
            // separator after max_value.
            char separator;
            image_file.get(separator);

            if (!image_file ||
                (separator != ' ' && separator != '\n' &&
                 separator != '\r' && separator != '\t'))
            {
                throw std::runtime_error(
                    "Invalid P5 PGM header separator.");
            }

            // Handle CRLF headers.
            if (separator == '\r' && image_file.peek() == '\n')
            {
                image_file.get();
            }

            for (int &pixel : pixels)
            {
                const int value = image_file.get();

                if (value == std::char_traits<char>::eof())
                {
                    throw std::runtime_error(
                        "Unexpected end of PGM pixel data.");
                }

                pixel = static_cast<unsigned char>(value);

                if (pixel > max_value)
                {
                    throw std::runtime_error(
                        "PGM pixel value exceeds max_value.");
                }
            }
        }

        nav_msgs::msg::OccupancyGrid map;

        map.header.frame_id = "map";

        map.info.resolution = 1.0;
        map.info.width = static_cast<uint32_t>(width);
        map.info.height = static_cast<uint32_t>(height);

        map.info.origin.position.x = 0.0;
        map.info.origin.position.y = 0.0;
        map.info.origin.position.z = 0.0;

        map.info.origin.orientation.x = 0.0;
        map.info.origin.orientation.y = 0.0;
        map.info.origin.orientation.z = 0.0;
        map.info.origin.orientation.w = 1.0;

        map.data.reserve(pixels.size());

        /*
         * New editor format (UR_MAP_V2):
         *   0       = occupied
         *   1..253  = grayscale cost
         *   254     = free
         *   255     = unknown
         *
         * Older maps:
         *   black   = occupied
         *   white   = free
         *   gray    = intermediate cost
         *
         * OccupancyGrid stores signed int8 values. We explicitly
         * convert unknown to -1 so it is not confused with an
         * ordinary unsigned cost value.
         *
         * Flip the vertical direction because PGM rows start at
         * the top, while the map convention starts at the bottom.
         */

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                const int pixel =
                    pixels[
                        static_cast<size_t>(x) +
                        static_cast<size_t>(height - 1 - y) *
                        static_cast<size_t>(width)];

                int occupancy;

                if (is_v2)
                {
                    if (pixel == 255)
                    {
                        occupancy = -1;       // Unknown
                    }
                    else if (pixel == 254)
                    {
                        occupancy = 0;        // Free
                    }
                    else if (pixel == 0)
                    {
                        occupancy = 254;      // Occupied
                    }
                    else
                    {
                        // Preserve grayscale costs in the
                        // editor's custom encoding.
                        occupancy = 254 - pixel;
                    }
                }
                else
                {
                    // Legacy maps: black is occupied and white
                    // is free. Avoid producing -1 from a
                    // fully occupied pixel.
                    if (pixel == 0)
                    {
                        occupancy = 254;
                    }
                    else
                    {
                        occupancy = max_value - pixel;
                    }
                }

                map.data.push_back(
                    static_cast<int8_t>(occupancy));
            }
        }

        RCLCPP_INFO(
            get_logger(),
            "Loaded %s: %d x %d (%s format)",
            map_name.c_str(),
            width,
            height,
            is_v2 ? "UR_MAP_V2" : "legacy");

        return map;
    }

    void publish_map_and_goal(
        const std::string &map_name,
        double goal_x,
        double goal_y)
    {
        try
        {
            nav_msgs::msg::OccupancyGrid map = load_map(map_name);
            map.header.stamp = get_clock()->now();

            geometry_msgs::msg::Pose2D goal;
            goal.x = goal_x;
            goal.y = goal_y;
            goal.theta = 0.0;

            map_publisher_->publish(map);

            rclcpp::sleep_for(std::chrono::milliseconds(100));

            goal_publisher_->publish(goal);

            RCLCPP_INFO(
                get_logger(),
                "Published %s with goal (%.2f, %.2f)",
                map_name.c_str(),
                goal_x,
                goal_y);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to publish map: %s",
                e.what());
        }
    }

    void input_loop()
    {
        std::string line;

        while (running_ && rclcpp::ok())
        {
            std::cout << "> ";
            std::cout.flush();

            if (!std::getline(std::cin, line))
            {
                break;
            }

            if (line.empty())
            {
                continue;
            }

            if (line == "help")
            {
                std::cout << "\nCommands:\n"
                          << "  map1 200 200\n"
                          << "      Load map1.pgm and publish goal (200, 200)\n\n"
                          << "  map2 150 200\n"
                          << "      Load map2.pgm and publish goal (150, 200)\n\n"
                          << "  quit\n"
                          << "      Exit the program\n\n";
                continue;
            }

            if (line == "quit" || line == "exit")
            {
                rclcpp::shutdown();
                break;
            }

            std::stringstream command(line);

            std::string map_name;
            double goal_x;
            double goal_y;

            if (!(command >> map_name >> goal_x >> goal_y))
            {
                std::cout << "Invalid command. Use: map1 200 200\n";
                continue;
            }

            std::string extra;

            if (command >> extra)
            {
                std::cout << "Too many arguments. Use: map1 200 200\n";
                continue;
            }

            if (map_name.rfind("map", 0) != 0)
            {
                std::cout << "Map name must start with 'map'.\n";
                continue;
            }

            publish_map_and_goal(map_name, goal_x, goal_y);
        }
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        map_publisher_;

    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr
        goal_publisher_;

    std::string package_share_directory_;
    std::thread input_thread_;
    bool running_ = true;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TestMapPublisher>();

    rclcpp::spin(node);

    node.reset();

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }

    return 0;
}