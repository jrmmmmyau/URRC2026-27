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

        RCLCPP_INFO(
            get_logger(),
            "Test map publisher ready.");

        RCLCPP_INFO(
            get_logger(),
            "Maps are loaded from: %s/maps",
            package_share_directory_.c_str());

        RCLCPP_INFO(
            get_logger(),
            "Command format: map1 200 200");

        RCLCPP_INFO(
            get_logger(),
            "Type 'help' for commands.");

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

    static std::string next_token(std::istream &stream)
    {
        std::string token;

        while (stream >> token)
        {
            if (!token.empty() && token[0] == '#')
            {
                std::string ignored;
                std::getline(stream, ignored);
                continue;
            }

            return token;
        }

        throw std::runtime_error(
            "Unexpected end of PGM file");
    }

    nav_msgs::msg::OccupancyGrid load_map(
        const std::string &map_name)
    {
        std::string map_path =
            package_share_directory_ +
            "/maps/" +
            map_name +
            ".pgm";

        RCLCPP_INFO(
            get_logger(),
            "Loading %s",
            map_path.c_str());

        std::ifstream image_file(
            map_path,
            std::ios::binary);

        if (!image_file)
        {
            throw std::runtime_error(
                "Could not open " + map_path);
        }

        std::string magic = next_token(image_file);

        if (magic != "P2" && magic != "P5")
        {
            throw std::runtime_error(
                "Unsupported PGM format: " + magic);
        }

        const int width =
            std::stoi(next_token(image_file));

        const int height =
            std::stoi(next_token(image_file));

        const int max_value =
            std::stoi(next_token(image_file));

        if (max_value > 255)
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
                pixel =
                    std::stoi(next_token(image_file));
            }
        }
        else
        {
            // Move past the whitespace following max_value.
            image_file.get();

            for (int &pixel : pixels)
            {
                pixel =
                    static_cast<unsigned char>(
                        image_file.get());
            }
        }

        nav_msgs::msg::OccupancyGrid map;

        map.header.frame_id = "map";

        map.info.resolution = 1.0;
        map.info.width = width;
        map.info.height = height;

        map.info.origin.position.x = 0.0;
        map.info.origin.position.y = 0.0;
        map.info.origin.position.z = 0.0;

        map.info.origin.orientation.x = 0.0;
        map.info.origin.orientation.y = 0.0;
        map.info.origin.orientation.z = 0.0;
        map.info.origin.orientation.w = 1.0;

        map.data.reserve(
            static_cast<size_t>(width) *
            static_cast<size_t>(height));

        /*
         * PGM:
         *
         *   white = 255
         *   black = 0
         *
         * Our OccupancyGrid:
         *
         *   0   = definitely free
         *   255 = definitely occupied
         *
         * Therefore we invert the grayscale value.
         *
         * We also flip the vertical direction because
         * PGM images start at the top while the OccupancyGrid
         * convention starts at the bottom.
         */

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                const int pixel =
                    pixels[
                        static_cast<size_t>(x) +
                        static_cast<size_t>(height - 1 - y) *
                        static_cast<size_t>(width)];

                const int occupancy =
                    max_value - pixel;

                map.data.push_back(
                    static_cast<int8_t>(occupancy));
            }
        }

        RCLCPP_INFO(
            get_logger(),
            "Loaded %s: %d x %d",
            map_name.c_str(),
            width,
            height);

        return map;
    }

    void publish_map_and_goal(
        const std::string &map_name,
        double goal_x,
        double goal_y)
    {
        try
        {
            nav_msgs::msg::OccupancyGrid map =
                load_map(map_name);

            map.header.stamp =
                get_clock()->now();

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
                std::cout << "\nCommands:\n";
                std::cout << "  map1 200 200\n";
                std::cout << "      Load map1.pgm and publish "
                          << "goal (200, 200)\n\n";

                std::cout << "  map2 150 200\n";
                std::cout << "      Load map2.pgm and publish "
                          << "goal (150, 200)\n\n";

                std::cout << "  quit\n";
                std::cout << "      Exit the program\n\n";

                continue;
            }

            if (line == "quit" ||
                line == "exit")
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
                std::cout << "Invalid command.\n";
                std::cout << "Use: map1 200 200\n";
                continue;
            }

            std::string extra;

            if (command >> extra)
            {
                std::cout << "Too many arguments.\n";
                std::cout << "Use: map1 200 200\n";
                continue;
            }

            if (map_name.rfind("map", 0) != 0)
            {
                std::cout <<
                    "Map name must start with 'map'.\n";
                continue;
            }

            publish_map_and_goal(
                map_name,
                goal_x,
                goal_y);
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

    auto node =
        std::make_shared<TestMapPublisher>();

    rclcpp::spin(node);

    node.reset();

    rclcpp::shutdown();

    return 0;
}